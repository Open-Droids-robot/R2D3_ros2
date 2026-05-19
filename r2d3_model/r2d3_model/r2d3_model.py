"""R2D3 Model — ROS 2 Lifecycle Node hosting a participant policy.

Modeled on intrinsic-dev/aic's aic_model.py:
  https://github.com/intrinsic-dev/aic/blob/main/aic_model/aic_model/aic_model.py

The host owns the ROS 2 graph. Participants implement a Python policy
class (see :mod:`r2d3_model.policy`) and pass its module name as the
``policy`` ROS parameter. The host imports the module at configure-time
and looks for a class with the same name as the last dotted segment.

State machine (per ROS 2 Managed Nodes spec):

    Unconfigured ──configure──▶ Inactive ──activate──▶ Active
        ▲                          │                       │
        │                          deactivate              │
        │                          ◀───────────────────────┘
        │                          │
        │                       cleanup
        └──────────────────────────┘

Anti-cheat: while in Unconfigured / Inactive the host MUST NOT publish
any robot commands. The engine enforces this.
"""

from __future__ import annotations

import importlib
import inspect
import threading
from typing import Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.lifecycle import (
    LifecycleNode,
    LifecycleState,
    TransitionCallbackReturn,
)
from std_srvs.srv import Empty
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# These imports fail when ROS 2 is not present (e.g. running on the host
# Python). They will resolve inside the Humble container once the
# r2d3_task_interfaces and r2d3_model_interfaces packages are built.
from r2d3_task_interfaces.action import RunTask  # noqa: E402
from r2d3_model_interfaces.msg import Observation  # noqa: E402
from rm_ros_interfaces.msg import Gripperset, Liftheight, Movej, Movejp  # noqa: E402


class R2D3Model(LifecycleNode):
    """Lifecycle Node hosting a policy that responds to RunTask actions."""

    POLICY_PARAM = "policy"
    DEFAULT_POLICY_MODULE = "r2d3_model.example_policies.idle"

    # ------------------------------------------------------------------ init
    def __init__(self) -> None:
        super().__init__("r2d3_model")

        self.declare_parameter(self.POLICY_PARAM, self.DEFAULT_POLICY_MODULE)

        self._policy_class = None
        self._policy = None
        self._latest_observation: Optional[Observation] = None
        self._observation_lock = threading.Lock()
        self._action_callback_group = ReentrantCallbackGroup()
        self._action_thread: Optional[threading.Thread] = None
        self._goal_handle: Optional[ServerGoalHandle] = None

        # TF buffer (handy for participants reading object poses post-grasp)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(
            buffer=self._tf_buffer, node=self, spin_thread=True
        )

        # Observation subscription is always active; the host buffers the
        # latest message and the policy reads from `_latest_observation`.
        self.observation_sub = self.create_subscription(
            Observation, "/r2d3/observations", self._observation_callback, 10
        )

        # Cancel service (engine calls this to abort a trial)
        self.cancel_service = self.create_service(
            Empty, "cancel_task", self._cancel_task_callback
        )

        # Per-arm + per-gripper + lift command publishers — only published
        # to from the Active state (anti-cheat: nothing moves before activate).
        self._cmd_pubs = {
            ("movej", "left"): self.create_lifecycle_publisher(
                Movej, "/left_arm_controller/rm_driver/movej_cmd", 5
            ),
            ("movej", "right"): self.create_lifecycle_publisher(
                Movej, "/right_arm_controller/rm_driver/movej_cmd", 5
            ),
            ("movej_p", "left"): self.create_lifecycle_publisher(
                Movejp, "/left_arm_controller/rm_driver/movej_p_cmd", 5
            ),
            ("movej_p", "right"): self.create_lifecycle_publisher(
                Movejp, "/right_arm_controller/rm_driver/movej_p_cmd", 5
            ),
            ("gripper", "left"): self.create_lifecycle_publisher(
                Gripperset,
                "/left_arm_controller/rm_driver/set_gripper_position_cmd",
                5,
            ),
            ("gripper", "right"): self.create_lifecycle_publisher(
                Gripperset,
                "/right_arm_controller/rm_driver/set_gripper_position_cmd",
                5,
            ),
            ("lift",): self.create_lifecycle_publisher(
                Liftheight,
                "/left_arm_controller/rm_driver/set_lift_height_cmd",
                5,
            ),
        }

        # Action server (always running so the engine can discover it;
        # goal rejection in Inactive state enforces the anti-cheat rule).
        self.action_server = ActionServer(
            self,
            RunTask,
            "/run_task",
            execute_callback=self._run_task_execute_callback,
            goal_callback=self._run_task_goal_callback,
            cancel_callback=self._run_task_cancel_callback,
            callback_group=self._action_callback_group,
        )

    # ---------------------------------------------------- lifecycle hooks
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"on_configure({state.label})")

        policy_module_name = (
            self.get_parameter(self.POLICY_PARAM).get_parameter_value().string_value
        )
        self.get_logger().info(f"loading policy module: {policy_module_name}")
        try:
            policy_module = importlib.import_module(policy_module_name)
        except Exception as e:  # noqa: BLE001
            self.get_logger().fatal(
                f"unable to load policy module {policy_module_name}: {e}"
            )
            return TransitionCallbackReturn.FAILURE

        expected_class_name = policy_module_name.rsplit(".", 1)[-1]
        # capitalize convention: module 'wave_arm' -> class 'WaveArm'? aic does
        # exact-match. Stick with exact-match for clarity.
        for class_name, klass in inspect.getmembers(policy_module, inspect.isclass):
            if class_name == expected_class_name:
                self._policy_class = klass
                break

        if self._policy_class is None:
            self.get_logger().fatal(
                f"class {expected_class_name!r} not found in module "
                f"{policy_module_name!r}"
            )
            return TransitionCallbackReturn.FAILURE

        try:
            self._policy = self._policy_class()
            self._policy.on_configure()
        except Exception as e:  # noqa: BLE001
            self.get_logger().fatal(f"policy.on_configure failed: {e}")
            return TransitionCallbackReturn.FAILURE

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"on_activate({state.label})")
        try:
            self._policy.on_activate()
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"policy.on_activate failed: {e}")
            return TransitionCallbackReturn.FAILURE
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"on_deactivate({state.label})")
        try:
            self._policy.on_deactivate()
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"policy.on_deactivate failed: {e}")
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"on_cleanup({state.label})")
        if self._policy is not None:
            try:
                self._policy.on_cleanup()
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f"policy.on_cleanup failed: {e}")
        self._policy = None
        self._policy_class = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"on_shutdown({state.label})")
        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------ observation
    def _observation_callback(self, msg: Observation) -> None:
        with self._observation_lock:
            self._latest_observation = msg

    # --------------------------------------------------------- cancel
    def _cancel_task_callback(self, request, response):  # noqa: ANN001
        if self._goal_handle is not None and self._goal_handle.is_active:
            self._goal_handle.canceled()
        return response

    # --------------------------------------------------------- action
    def _run_task_goal_callback(self, goal_request):  # noqa: ANN001
        # Reject goals unless in Active state (engine validates this).
        if self._current_state_label() != "active":
            self.get_logger().warn(
                f"rejecting RunTask goal: node is {self._current_state_label()!r}"
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _run_task_cancel_callback(self, goal_handle):  # noqa: ANN001
        return CancelResponse.ACCEPT

    def _run_task_execute_callback(self, goal_handle: ServerGoalHandle):
        self._goal_handle = goal_handle
        goal = goal_handle.request
        self.get_logger().info(
            f"RunTask: id={goal.task_id!r} seed={goal.task_seed} trial={goal.trial_index}"
        )

        # Notify the policy of the new task.
        try:
            self._policy.on_task(goal.task_id, goal.task_seed, goal.trial_index)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"policy.on_task raised: {e}")
            result = RunTask.Result()
            result.success = False
            result.message = f"policy.on_task raised: {e}"
            goal_handle.abort()
            return result

        # Main step loop — runs until the action is cancelled by the engine
        # (engine cancels when its own goal-predicate monitor signals success
        # or time_limit_seconds elapses).
        feedback = RunTask.Feedback()
        feedback.phase = "running"
        feedback.progress = -1.0
        result = RunTask.Result()

        try:
            while rclpy.ok() and goal_handle.is_active:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.message = "cancelled by engine"
                    return result

                with self._observation_lock:
                    obs = self._latest_observation
                if obs is None:
                    continue

                cmd = self._policy.step(obs)
                if cmd is not None:
                    self._publish_command(cmd)

                goal_handle.publish_feedback(feedback)

            # Reaching here means the action was deactivated externally.
            goal_handle.succeed()
            result.success = True
            result.message = "task loop exited cleanly"
            return result
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"step loop raised: {e}")
            result.success = False
            result.message = f"step loop raised: {e}"
            goal_handle.abort()
            return result
        finally:
            self._goal_handle = None

    # --------------------------------------------------------- helpers
    def _publish_command(self, cmd: tuple) -> None:
        """Dispatch a policy-returned command tuple to the right publisher."""
        kind = cmd[0]
        if kind == "movej":
            _, side, joint_targets = cmd
            msg = Movej()
            msg.joint = list(joint_targets)
            self._cmd_pubs[("movej", side)].publish(msg)
        elif kind == "movej_p":
            _, side, pose = cmd
            # pose is expected to be a list/tuple compatible with Movejp;
            # leave detailed packing to the participant utility helpers.
            self._cmd_pubs[("movej_p", side)].publish(pose)
        elif kind == "gripper":
            _, side, position = cmd
            msg = Gripperset()
            msg.position = int(position)
            msg.block = False
            self._cmd_pubs[("gripper", side)].publish(msg)
        elif kind == "lift":
            _, height_mm, speed_pct = cmd
            msg = Liftheight()
            msg.height = int(height_mm)
            msg.speed = int(speed_pct)
            msg.block = False
            self._cmd_pubs[("lift",)].publish(msg)
        else:
            self.get_logger().warn(f"unknown command kind: {kind!r}")

    def _current_state_label(self) -> str:
        # rclpy.lifecycle's LifecycleNode exposes the current label via
        # _state_machine on most versions; fall back to a sentinel.
        try:
            return self._state_machine.current_state[1]
        except Exception:  # noqa: BLE001
            return "unknown"


def main(args=None) -> None:
    rclpy.init(args=args)
    node = R2D3Model()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
