#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <functional>
#include <vector>

namespace realsense2_camera
{   
    // We are making things compatible with ros2 Jazzy
    // Both Humble's OnParametersSetCallbackType and Jazzy's OnSetParametersCallbackType
    // resolve to this same function signature, so we use it directly for portability.
    using ParametersCallbackType = std::function<rcl_interfaces::msg::SetParametersResult(const std::vector<rclcpp::Parameter> &)>;

    class ParametersBackend
    {
        public:
            ParametersBackend(rclcpp::Node& node) : 
                _node(node),
                _logger(rclcpp::get_logger("RealSenseCameraNode"))
                {};
            ~ParametersBackend();
            void add_on_set_parameters_callback(ParametersCallbackType callback);


        private:
            rclcpp::Node& _node;
            rclcpp::Logger _logger;
            std::shared_ptr<void> _ros_callback;
    };
}