# Architecture

> Placeholder — populate as the system takes shape.

## Components (planned)

```
┌──────────────────────────────────────────┐
│  Host (Ubuntu 22.04/24.04 + NVIDIA GPU)  │
│                                          │
│  ┌────────────────┐  ┌────────────────┐  │
│  │ Isaac Sim 5.1  │◄─┤ ROS2 Humble    │  │
│  │  container     │  │  container     │  │
│  │  (Docker/isaac)│  │  (Docker/docker│  │
│  │                │  │   /Dockerfile  │  │
│  │  ┌──────────┐  │  │   .humble)     │  │
│  │  │ r2d3_sim │  │  │                │  │
│  │  └──────────┘  │  │  MoveIt2       │  │
│  │  ROS2 bridge ◄─┼──┼─►              │  │
│  └────────────────┘  └────────────────┘  │
└──────────────────────────────────────────┘
```

## Topics
- Joint state / command — ROS2 standard topics, mapped by the Isaac Sim ROS2 bridge
- Camera streams — D435 RGB + depth, published as `sensor_msgs/Image` and `PointCloud2`
- TF tree — published from Isaac, consumed by MoveIt2

## Open questions
- Whether to keep MoveIt2 in a separate container or bundle into the Humble container
- Whether the `r2d3-eval` CLI runs inside or outside the Isaac container
