# ADAS Camera Geometry Toolkit

A study-driven computer vision toolkit focused on the geometric foundations behind
ADAS, autonomous driving, and robotics perception systems.

This repository documents and implements the core camera geometry concepts used in
visual navigation pipelines: coordinate frames, rigid-body transformations, camera
projection models, calibration, and the bridge between mathematical theory and
production-oriented perception code.

## Why This Project Exists

Modern ADAS and robotics systems depend on precise spatial reasoning. A camera
measurement only becomes useful when it can be connected to a vehicle frame, a world
frame, a map, another sensor, or a downstream planning module.

This project is my structured learning path for becoming an AI engineer specialized
in computer vision for ADAS, autonomous systems, and robotics. The goal is to build
both:

- clear technical notes that explain the geometry behind perception systems
- tested Python implementations of the same concepts
- calibration and projection utilities that can grow into practical perception
  tooling

## Current Focus

This week focuses on geometric basics and camera calibration knowledge before moving
deeper into implementation.

The learning sequence is:

1. Visual navigation foundations
2. Coordinate frames and rigid-body transformations
3. Camera models and projection geometry
4. Intrinsic and extrinsic calibration
5. Python implementation with tests
6. Practical ADAS-style examples

## Study Notes

The `docs/` folder contains the theory notes that guide the implementation work.

| Topic | Note | Status |
| --- | --- | --- |
| Visual navigation basics | [docs/visual_navigation.md](docs/visual_navigation.md) | In progress |
| Camera projection model | [docs/camera_models.md](docs/camera_models.md) | In progress |
| Calibration fundamentals | Planned | Upcoming |

## Planned Implementation

The coding section will translate the notes into reusable Python modules.

Planned components:

- `src/geometry/transform.py`: rotation matrices, homogeneous transforms, pose
  composition, inverse transforms, and frame conversions
- `configs/frames.yaml`: example vehicle, camera, and world frame definitions
- `tests/`: unit tests for transformation correctness and numerical stability
- calibration utilities for intrinsic/extrinsic camera parameters
- projection and back-projection examples for image-to-world reasoning

## Skills Demonstrated

This repository is designed to show practical readiness for computer vision and
robotics engineering work:

- 3D coordinate frames and rigid-body transformations
- camera intrinsic and extrinsic parameterization
- pinhole camera projection
- rotation representations: matrices, Euler angles, axis-angle, and quaternions
- calibration concepts used in ADAS camera pipelines
- clean technical documentation
- test-driven implementation of numerical geometry code

## Repository Structure

```text
.
|-- configs/              # Frame and calibration configuration examples
|-- docs/                 # Study notes and technical explanations
|   |-- camera_models.md
|   `-- visual_navigation.md
|-- src/                  # Geometry and camera tooling implementation
|   `-- geometry/
|       `-- transform.py
|-- tests/                # Unit tests for geometry modules
|-- requirements.txt      # Python dependencies
`-- readme.md
```

## Roadmap

- [x] Start visual navigation notes
- [x] Add camera model notes
- [ ] Expand geometric basics: frames, poses, SE(3), SO(3)
- [ ] Add camera calibration notes
- [ ] Implement transformation utilities
- [ ] Add projection and back-projection functions
- [ ] Add calibration examples with synthetic data
- [ ] Add unit tests and numerical validation
- [ ] Build small ADAS-style demos such as lane-frame projection or bird's-eye view mapping

## References

- MIT OpenCourseWare, 16.485 Visual Navigation for Autonomous Vehicles
- Multiple View Geometry in Computer Vision, Hartley and Zisserman
- Computer Vision: Algorithms and Applications, Richard Szeliski

## About

This is a long-term study and portfolio project for computer vision engineering in
ADAS, autonomous driving, and robotics. The project emphasizes mathematical
correctness, readable implementation, and clear communication of technical ideas.
