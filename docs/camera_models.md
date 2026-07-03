# Camera Model

A 3D points in the world frame is projected into the image using:

```math
\begin{bmatrix} x \\ y \\ 1 \end{bmatrix} = K [R|t] \begin{bmatrix} X \\ Y \\ Z \\ 1 \end{bmatrix} 
```

where `K` contains the camera intrinsic parameters, `[R|t]` represents the camera pose relative to the world frame.

```math
K = \begin{bmatrix} fx & 0 & cx \\ 0 & fy & cy \\ 0 & 0 & 1 \end{bmatrix}
```

```
fx, fy:    focal length in pixel
cx, cy:    principal point
[R|t]:     extrinsic, transformation from world frame to camera frame
(u, v):    pixel coordinate
(X, Y, Z): 3D point
```
