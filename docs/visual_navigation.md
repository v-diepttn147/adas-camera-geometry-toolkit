# Visual Navigation

## Coordinate Frames

- robot `r`
- each sensor (e.g. camera frame `c`)
- world frame `w`
- external bodies (e.g. other robots, objects)

## Points, positions and translations

We represent the position of a 3D point `p` in the world frame using a 3D vector:

```math
p^w = \begin{bmatrix} p^w_x \\ p^w_y \\ p^w_z \end{bmatrix}
```

## Attitute and rotations

### Rotation matrix representation

- 2D rotation matrix

```math
R^W_T = \begin{bmatrix} cos(\theta) & -sin(\theta) \\ sin(\theta) & cos(\theta) \end{bmatrix}
```

- 3D rotation matrix

```math
R^W_T = \begin{bmatrix} x^W_r & y^W_r & z^W_r \end{bmatrix}
```

The matrix $R^W_T$ is called rotation matrix, it's not a generic matrix, since its columns represent orthogonal unit-length axis to satisfy the right-hand rule. Therefore, any rotation matrix has to satisfy:

- orthogonality (trực giao): the axes $x^W_r, y^W_r, z^W_r$ have unit length and are orthogonal to each other.
- right-handedness: the 3D axes satisfy the right-hand rule, which means $x^W_r \times y^W_r = z^W_r$, where $\times$ denotes the cross product between vectors.

#### Operations involving rotations

- Express points in a rotated frame: given the coordinates of a point $p^r$ in frame `r`, the following computes the coordinates $p^W$ in frame `w`

```math
p^W = R^W_rp^r
```

- Rotation composition: given the attitude of a frame `r` with respect to a frame `w` namely $R^W_r$, and the attitude of a frame `c` with respect to a frame `r` namely $R^r_c$, the attitude of the frame `c` with respect to the frame `w` is computed as follow:

```math
R^W_c = R^W_rR^r_c
```

- Inverse a rotation:

```math
R^r_W = (R^W_r)^{-1} = (R^W_r)^T
```

### Elementary rotations and Euler angles representation

- Theorem 1 (Euler's rotation theorem). Any rotation can be written as the product of no more than three elementaty rotations, where the elementary rotations are defined as follows:

```math
R_x(\phi) = \begin{bmatrix} 1 & 0 & 0 \\ 0 & cos(\phi) & -sin(\phi) \\ 0 & sin(\phi) & cos(\phi) \end{bmatrix}

R_y(\theta) = \begin{bmatrix} cos(\theta) & 0 & sin(\theta) \\ 0 & 1 & 0 \\ -sin(\theta) & 0 & cos(\theta) \end{bmatrix}

R_y(\psi) = \begin{bmatrix}  cos(\psi) & -sin(\psi) & 0\\ sin(\psi) & cos(\psi) & 0\\ 0 & 0 & 1 \end{bmatrix}
```

Therefore, one can parameterize the same rotation &R^W_r$ as:

```math
R^W_r = R_z(\psi)R_y(\theta)R_x(\phi)
```

where $\psi$ is called yaw angle, $\theta$ is called pitch angle and $\phi$ is called the roll angle.

The RPY has a singularuty for pitch angle equal to $\pi/2$. This singularity is called **Gimbal lock**.

### Axis-angle representation

- Theorem 2 (Euler's rotation theorem). Every rotation can be described as a rotation of an angle $\theta$ around an axis $u$ with $||u||=1$, not neccessarily assigned with the Cartesian axes. This suggests the axis-angle representations, which encodes a 3D rotation using the pair $(u, \theta)$.

#### Conversions

- From axis-angle to rotation matrix (**Rodrigues' rotation formula**):

```math
R^W_r = cos(\theta)I_3 + sin(\theta)[u]_{\times} + (1-cos(\theta))uu^T
```

where

```math
[u]_{\times} = \begin{bmatrix} 0 & -u_z & u_y \\ u_z & 0 & -u_x \\ -u_y & u_x & 0 \end{bmatrix}
```

is a skew symmetric matrix (the cross product matrix).

### Quaternion representation

If 3-parameter representations are ideal for storage (but have singularities and requires trigonometry), rotation matrices are singularity-free (but largely over-parametrize the attitude), W.R. Hamilton provided a solution called **quaternion** representation. Depending on the context, a quaternion is denoted as a column vector

```math
q = \begin{bmatrix} q_1 \\ q_2 \\ q_3 \\ q_4 \end{bmatrix}
```

or as an ipercomplex number

```math
q = iq_1 + jq_2 + kq_3 + q_4
```

with $i, j, k$ satisfying:

```math
i^2 = j^2 = k^2 = ijk = -1 \\
ij = -ji = k \\
jk = -kj = i \\
ki = -ik = j
```

The rotation matrix corresponding to a unit quaternion $q$ is given by:

```math
R(q) = \begin{bmatrix}
        q_1^2 - q_2^2 - q_3^2 + q_4^2   & 2(q_1q_2 - q_3q_4)                & 2(q_1q_3 + q_2q_4)            \\
        2(q_1q_2 + q_3q_4)              & -q_1^2 + q_2^2 - q_3^2 + q_4^2    & 2(q_2q_3 - q_1q_4)            \\
        2(q_1q_3 - q_2q_4)              & 2(q_2q_3 + q_1q_4)                & -q_1^2 - q_2^2 + q_3^2 + q_4^2\\
        \end{bmatrix}
```

## Poses and rigid-body transformations

If we call $t_r^W$ the position of the origin of $r$ (attached to a body) with respect to frame $W$, then the pair:

```math
(R_r^W, t_r^W)
```

fully characterizes the geometry of $r$ with respect to $W$, or the **pose** of $r$ with respect to $W$. A pose is fully defined by 6 parameters (3 for translation and 3 for attitude).

```math
T_r^W = \begin{bmatrix} R_r^W & t_r^W \\ 0_d^T & 1 \end{bmatrix}
```

where d = 2 in 2D problems and 3 in 3D.

### Rigid-body transformations.

Assumes we are given the coordinates $p^r$ of a point $p$, expressed in the reference frame $r$ and that we know the relative pose of the reference frame $r$ with respect to the frame $W$, namely $T_r^W$. Then the position of point $p$ with respect to the frame $W$ is given by:

```math
p^W = R_r^Wp^r + t_r^W
```

### Pose composition

Lets assume we are given the pose of a frame $r$ with respect to a frame $W$, namely $T_r^W$, and the pose of a frame $c$ with respect to a frame $r$, namely $T_c^r$. The pose of frame $c$ with respect to the frame $W$ is described as follow:

```math
T_c^W = T_c^WT_c^r
```

### Inverse of a pose

```math
T_W^r = (T_r^W)^{-1} = \begin{bmatrix} (R_r^W)^T & -(R_r^W)^Tt_r^W \\ 0_3^T & 1 \end{bmatrix}
```

# References

- Massachusetts Institute of Technology. (2020). MIT 16.485 Visual Navigation for Autonomous Vehicles—LecNotes02/03 [Lecture notes]. MIT OpenCourseWare. https://ocw.mit.edu/courses/16-485-visual-navigation-for-autonomous-vehicles-vnav-fall-2020/resources/mit16_485f20_lec02and03/