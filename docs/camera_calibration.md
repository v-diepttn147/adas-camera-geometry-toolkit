# Camera Calibration

## Distortion

There are 2 major kinds of distortion, which are radial distortion and tangential distortion.

- Radial distortion: causes straight lines to appear curved. It becomes larger the farther points are from the center of the image. Radial distortion can be represented as follows:

```math
x_{distorted} = x(1 + k_1r^2 + k_2r^4 + k_3r^6)y_{distorted} = y(1 + k_1r^2 + k_2r^4 + k_3r^6)
```

- Tangential distortion: similar to radial distortion, it occurs because the image-taking lense is not aligned perfectly parallel to the imaging plane. So some areas in the image may look nearer than expected.

```math
x_{distorted} = x + [2p_1xy + p_2(r^2 + 2x^2)]y_{distorted} = y + [p_1(r^2 + 2y^2) + 2p_2xy]
```

In short, we need to find five parameters, known as distortion coefficients given by:

```math
Distortion\_coefficients = \begin{matrix} (k_1 & k_2 & p_1 & p_2 & k_3) \end{matrix}
```

In addition, we need some other information, such as intrinsic and extrinsic parameters of the camera.

- Intrinsic parameters are specific to a camera, including information like focal length $(f_x, f_y)$ and optical centers $(c_x, c_y)$. They can be used to create a camera matrix to remove distortion due to the lens and specific camera. The camera matrix is unique to a specific camera, once calculated, it can be reused on other images taken by the same camera. It is expressed as a 3x3 matrix:

```math
camera\_matrix = 
\begin{bmatrix}
f_x & 0 & c_x \\
0 & f_y & c_y \\
0 & 0 & 1
\end{bmatrix}
```

- Extrinsic parameters corresponds to rotation and translation vectors which translates a coordinate of 3D point to a coordinate system.

To find these parameters in a stereo camera system, we must provide some sample images of a well defined pattern (e.g. chessboard). We find some specific points of which we already know the relative positions (e.g. square corners in the chessboard). We know the coordinates of these points in real world space and we know the coordinates in the image, so we can solve and find the distortion coefficients.

## How to run intrinsic calibration

```bash
python src/calibrate/intrinsic_calibrate.py
```

### Input samples

|                             Left camera                              |                              Right camera                              |
| :------------------------------------------------------------------: | :--------------------------------------------------------------------: |
| ![Left cam](/data/kaggle_calib_data/data/imgs/leftcamera/Im_L_1.png) | ![Right cam](/data/kaggle_calib_data/data/imgs/rightcamera/Im_R_1.png) |

### Output

```bash
Left camera matrix:
[[718.91836354   0.         523.81856036]
 [  0.         729.96804306 285.67618074]
 [  0.           0.           1.        ]]
Left distortion: [[ 3.10666569e-02 -1.79722346e-01  8.74094727e-04  1.77154208e-04
   2.13751737e-01]]
Left RMS error: 0.18599515318463716
Left mean error: 0.021004377357422284

Right camera matrix:
[[722.05689897   0.         513.69527279]
 [  0.         732.73196496 292.73884   ]
 [  0.           0.           1.        ]]
Right distortion: [[ 0.01106685 -0.13689339 -0.00141557  0.0012989   0.20227914]]
Right RMS error: 0.19947302663008826
Right mean error: 0.022530154394968888
```

### Camera model

This code use standard pinhole camera model with a 5-parameter Brown-Conrady lens-distortion model.

1. Pinhole projection
A 3D point on the calibration board is first transformed into the camera coordinate system:

```math
\begin{bmatrix} X_c \\ Y_c \\ Z_c \end{bmatrix}
= R \begin{bmatrix} X_w \\ Y_w \\ Z_w \end{bmatrix} + t
```

It is then projected onto the normalized image plane:

```math
x = \frac{X_c}{Z_c}, y = \frac{Y_c}{Z_c}
```

Without lens distortion, this is an ideal pinhole camera

2. Lens distortion

The normalized coordinates are modified using 5 distortion coefficients:

```math
D = [k_1, k_2, p_1, p_2, k_3]
```

where:

- $k_1, k_2, k_3$: radial distortion
- $p_1, p_2$: tangential distortion

Let

```math
r^2 = x^2 + y^2
```

Then the distorted normalized coordinates are:

```math
x_d = x(1 + k_1r^2 + k_2r^4 + k_3r^6) + 2p_1xy + p_2(r^2 + 2x^2) \\
y_d = y(1 + k_1r^2 + k_2r^4 + k_3r^6) + p_1(r^2 + 2y^2) + 2p_2xy
```

3. Conversion to pixel coordinates

The intrinsic matrix converts the distorted normalized coordinates into pixels:

```math
K = 
\begin{bmatrix}
f_x & 0 & c_x \\
0 & f_y & c_y \\
0 & 0 & 1
\end{bmatrix}
```

therefore:

```math
u = f_xx_d + c_x \\
v = f_yy_d + c_y
```

4. Complete model

```math
pixel = K distort (\frac{RX_w + t}{Z_c})
```

## How to run extrinsic calibration
