from pathlib import Path
import re

import cv2
import numpy as np


def image_number(path):
    """Sort Im_L_2 before Im_L_10."""
    return int(re.search(r"(\d+)$", path.stem).group(1))


def calibrate_camera(image_dir, board_size=(11, 7), square_size=30.0):
    image_paths = sorted(
        Path(image_dir).glob("*.png"),
        key=image_number,
    )

    # Coordinates of the 77 inner corners in the board coordinate system:
    # (0,0,0), (30,0,0), ..., assuming square_size is 30.
    template_points = np.zeros(
        (board_size[0] * board_size[1], 3),
        dtype=np.float32,
    )
    template_points[:, :2] = (
        np.mgrid[0:board_size[0], 0:board_size[1]]
        .T.reshape(-1, 2)
    )
    template_points[:, :2] *= square_size

    object_points = []  # Known 3D board coordinates
    image_points = []   # Detected 2D pixel coordinates
    used_images = []

    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        50,
        1e-4,
    )

    image_size = None

    for path in image_paths:
        image = cv2.imread(str(path))
        if image is None:
            print(f"Could not read: {path}")
            continue

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        image_size = gray.shape[::-1]  # (width, height)

        found, corners = cv2.findChessboardCorners(
            gray,
            board_size,
            flags=(
                cv2.CALIB_CB_ADAPTIVE_THRESH
                | cv2.CALIB_CB_NORMALIZE_IMAGE
            ),
        )

        if not found:
            print(f"Chessboard not detected: {path.name}")
            continue

        corners = cv2.cornerSubPix(
            gray,
            corners,
            winSize=(11, 11),
            zeroZone=(-1, -1),
            criteria=criteria,
        )

        object_points.append(template_points.copy())
        image_points.append(corners)
        used_images.append(path)

    if image_size is None:
        raise RuntimeError(f"No readable images in {image_dir}")

    if len(object_points) < 5:
        raise RuntimeError(
            f"Only {len(object_points)} usable images; more board poses are needed"
        )

    rms, camera_matrix, distortion, rvecs, tvecs = (
        cv2.calibrateCamera(
            object_points,
            image_points,
            image_size,
            None,
            None,
        )
    )

    # Calculate mean reprojection error.
    errors = []

    for obj, detected, rvec, tvec in zip(
        object_points, image_points, rvecs, tvecs
    ):
        projected, _ = cv2.projectPoints(
            obj,
            rvec,
            tvec,
            camera_matrix,
            distortion,
        )

        error = cv2.norm(
            detected,
            projected,
            cv2.NORM_L2,
        ) / len(projected)

        errors.append(error)

    return {
        "rms": rms,
        "mean_error": float(np.mean(errors)),
        "per_image_errors": np.asarray(errors),
        "camera_matrix": camera_matrix,
        "distortion": distortion,
        "rvecs": rvecs,
        "tvecs": tvecs,
        "used_images": used_images,
        "image_size": image_size,
    }


data_root = Path("data/kaggle_calib_data/data")

left = calibrate_camera(
    data_root / "imgs/leftcamera",
    board_size=(11, 7),
    square_size=30.0,
)

right = calibrate_camera(
    data_root / "imgs/rightcamera",
    board_size=(11, 7),
    square_size=30.0,
)

print("\nLeft camera matrix:")
print(left["camera_matrix"])
print("Left distortion:", left["distortion"])
print("Left RMS error:", left["rms"])
print("Left mean error:", left["mean_error"])

print("\nRight camera matrix:")
print(right["camera_matrix"])
print("Right distortion:", right["distortion"])
print("Right RMS error:", right["rms"])
print("Right mean error:", right["mean_error"])

np.save(data_root / "left_intrinsic.npy", left["camera_matrix"])
np.save(data_root / "left_distortion.npy", left["distortion"])
np.save(data_root / "right_intrinsic.npy", right["camera_matrix"])
np.save(data_root / "right_distortion.npy", right["distortion"])