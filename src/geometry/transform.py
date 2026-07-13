import numpy as np


def make_transform(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Create 4x4 homogeneous transform from rotation R and translation t. T = [R|t]"""
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t, dtype=np.float64).reshape(3)
    return T


def invert_transform(T_ab: np.ndarray) -> np.ndarray:
    """Invert transform T_ab into T_ba. T_ba = [R_ab^T | -R_ab^T t_ab]"""
    R_ab = T_ab[:3, :3]
    t_ab = T_ab[:3, 3]

    T_ba = np.eye(4, dtype=np.float64)
    T_ba[:3, :3] = R_ab.T
    T_ba[:3, 3] = -R_ab.T @ t_ab
    return T_ba


def compose_transform(T_ab: np.ndarray, T_bc: np.ndarray) -> np.ndarray:
    """Compose T_ab and T_bc into T_ac. T_ac = T_ab * T_bc"""
    return T_ab @ T_bc


def transform_points(T_ab: np.ndarray, points_b: np.ndarray) -> np.ndarray:
    """Transform Nx3 points from frame b to frame a. p_a = T_ab @ p_b"""
    points_b = np.asarray(points_b, dtype=np.float64)
    ones = np.ones((points_b.shape[0], 1), dtype=np.float64)
    points_b_h = np.hstack([points_b, ones])

    points_a_h = (T_ab @ points_b_h.T).T
    return points_a_h[:, :3]


def rot_x(angle_rad: float) -> np.ndarray:
    """Create rotation matrix for rotation around x-axis."""
    c = np.cos(angle_rad)
    s = np.sin(angle_rad)
    return np.array([
        [1, 0, 0],
        [0, c, -s],
        [0, s, c],
    ], dtype=np.float64)


def rot_y(angle_rad: float) -> np.ndarray:
    c = np.cos(angle_rad)
    s = np.sin(angle_rad)
    return np.array([
        [c, 0, s],
        [0, 1, 0],
        [-s, 0, c],
    ], dtype=np.float64)


def rot_z(angle_rad: float) -> np.ndarray:
    c = np.cos(angle_rad)
    s = np.sin(angle_rad)
    return np.array([
        [c, -s, 0],
        [s, c, 0],
        [0, 0, 1],
    ], dtype=np.float64)