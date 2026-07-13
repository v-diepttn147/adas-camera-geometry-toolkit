import numpy as np

from src.geometry.transform import (
    make_transform,
    invert_transform,
    compose_transform,
    transform_points,
    rot_z,
)


def test_transform_inverse_is_identity():
    R = rot_z(np.deg2rad(30.0))
    t = np.array([1.0, 2.0, 0.5])

    T_ab = make_transform(R, t)
    T_ba = invert_transform(T_ab)

    I = T_ab @ T_ba

    np.testing.assert_allclose(I, np.eye(4), atol=1e-9)


def test_transform_points_roundtrip():
    R = rot_z(np.deg2rad(45.0))
    t = np.array([2.0, -1.0, 0.3])

    T_ab = make_transform(R, t)
    T_ba = invert_transform(T_ab)

    points_b = np.array([
        [1.0, 0.0, 0.0],
        [2.0, 1.0, 0.0],
        [0.0, -1.0, 1.0],
    ])

    points_a = transform_points(T_ab, points_b)
    points_b_recovered = transform_points(T_ba, points_a)

    np.testing.assert_allclose(points_b_recovered, points_b, atol=1e-9)


def test_compose_transform():
    T_world_vehicle = make_transform(np.eye(3), np.array([10.0, 0.0, 0.0]))
    T_vehicle_camera = make_transform(np.eye(3), np.array([1.0, 0.0, 1.0]))

    T_world_camera = compose_transform(T_world_vehicle, T_vehicle_camera)

    p_camera = np.array([[0.0, 0.0, 0.0]])
    p_world = transform_points(T_world_camera, p_camera)

    expected = np.array([[11.0, 0.0, 1.0]])
    np.testing.assert_allclose(p_world, expected, atol=1e-9)
