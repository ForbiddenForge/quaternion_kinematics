import numpy as np
import pytest
import quaternion

from quaternion_kinematics import *


@pytest.fixture
def example_euler_angles():
    phi = 25 * np.pi / 180
    theta = 130 * np.pi / 180
    psi = 70 * np.pi / 180
    return np.array([phi, theta, psi])


def test_forward_and_back_euler_conversion(example_euler_angles):
    """Check that an inputted Euler angle can be converted to a quaternion and back as the same value"""
    euler_to_quat = euler_angles_to_quaternion(example_euler_angles)
    quat_to_euler = quaternion_to_euler(euler_to_quat)
    np.testing.assert_allclose(
        quat_to_euler,
        example_euler_angles,
        rtol=1e-07,
        atol=0,
        err_msg="The two arrays are not equal to 7 decimal places!",
    )
