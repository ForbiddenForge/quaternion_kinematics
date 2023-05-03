import numpy as np
import pytest
import quaternion

from quaternion_kinematics import *


@pytest.fixture
def example_euler_angles():
    phi = 25 * np.pi / 180
    theta = 130 * np.pi / 180
    psi = 70 * np.pi / 180
    return np.array([phi,theta,psi])

def test_forward_and_back_euler_conversion(example_euler_angles):
    """Check that an inputted Euler angle can be converted to a quaternion and back as the same value"""
    euler_to_quat = euler_angles_to_quaternion(example_euler_angles)
    quat_to_euler = quaternion_to_euler(euler_to_quat)
    some_bs_euler = np.array([1.55,0.56,3.12])
    
    np.testing.assert_allclose(quat_to_euler, example_euler_angles, rtol=1e-09, atol=0, err_msg='The two arrays do not equal each other at 7 digits!')






