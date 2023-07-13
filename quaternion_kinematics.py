import numpy as np
import quaternion

"""Extend the Quaternion module for use with kinematical equations, especially converting between
Euler Angles, Direction Cosine Matrices, and Quaternions. Useful in aerospace (rigid body) and 3D visualization.
Rotations are right-hand ruled and conversions between reference frames assumes going from NED Frame => Body Frame of vehicle."""


def quaternion_norm(qt):
    """Find Magnitude / Length / Norm of a quaternion. Takes a Quaternion as an argument"""
    return np.sqrt(qt.w**2 + qt.x**2 + qt.y**2 + qt.z**2)


def quaternion_inverse(qt):
    """Find the inverse of a quaternion. Takes a Quaternion as an argument"""
    norm = quaternion_norm(qt)
    conjugate = np.conjugate(qt)
    return conjugate / norm


def dcm_to_quaternion(dcm_matrix):
    """Convert from a Direction Cosine Matrix to its corresponding Quaternion. Note: The DCM
    must be representative of Cb/v, or a representation of the rotation from the NED frame to the vehicle body frame.
    """
    dcm_matrix = dcm_matrix
    # Assign position values to each index of the DCM
    dcm_position_11 = dcm_matrix[0, 0]
    dcm_position_12 = dcm_matrix[0, 1]
    dcm_position_13 = dcm_matrix[0, 2]
    dcm_position_21 = dcm_matrix[1, 0]
    dcm_position_22 = dcm_matrix[1, 1]
    dcm_position_23 = dcm_matrix[1, 2]
    dcm_position_31 = dcm_matrix[2, 0]
    dcm_position_32 = dcm_matrix[2, 1]
    dcm_position_33 = dcm_matrix[2, 2]

    def calculate_q_tilde():
        """Calculate the intermediate Q tilde Quaternion from the Direction Cosine Matrix"""
        qs_tilde = np.sqrt(
            0.25 * (1 + dcm_position_11 + dcm_position_22 + dcm_position_33)
        )
        qx_tilde = np.sqrt(
            0.25 * (1 + dcm_position_11 - dcm_position_22 - dcm_position_33)
        )
        qy_tilde = np.sqrt(
            0.25 * (1 - dcm_position_11 + dcm_position_22 - dcm_position_33)
        )
        qz_tilde = np.sqrt(
            0.25 * (1 - dcm_position_11 - dcm_position_22 + dcm_position_33)
        )
        return np.array([qs_tilde, qx_tilde, qy_tilde, qz_tilde])

    q_tilde = calculate_q_tilde()
    """Find the max value from the s, x, y, z components of the Q Tilde Quaternion. Note that this is currently
     an ndarray object in order to find the max value in a straightforward fashion."""
    q_max = np.max(q_tilde)

    def calculate_qt_from_q_max():
        """Use the qmax value to formulate the final quaternion."""
        # Breakup Q tilde into components
        qs_tilde, qx_tilde, qy_tilde, qz_tilde = q_tilde
        # Assign final component values of the DCM -> Quaternion conversion based on the max value
        # within q_tilde
        if q_max == qs_tilde:
            qs = qs_tilde
            qx = (dcm_position_23 - dcm_position_32) / (4 * qs_tilde)
            qy = (dcm_position_31 - dcm_position_13) / (4 * qs_tilde)
            qz = (dcm_position_12 - dcm_position_21) / (4 * qs_tilde)
        elif q_max == qx_tilde:
            qs = (dcm_position_23 - dcm_position_32) / (4 * qx_tilde)
            qx = qx_tilde
            qy = (dcm_position_12 - dcm_position_21) / (4 * qx_tilde)
            qz = (dcm_position_31 - dcm_position_13) / (4 * qx_tilde)
        elif q_max == qy_tilde:
            qs = (dcm_position_31 - dcm_position_13) / (4 * qy_tilde)
            qx = (dcm_position_12 + dcm_position_21) / (4 * qy_tilde)
            qy = qy_tilde
            qz = (dcm_position_23 + dcm_position_32) / (4 * qy_tilde)

        elif q_max == qz_tilde:
            qs = (dcm_position_12 - dcm_position_21) / (4 * qz_tilde)
            qx = (dcm_position_31 + dcm_position_13) / (4 * qz_tilde)
            qy = (dcm_position_23 + dcm_position_32) / (4 * qz_tilde)
            qz = qz_tilde
        return np.quaternion(qs, qx, qy, qz)

    dcm_to_quaternion_output = calculate_qt_from_q_max()
    return dcm_to_quaternion_output


def dcm_to_euler(dcm_matrix):
    """Extract Euler Angles from a Direction Cosine Matrix. Output is a numpy array of the form
    np.array([phi,theta,psi]). Current numpy and quaternion functions output these angles in reverse order and
    I find it annoying so I changed it. Due to the nature of arctan and arcsin, things can get a bit tricky tricky.
    The logic inserted is based on a paper by the famous Gregory G. Slabaugh entitled 'Computing Euler angles from
    a rotation matrix.'"""
    dcm_matrix = dcm_matrix
    # Assign position values to each index of the DCM
    dcm_position_11 = dcm_matrix[0, 0]
    dcm_position_12 = dcm_matrix[0, 1]
    dcm_position_13 = dcm_matrix[0, 2]
    dcm_position_21 = dcm_matrix[1, 0]
    dcm_position_22 = dcm_matrix[1, 1]
    dcm_position_23 = dcm_matrix[1, 2]
    dcm_position_31 = dcm_matrix[2, 0]
    dcm_position_32 = dcm_matrix[2, 1]
    dcm_position_33 = dcm_matrix[2, 2]
    phi = np.arctan2(dcm_position_23, dcm_position_33)
    # Raise exceptions if outside of arctan2 range of -pi to pi
    if not -np.pi <= phi <= np.pi:
        raise Exception(
            "Function dcm_to_euler failed; Arctan2 input(s) are not between -pi and pi!"
        )
    theta = -np.arcsin(dcm_position_13)
    psi = np.arctan2(dcm_position_12, dcm_position_11)
    if not -np.pi <= psi <= np.pi:
        raise Exception(
            "Function dcm_to_euler failed; Arctan2 input(s) are not between -pi and pi!"
        )
    phi = phi + np.pi
    theta = np.pi - theta
    psi = psi + np.pi
    return np.array([phi, theta, psi])


def quaternion_to_dcm(qt):
    """Convert from a Quaternion to a Direction Cosine Matrix. Takes a Quaternion object as argument.
    Note: Argument must be formed from np.quaternion(w,x,y,z). Np.array will not work since
    We're assuming you're already using the Quaternion module LOL."""

    # Setup the DCM positions and their corresponding equations
    dcm_position_11 = qt.w**2 + qt.x**2 - qt.y**2 - qt.z**2
    dcm_position_12 = 2 * (qt.x * qt.y + qt.z * qt.w)
    dcm_position_13 = 2 * (qt.x * qt.z - qt.y * qt.w)
    dcm_position_21 = 2 * (qt.x * qt.y - qt.z * qt.w)
    dcm_position_22 = qt.w**2 - qt.x**2 + qt.y**2 - qt.z**2
    dcm_position_23 = 2 * (qt.y * qt.z + qt.x * qt.w)
    dcm_position_31 = 2 * (qt.x * qt.z + qt.y * qt.w)
    dcm_position_32 = 2 * (qt.y * qt.z - qt.x * qt.w)
    dcm_position_33 = qt.w**2 - qt.x**2 - qt.y**2 + qt.z**2
    # Form the Final Direction Cosine Matrix
    dcm = np.array(
        [
            [dcm_position_11, dcm_position_12, dcm_position_13],
            [dcm_position_21, dcm_position_22, dcm_position_23],
            [dcm_position_31, dcm_position_32, dcm_position_33],
        ]
    )
    return dcm


def quaternion_to_euler(qt):
    """Convert a Quaternion to Euler angles. Input should be a Quaternion formed from
    np.quaternion(w,x,y,z) and outputs an np.array(phi,theta,psi) Euler Angle."""
    dcm = quaternion_to_dcm(qt)
    final_euler_angles = dcm_to_euler(dcm)
    return final_euler_angles


def euler_to_dcm(euler_angles):
    """Convert from Euler Angles to a Direction Cosine Matrix. Input should be
    Euler Angles as a numpy ndarray vector formed from np.array([phi, theta, psi]) in radians.
    """
    # Roll / Bank (X axis rotation North / South)
    phi, theta, psi = euler_angles
    cb2_x = np.array(
        [[1, 0, 0], [0, np.cos(phi), np.sin(phi)], [0, -np.sin(phi), np.cos(phi)]]
    )

    # Pitch (Y axis rotation East / West )
    c21_y = np.array(
        [
            [np.cos(theta), 0, -np.sin(theta)],
            [0, 1, 0],
            [np.sin(theta), 0, np.cos(theta)],
        ]
    )

    # Yaw (Z axis rotation up/down)
    c1v_z = np.array(
        [[np.cos(psi), np.sin(psi), 0], [-np.sin(psi), np.cos(psi), 0], [0, 0, 1]]
    )
    # Perform rotation in ZYX order convention, multiplying in REVERSE order for this effect
    rotation_matrix = cb2_x @ c21_y @ c1v_z
    return rotation_matrix


def euler_angles_to_quaternion(euler_angles):
    """Convert from Euler Angles to a Quaternion. Argument should be Euler Angles as a numpy ndarray vector formed
    from np.array([phi, theta, psi]) in radians."""

    # First convert Euler Angles to a Direction Cosine Matrix
    dcm = euler_to_dcm(euler_angles)
    # With this Direction Cosine Matrix we can now convert to a Quaternion using the dcm_to_quaternion function
    output_quaternion = dcm_to_quaternion(dcm)
    return output_quaternion


def get_quaternion_rotation_angle_rad(qt):
    """Extract the scalar component of a Quaternion to find its rotational angle in radians.
    Takes a Quaternion formed from np.quaternion(w,x,y,z) as argument."""
    if not -1 <= qt.w <= 1:
        raise Exception(
            "The scalar component of your quaternion q.w must be between -1 and 1 for the \"get_quaternion_rotation_angle_rad function to work. We're dealing with arccos here, people!"
        )
    rotation_angle_in_rads = 2 * np.arccos(qt.w)
    return rotation_angle_in_rads


def get_quaternion_rotation_angle_deg(qt):
    """Extract the scalar component of a Quaternion to find its rotational angle in degrees.
    Takes a Quaternion formed from np.quaternion(w,x,y,z) as argument."""
    if not -1 <= qt.w <= 1:
        raise Exception(
            "The scalar component of your quaternion q.w must be between -1 and 1 for the get_quaternion_rotation_angle_deg function to work. We're dealing with arccos here, people!"
        )
    rotation_angle_in_deg = 2 * np.arccos(qt.w) * 180 / np.pi
    return rotation_angle_in_deg


def get_quaternion_axis_of_rot(qt):
    """Find the axis of rotation as a unit vector.
    Takes a Quaternion formed from np.quaternion(w,x,y,z) as argument."""
    vector_part = quaternion.as_vector_part(qt)
    angle_part = np.sin(get_quaternion_rotation_angle_rad(qt) / 2)
    # prevent division by zero
    if np.sin(get_quaternion_rotation_angle_rad(qt)) == 0:
        axis_of_rotation = vector_part * 0
    else:
        axis_of_rotation = vector_part * (1 / angle_part)
    return axis_of_rotation


def axis_angle_to_quaternion(angle, axis):
    """Create a Quaternion with the inputs transformation angle in radians and a rotation axis as a unit vector"""
    transformation_angle = np.cos(angle / 2)
    vector_part = axis * np.sin(angle / 2)
    output_quaternion = np.quaternion(transformation_angle, *vector_part)
    return output_quaternion
