# Quaternion Kinematics

This repository extends the Quaternion module for use with kinematical equations, especially converting between Euler Angles, Direction Cosine Matrices, and Quaternions. It is particularly useful in aerospace applications (rigid body) and 3D visualization. The rotations are right-hand ruled, and the conversions between reference frames assume transitioning from the NED Frame to the Body Frame of a vehicle.

## Functions

### `quaternion_norm(qt)`

Find the magnitude/length/norm of a quaternion. Takes a Quaternion as an argument.

### `quaternion_inverse(qt)`

Find the inverse of a quaternion. Takes a Quaternion as an argument.

### `dcm_to_quaternion(dcm_matrix)`

Convert from a Direction Cosine Matrix to its corresponding Quaternion. The DCM must be representative of Cb/v, or a representation of the rotation from the NED frame to the vehicle body frame.

### `dcm_to_euler(dcm_matrix)`

Extract Euler Angles from a Direction Cosine Matrix. The output is a numpy array of the form `np.array([phi, theta, psi])`. Please note that the current numpy and quaternion functions output these angles in reverse order.

### `quaternion_to_dcm(qt)`

Convert from a Quaternion to a Direction Cosine Matrix. Takes a Quaternion object as an argument. Please ensure that the argument is formed using `np.quaternion(w, x, y, z)`.

### `quaternion_to_euler(qt)`

Convert a Quaternion to Euler angles. The input should be a Quaternion formed using `np.quaternion(w, x, y, z)`, and the output is an np.array `[phi, theta, psi]` representing the Euler angles.

### `euler_to_dcm(euler_angles)`

Convert from Euler Angles to a Direction Cosine Matrix. The input should be Euler Angles as a numpy ndarray vector formed using `np.array([phi, theta, psi])` in radians.

### `euler_angles_to_quaternion(euler_angles)`

Convert from Euler Angles to a Quaternion. The argument should be Euler Angles as a numpy ndarray vector formed using `np.array([phi, theta, psi])` in radians.

### `get_quaternion_rotation_angle_rad(qt)`

Extract the scalar component of a Quaternion to find its rotational angle in radians. Takes a Quaternion formed using `np.quaternion(w, x, y, z)` as an argument.

### `get_quaternion_rotation_angle_deg(qt)`

Extract the scalar component of a Quaternion to find its rotational angle in degrees. Takes a Quaternion formed using `np.quaternion(w, x, y, z)` as an argument.

### `get_quaternion_axis_of_rot(qt)`

Find the axis of rotation as a unit vector. Takes a Quaternion formed using `np.quaternion(w, x, y, z)` as an argument.

### `axis_angle_to_quaternion(angle, axis)`

Create a Quaternion with the input transformation angle in radians and a rotation axis as a unit vector.

## License

This code is released under the [MIT License](https://github.com/ForbiddenForge/quaternion_kinematics/blob/main/LICENSE).

Feel free to modify and use it according to your needs.

For more information and usage examples, please refer to the [quaternion_kinematics.py](https://github.com/ForbiddenForge/quaternion_kinematics/blob/main/quaternion_kinematics.py) file in this repository.
