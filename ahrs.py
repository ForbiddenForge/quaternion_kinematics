import time

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FFMpegWriter, FuncAnimation, PillowWriter
from mpl_toolkits.mplot3d import Axes3D
from pynput import keyboard

# edit environment variable to tell matplotlib where ffmepgwriter lives, if you want to save as mp4
plt.rcParams[
    "animation.ffmpeg_path"
] = "C:\\ProgramData\\chocolatey\\lib\\ffmpeg\\tools\\ffmpeg\\bin\\ffmpeg.exe"


class AHRS:
    """Creates a simple Attitude Heading Reference System to calculate 3D movement of a rigid
    body according to Tait-Bryan angles, using right hand rule, counter-clockwise rotations.
    Range for Psi (Yaw) and Phi (Roll) are 2PI radians; Theta (pitch) has a range of PI radians
    """

    def __init__(self):
        self.roll_pitch_yaw_rate_input = np.array([0, 0, 0])
        self.euler_angles = np.array([0, 0, 0])

    @property
    def roll(self):
        return self.euler_angles[0]

    @property
    def pitch(self):
        return self.euler_angles[1]

    @property
    def yaw(self):
        return self.euler_angles[2]

    def create_h_matrix(self, euler_angles):
        """Create the H matrix using current Euler angle values.
        Note that this is NOT a rotation matrix but just a function
        of the given input p,q,r rate of change to give us the corresponding
        Euler angle rate of change."""
        phi, theta, psi = euler_angles
        print(f"create h matrix euler angles {euler_angles}")
        h_matrix = np.array(
            [
                [1, np.sin(phi) * np.tan(theta), np.cos(phi) * np.tan(theta)],
                [0, np.cos(theta), -np.sin(phi)],
                [0, np.sin(phi) / np.cos(theta), np.cos(phi) / np.cos(theta)],
            ]
        )
        print(f"H MATRIX: {h_matrix}")
        return h_matrix

    def calc_phi_theta_psi_dot(self, euler_angles):
        """Matrix multiply H with the roll(p), pitch(q), yaw(r) rate input"""
        return self.create_h_matrix(euler_angles) @ self.roll_pitch_yaw_rate_input

    def integrate_phi_theta_psi_dot(self, dt):
        """Integrate the Euler Angles' rates of change over the dt specified and add to current Euler Angles"""
        euler_rate_of_change = self.calc_phi_theta_psi_dot(self.euler_angles)
        print(euler_rate_of_change)
        phi_dot = euler_rate_of_change[0]
        theta_dot = euler_rate_of_change[1]
        psi_dot = euler_rate_of_change[2]

        self.euler_angles = self.euler_angles + np.array(
            [phi_dot * dt, theta_dot * dt, psi_dot * dt]
        )
        euler_list.append(self.euler_angles)

    def input(self):
        """Receive inputs of instantaneous rate of change of roll(p), pitch(q) and yaw(r)
        from either user input or rate gyro measurements"""
        pass

    def assert_euler_constraints(self):
        """Range for Psi (Yaw) and Phi (Roll) are 2PI radians; Theta (pitch) has a range of PI radians"""
        # roll
        if self.euler_angles[0] > 2 * np.pi:
            self.euler_angles[0] = 0
        if self.euler_angles[0] < -2 * np.pi:
            self.euler_angles[0] = 0
        # pitch; don't allow the plane to exceed straight vertical or straight down orientations
        if self.euler_angles[1] > np.pi / 2:
            self.euler_angles[1] = np.pi / 2
        if self.euler_angles[1] < -np.pi / 2:
            self.euler_angles[1] = -np.pi / 2
        # yaw
        if self.euler_angles[2] > 2 * np.pi:
            self.euler_angles[2] = 0
        if self.euler_angles[2] < -2 * np.pi:
            self.euler_angles[2] = 0

    def update(self, dt):
        self.integrate_phi_theta_psi_dot(dt)
        self.assert_euler_constraints()


def on_press(key):
    """Use keyboard inputs to update p,q,r as if it were receiving measurements from a rate gyro"""
    if key == keyboard.Key.esc:
        return False  # stop listener on ESCAPE key press
    elif key == keyboard.Key.right:
        ahrs.roll_pitch_yaw_rate_input = np.array([1, 0, 0])  # rads/second
    elif key == keyboard.Key.left:
        ahrs.roll_pitch_yaw_rate_input = np.array([-1, 0, 0])
    elif key == keyboard.Key.up:
        ahrs.roll_pitch_yaw_rate_input = np.array([0, 1, 0])
    elif key == keyboard.Key.down:
        ahrs.roll_pitch_yaw_rate_input = np.array([0, -1, 0])
    elif key == keyboard.Key.alt:
        ahrs.roll_pitch_yaw_rate_input = np.array([0, 0, 1])
    elif key == keyboard.Key.ctrl:
        ahrs.roll_pitch_yaw_rate_input = np.array([0, 0, -1])
    else:
        ahrs.roll_pitch_yaw_rate_input = np.array([0, 0, 0])


def update_rotation(euler_list):
    ax.cla()
    # Define origin, don't change it lol
    start = [0, 0, 0]
    print(f"Current Euler Angle: {euler_list} of total length {len(euler_list)}")
    v_arrow = ax.quiver(*start, *euler_list, color="blue", linewidths=10)
    ax.text(*euler_list, "V", color="blue")
    ax.set_xlim3d([-10, 10])
    ax.set_xlabel("X")
    ax.set_ylim3d([-10, 10])
    ax.set_ylabel("Y")
    ax.set_zlim3d([-10, 10])
    ax.set_zlabel("Z")


# Create AHRS object instance
ahrs = AHRS()
# Create a keyboard listener for duration of time loop below to simulate rate gyro inputs
listener = keyboard.Listener(on_press=on_press)
listener.start()


euler_list = []

dt = 0.1  # Same as funcAnimation interval
t = 0
# listener.join()
while t < 1:
    t += dt
    ahrs.update(dt)
    time.sleep(0.1)


listener.stop()

print(euler_list)
# Create Figure and animate the Euler Angle vector changes during the time loop
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")
ax.view_init(elev=20, azim=-45)
ani = FuncAnimation(
    fig=fig,
    func=update_rotation,
    frames=euler_list,
    interval=10,
    blit=False,
)
ani.save(
    "AHRS_euler_vector_changes.gif",
    writer="Pillow",
)
plt.show()
