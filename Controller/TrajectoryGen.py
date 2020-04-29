import numpy as np


class TrajectoryGen(object):
    def __init__(self):
        # trajectory constants
        self.traj_coeffs = []
        self.tf = 0
        pass

    def create_traj(self, q0, qf, v0, vf, tf):
        """
        q0: initial position
        qf: final position
        v0: initial velocity
        vf: final velocity
        tf: final time
        """

        b = np.array([q0, v0, qf, vf]).reshape((-1, 1))
        A = np.array([[1.0, 0.0, 0.0, 0.0],
                      [0.0, 1.0, 0.0, 0.0],
                      [1.0, tf, tf ** 2, tf ** 3],
                      [0.0, 1.0, 2 * tf, 3 * tf ** 2]])

        self.traj_coeffs = np.linalg.solve(A, b)
        self.tf = tf

    def get_traj(self, t):
        """
        t: current time
        return: q, qd, qdd based on the trajectory
                false if t is out of bounds
        """

        if t > self.tf:
            print("ERROR: t out of bounds")

        a = self.traj_coeffs

        q = a[0] + a[1] * t + a[2] * t**2 + a[3] * t**3
        qd = a[1] + 2 * a[2] * t + 3 * a[3] * t**2
        qdd = 2 * a[2] + 6 * a[3] * t

        return q, qd, qdd
