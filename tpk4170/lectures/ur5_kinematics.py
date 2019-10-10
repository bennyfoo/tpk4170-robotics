import numpy as np

import modern_robotics as mr
from tpk4170.utils.transformations import quaternion_from_euler


class Ur5Kinematics:
    def __init__(self):
        W1 = 0.10915
        W2 = 0.0823
        L1 = 0.425
        L2 = 0.39225
        H1 = 0.089159
        H2 = 0.09465
        self._M = np.array([[-1.0, 0.0, 0.0, L1 + L2],
                            [0.0, 0.0, 1.0, W1 + W2],
                            [0.0, 1.0, 0.0, H1 - H2],
                            [0.0, 0.0, 0.0, 1.0]])

        self._SList = np.array([[0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                                [0.0, 1.0, 0.0, -H1, 0.0, 0.0],
                                [0.0, 1.0, 0.0, -H1, 0.0, L1],
                                [0.0, 1.0, 0.0, -H1, 0.0, L1 + L2],
                                [0.0, 0.0, -1.0, -W1, L1 + L2, 0.0],
                                [0.0, 1.0, 0.0, H2 - H1, 0.0, L1 + L2]]).T

    def fkin(self, theta):
        return mr.FKinSpace(self._M, self._SList, theta)

    def ikin(self, T, theta0, eomg=1e-3, ev=1e-6, maxiterations=20):
        '''Computes inverse kinematics in the space frame for an open chain robot
        Copied from https://github.com/NxRLab/ModernRobotics/blob/master/packages/Python/modern_robotics/core.py
        '''

        thetalist = np.array(theta0).copy()
        iterates = [thetalist.copy()]
        i = 0
        Tsb = mr.FKinSpace(self._M, self._SList, thetalist)
        Vs = np.dot(mr.Adjoint(Tsb),
                    mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(Tsb), T))))
        err = np.linalg.norm([Vs[0], Vs[1], Vs[2]]) > eomg \
            or np.linalg.norm([Vs[3], Vs[4], Vs[5]]) > ev
        while err and i < maxiterations:
            thetalist = thetalist \
                + np.dot(np.linalg.pinv(mr.JacobianSpace(self._SList,
                                                         thetalist)), Vs)
            iterates.append(thetalist.copy())
            i = i + 1
            Tsb = mr.FKinSpace(self._M, self._SList, thetalist)
            Vs = np.dot(mr.Adjoint(Tsb),
                        mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(Tsb), T))))
            err = np.linalg.norm([Vs[0], Vs[1], Vs[2]]) > eomg \
                or np.linalg.norm([Vs[3], Vs[4], Vs[5]]) > ev
        return (thetalist, not err, i, iterates)

    def ikina(self, T, shoulder=-1, elbow=1, wrist=-1):

        a = [0.00000, -0.42500, -0.39225,  0.00000,  0.00000,  0.0000]
        d = [0.089159,  0.00000,  0.00000,  0.10915,  0.09465,  0.0823]

        Te = T
        ne, se, ae, pe = Te[:3, :].T

        d1, _, _, d4, d5, d6 = d
        _, a2, a3, _, _, _ = a

        L2 = a2
        L3 = a3

        p5 = pe - d6 * ae
        p5x, p5y, _ = p5

        psi5 = np.arctan2(shoulder * p5y, shoulder * p5x)
        offset1 = np.arctan2(d4, np.sqrt(p5x*p5x + p5y*p5y - d4*d4))

        q1 = psi5 + shoulder * offset1

        x1 = np.array([np.cos(q1), np.sin(q1), 0])
        y1 = np.array([0, 0, 1])
        z1 = np.array([np.sin(q1), -np.cos(q1), 0])

        z4 = wrist * shoulder * np.cross(z1, ae)
        z4 /= np.linalg.norm(z4)
        y4 = z1
        x4 = np.cross(y4, z4)

        q5 = np.arctan2(np.inner(-ae, x4), np.inner(ae, y4))

        q6 = np.arctan2(np.inner(-z4, ne), np.inner(-z4, se))

        p3x, p3y, p3z = p5 - d5 * z4 - d4 * z1

        pv = p3z - d1
        ph = shoulder * np.sqrt(p3x*p3x + p3y*p3y)

        q234 = np.arctan2(np.inner(z4, x1), np.inner(z4, -y1))

        c3 = (ph*ph + pv*pv - L2*L2 - L3*L3) / (2 * L2 * L3)
        s3 = elbow * np.sqrt(1.0 - c3*c3)
        q3 = np.arctan2(s3, c3)

        c2 = (ph * (L2 + L3*c3) + pv*L3*s3) / (L2*L2 + L3*L3 + 2*L2*L3*c3)
        s2 = (pv * (L2 + L3*c3) - ph*L3*s3) / (L2*L2 + L3*L3 + 2*L2*L3*c3)
        q2 = np.arctan2(s2, c2)

        q4 = q234 - (q2 + q3)
        if q4 > np.pi:
            q4 -= 2 * np.pi

        q1 += np.pi

        return np.array([q1, q2, q3, q4, q5, q6])
