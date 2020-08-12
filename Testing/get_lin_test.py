import rbdl
import numpy as np

def get_model():

    model = rbdl.Model()

    m1 = 1
    m2 = 1
    l1 = 1
    l2 = 1
    model.gravity = np.array([0, -9.81, 0])
    joint_rot_y = rbdl.Joint.fromJointType("JointTypeRevoluteZ")
    xtrans = rbdl.SpatialTransform()

    xtrans.r = np.array([0., -3., 0.])

    link0 = rbdl.Body.fromMassComInertia(0,
                                         np.array([0., 0, 0.]),
                                         np.diag([0.0, 0.0, 0.0])
                                         )

    link1 = rbdl.Body.fromMassComInertia(m1,
                                         np.array([0., -l1 * 0.5, 0.]),
                                         np.diag([m1 * l1 * l1 / 3., m1 * l1 * l1 / 30., m1 * l1 * l1 / 3.])
                                         )

    link2 = rbdl.Body.fromMassComInertia(m2,
                                         np.array([0., -l2 * 0.5, 0.]),
                                         np.diag([m2 * l2 * l2 / 3., m2 * l2 * l2 / 30., m1 * l2 * l2 / 3.])
                                         )

    a = model.AppendBody(rbdl.SpatialTransform(), joint_rot_y, link0)
    b = model.AppendBody(xtrans, joint_rot_y, link1)
    #c = model.AppendBody(xtrans, joint_rot_y, link2)

    return model


def finite_differences(model, x, u):
    """ calculate gradient of plant dynamics using finite differences
    x np.array: the state of the system
    u np.array: the control signal
    """
    dof = u.shape[0]
    num_states = model.q_size

    A = np.zeros((num_states, num_states))
    B = np.zeros((num_states, dof))

    eps = 1e-4  # finite differences epsilon
    for ii in range(num_states):
        # calculate partial differential w.r.t. x
        inc_x = x.copy()
        inc_x[ii] += eps
        state_inc, _ = runge_integrator(model, inc_x, u.copy())
        dec_x = x.copy()
        dec_x[ii] -= eps
        state_dec, _ = runge_integrator(model, dec_x, u.copy())
        A[:, ii] = (state_inc - state_dec) / (2 * eps)

    for ii in range(dof):
        # calculate partial differential w.r.t. u
        inc_u = u.copy()
        inc_u[ii] += eps
        state_inc, _ = runge_integrator(model, x.copy(), inc_u)
        dec_u = u.copy()
        dec_u[ii] -= eps
        state_dec, _ = runge_integrator(model, x.copy(), dec_u)
        B[:, ii] = (state_inc - state_dec) / (2 * eps)

    return A, B


def runge_integrator(model, t, y, h, tau):

    k1 = rhs(model, y, tau)
    k2 = rhs(model, y + 0.5 * h * k1, tau)
    k3 = rhs(model, y + 0.5 * h * k2, tau)
    k4 = rhs(model, y + h * k3, tau)

    return (k1 + 2. * k2 + 2. * k3 + k4)/6.0


def rhs(model, y, tau):

    dim = model.dof_count
    res = np.zeros(dim * 2)
    Q = np.zeros(model.q_size)
    QDot = np.zeros(model.qdot_size)
    QDDot = np.zeros(model.qdot_size)
    Tau = np.zeros(model.qdot_size)
    Tau[0] = tau
    for i in range(0, dim):
        Q[i] = y[i]
        QDot[i] = y[i + dim]

    rbdl.ForwardDynamics(model, Q, QDot, Tau, QDDot)
    for i in range(0, dim):
        res[i] = QDot[i]
        res[i + dim] = QDDot[i]

    return res