import rclpy
from rclpy.node import Node
from dataclasses import dataclass
import numpy as np
import copy
import threading
import cvxpy
import time
from cvxpy.expressions import constants
from interfaces.msg import Reference, State
from deepracer_interfaces_pkg.msg import ServoCtrlMsg

"""
MODELS
"""


@dataclass
class CarParams:
    # m: int = 5  # Mass [kg]
    # Iz: int = 50  # Yaw moment of inertia [m*N*s^2]
    lf: int = 0.085  # Longitudinal distance from c.g. to front tires [m]
    lr: int = 0.085  # Longitudinal distance from c.g. to reartires [m]
    # Cf: int = 19000  # Front tire cornering stiffness [N/rad]
    # Cr: int = 33000  # Rear tire cornering stiffness [N/rad]
    accel_max: float = 1  # [m/s^s]
    accel_min: float = -1  # [m/s^s]
    steer_max: float = 0.52  # [rad]
    steer_min: float = -0.52  # [rad]
    vel_max: float = 6.5  # [m/s]
    vel_min: float = 0  # [m/s]


class Car:
    def __init__(self, params: CarParams):
        self.x = 0
        self.y = 0
        self.xdot = 0
        self.ydot = 0
        self.psi = 0
        self.psidot = 0
        self.params = params

    def set_state(self, state):
        """
        :param state: [x, y, xdot, ydot, psi, psidot]
        :return:
        """
        self.x = state[0]
        self.y = state[1]
        self.xdot = state[2]
        self.ydot = state[3]
        self.psi = state[4]
        self.psidot = state[5]

    def get_v(self):
        return np.linalg.norm((self.xdot, self.ydot), ord=2)

    def get_state(self):
        return np.array([self.x, self.y, self.xdot, self.ydot, self.psi, self.psidot])


class Model:
    def __init__(self, car, dt):
        self.car = car
        self.dt = dt

    def step(self, control):
        raise NotImplementedError


class KBModel(Model):
    def __init__(self, car, dt):
        super().__init__(car, dt)

    def get_state(self):
        return np.array([self.car.x, self.car.y, self.car.get_v(), self.car.psi])

    def step(self, control):
        """
        Kinematic bicycle model from A. Carvalho et. al. (2015), section 3.1.1.
        :control: [a, delta]
        :return: np.array [X, Y, v, psi]
        """
        # Assign convenience names for legibility.
        accel = control[0]
        steer = control[1]
        p = self.car.params
        c = self.car
        v_old = self.car.get_v()
        state = self.car.get_state()

        new_state = np.zeros(6)

        steer = np.clip(steer, self.car.params.steer_min, self.car.params.steer_max)
        v_old = np.clip(v_old, self.car.params.vel_min, self.car.params.vel_max)

        beta = np.arctan(np.tan(steer) * p.lr / (p.lf + p.lr))
        v = v_old + self.dt * accel

        new_state[0] = state[0] + self.dt * v * np.cos(beta + c.psi)
        new_state[1] = state[1] + self.dt * v * np.sin(beta + c.psi)
        new_state[2] = state[2] + self.dt * accel * np.cos(beta)
        new_state[3] = state[3] + self.dt * accel * np.sin(beta)
        new_state[4] = state[4] + self.dt * v * np.sin(beta) / p.lr
        new_state[5] = (new_state[4] - c.psi) / self.dt

        self.car.set_state(new_state)

        # X, Y, vel, yaw.
        return np.array([new_state[0], new_state[1], self.car.get_v(), new_state[4]])

    def lin_step(self, vel, yaw, steer):
        # state matrix
        matrix_a = np.zeros((4, 4))
        matrix_a[0, 0] = 1.0
        matrix_a[1, 1] = 1.0
        matrix_a[2, 2] = 1.0
        matrix_a[3, 3] = 1.0
        matrix_a[0, 2] = self.dt * np.cos(yaw)
        matrix_a[0, 3] = -self.dt * vel * np.sin(yaw)
        matrix_a[1, 2] = self.dt * np.sin(yaw)
        matrix_a[1, 3] = self.dt * vel * np.cos(yaw)
        matrix_a[3, 2] = \
            self.dt * np.tan(steer) / (self.car.params.lf + self.car.params.lr)

        # input matrix
        matrix_b = np.zeros((4, 2))
        matrix_b[2, 0] = self.dt
        matrix_b[3, 1] = self.dt * vel / \
                         ((self.car.params.lf + self.car.params.lr) * np.cos(steer) ** 2)

        # constant matrix
        matrix_c = np.zeros(4)
        matrix_c[0] = self.dt * vel * np.sin(yaw) * yaw
        matrix_c[1] = -self.dt * vel * np.cos(yaw) * yaw
        matrix_c[3] = - self.dt * vel * steer / \
                      ((self.car.params.lf + self.car.params.lr) * np.cos(steer) ** 2)

        return matrix_a, matrix_b, matrix_c


"""
CONTROLLER
"""


"""
Adapted from Pylot source by Fangyu Wu, Edward Fang.
https://github.com/erdos-project/pylot/tree/a71ae927328388dc44acc784662bf32a99f273f0/pylot/control/mpc

Reference: "Model Predictive Control for Autonomous and Semiautonomous Vehicles"
by Gao (page 33)
"""


@dataclass
class ControllerConfig:
    Q: np.array = np.diag([1.0, 1.0, 0.01, 0.01])  # Weight on reference deviations.
    R: np.array = np.diag([0.01, 0.10])            # Weight on control input.
    S: np.array = np.diag([1.0, 1.0])             # Weight on change in control input.
    prediction_horizon: int = 20
    tracking_horizon: int = 5                     # tracking_horizon < prediction_horizon


@dataclass
class ReferenceTraj:
    states: np.array  # [X, Y, vel, psi] x prediction_horizon
    curvature: np.array  # [k] x prediction_horizon

    def __copy__(self):
        return ReferenceTraj(copy.copy(self.states), copy.copy(self.curvature))


class MPC:
    def __init__(self,
                 reference: ReferenceTraj,
                 model: KBModel,
                 config: ControllerConfig):
        self.reference = reference
        self.model = model
        self.config = config

        if self.reference.states.shape[1] != 4:
            raise AttributeError("Reference does not have 4 columns; has shape ", self.reference.states.shape)

        self.predicted_steer = np.zeros((self.config.tracking_horizon, 1))
        self.predicted_accel = np.zeros((self.config.tracking_horizon, 1))

        self.num_state = 4
        self.predicted_state = np.zeros((self.num_state, self.config.tracking_horizon + 1))

        self.set_imminent_ref()
        
        if self.reference.states.shape[1] != 4:
            raise AttributeError("Reference does not have 4 columns; has shape ", self.reference.states.shape)

    def set_imminent_ref(self):
        """
        If the reference trajectory is longer than the prediction horizon,
        shorten it.
        """
        # Compute the distance of the waypoints to the car.
        rel_x = [X - self.model.car.x for X in self.reference.states[:, 0]]
        rel_y = [Y - self.model.car.y for Y in self.reference.states[:, 1]]
        rel_coords = zip(rel_x, rel_y)
        dist = [np.linalg.norm(pair) for pair in rel_coords]

        start_idx = np.argmin(dist)  # Get the index of the closest waypoint.
        end_idx = min(len(self.reference.states)-2, start_idx+self.config.tracking_horizon+1)

        # Truncate.
        self.reference.states = self.reference.states[start_idx:end_idx, :]
        self.reference.curvature = self.reference.curvature[start_idx:end_idx]

        if self.reference.states.shape[1] != 4:
            raise AttributeError("Reference does not have 4 columns; has shape ", self.reference.states.shape)

    def get_predicted_states(self):
        """
        Get predicted states for current steering and acceleration.
        :return: nparray
        """
        predicted_state = np.zeros((self.num_state, self.config.tracking_horizon + 1))
        predicted_state[:, 0] = self.model.get_state()
        self.predicted_steer = self.reference.curvature

        # Propagate forward in time.
        for accel, steer, t in zip(self.predicted_accel,
                                   self.predicted_steer,
                                   range(1, self.config.tracking_horizon + 1)):
            state = self.model.step([accel, steer])
            predicted_state[:, t] = state
        return predicted_state

    def control(self):
        """
        Formulate and solve the control problem.
        :return: accel, steering
        """
        self.predicted_state = self.get_predicted_states()

        x = cvxpy.Variable((self.config.tracking_horizon+1, self.num_state))  # [X, Y, v, psi]
        u = cvxpy.Variable((self.config.tracking_horizon, 2))
        cost = constants.Constant(0.0)
        constraints = []

        for t in range(self.config.tracking_horizon):
            cost += cvxpy.quad_form(u[t, :], self.config.R)  # Penalize control inputs.

            if t != 0:
                # Penalize deviation from reference trajectory.
                cost += cvxpy.quad_form(self.reference.states[t, :] - x[t, :],
                                        self.config.Q)

            # Define the model.
            A, B, C = self.model.lin_step(self.predicted_state[2, t], self.predicted_state[3, t],
                                          self.predicted_steer[t])
            constraints += [x[t+1, :] == A @ x[t, :] + B @ u[t, :] + C]

            # Penalize changes in control.
            if t < self.config.tracking_horizon-1:
                cost += cvxpy.quad_form(u[t+1, :] - u[t, :],
                                        self.config.S)

        # Natural contraints arising from car design.
        constraints += [x[0, :] == self.model.get_state()]
        constraints += [x[:, 2] <= self.model.car.params.vel_max]
        constraints += [x[:, 2] >= self.model.car.params.vel_min]
        constraints += [u[:, 0] <= self.model.car.params.accel_max]
        constraints += [u[:, 0] >= self.model.car.params.accel_min]
        constraints += [u[:, 1] <= self.model.car.params.steer_max]
        constraints += [u[:, 1] >= self.model.car.params.steer_min]

        # Solve the problem.
        prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
        res = prob.solve(solver=cvxpy.OSQP, verbose=False, warm_start=True)

        return u.value[0, 0], u.value[0, 1]  # Acceleration, steering.


"""
ROS2
"""

state = list()
last_vel = 0
last_time = None
duration_between_cmds = None


def get_controls(ref):
    model_car = Car(CarParams())
    model_car.set_state(state)

    controller = MPC(ref, KBModel(model_car, 0.1), ControllerConfig())
    c = controller.control()

    return c[0], c[1]

def scale_controls(v, s):
    V_MAX = 6.5
    V_MIN = 0
    S_MAX = 0.52
    S_MIN = -0.52

    v = np.clip(v, V_MIN, V_MAX)
    s = np.clip(s, S_MIN, S_MAX)

    scaled_v = (v - V_MIN) / (V_MAX - V_MIN)
    scaled_s = (s - S_MIN) / (S_MAX - S_MIN)

    return scaled_v, scaled_s


class ReferenceSubscriber(Node):

    def __init__(self, pub):
        super().__init__('control_sub')
        self.subscription = self.create_subscription(
            Reference,                                             
            'reference',
            self.listener_callback,
            10)
        self.subscription
        self.pub = pub

    def listener_callback(self, msg):
        global last_vel, last_time, duration_between_cmds

        X = list(msg.x)
        Y = list(msg.y)
        vel = list(msg.vel)
        psi = list(msg.yaw)
        k = list(msg.curvatures)

        ref_traj = ReferenceTraj(np.transpose(np.array([X, Y, vel, psi])), np.array(k))

        print("Received reference: ")
        print("ref.states: ", ref_traj.states, " is dim ", ref_traj.states.shape)
        print("ref.curvatures: ", ref_traj.curvature, " is dim ", ref_traj.curvature.shape)

        print("Initial state: ", state)

        time_now = time.time()
        if last_time is not None:
            duration_between_cmds = time_now - last_time
        else:
            duration_between_cmds = 0.1
        last_time = time_now

        control_msg = ServoCtrlMsg()
        v = 0
        s = 0
        try:
            a, s = get_controls(ref_traj)
            v = last_vel + a * duration_between_cmds
            v, s = scale_controls(v, s)
        except:
            pass

        control_msg.throttle = v
        control_msg.angle = s

        self.pub.publish(control_msg)
        print(f"Published controls ({a}, {s})") 


class StateSubscriber(Node):

    def __init__(self):
        super().__init__('state_sub')
        self.subscription = self.create_subscription(
            State,                                             
            'state',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        global state
        state = [msg.x, msg.y, msg.xdot, msg.ydot, msg.psi, msg.psidot]
            

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('control_pub')
    pub = node.create_publisher(ServoCtrlMsg, '/ctrl_pkg/servo_msg', 1)

    state_sub = StateSubscriber()
    ref_sub = ReferenceSubscriber(pub)

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(state_sub)
    executor.add_node(ref_sub)

    exec_thread = threading.Thread(target=executor.spin, daemon=True)
    exec_thread.start()
    rate = node.create_rate(2)

    print('ready')
    try:    
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    exec_thread.join()


if __name__ == '__main__':
    main()
