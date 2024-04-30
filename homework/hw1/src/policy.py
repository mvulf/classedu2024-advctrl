from numpy.core.multiarray import array as array
from regelum.policy import Policy
import numpy as np
from scipy.special import expit
from src.system import InvertedPendulum, InvertedPendulumWithFriction, MyCartPole


def soft_switch(signal1, signal2, gate, loc=np.cos(np.pi / 4), scale=10):

    # Soft switch coefficient
    switch_coeff = expit((gate - loc) * scale)

    return (1 - switch_coeff) * signal1 + switch_coeff * signal2


def pd_based_on_sin(observation, pd_coefs=[20, 10]):
    return -pd_coefs[0] * np.sin(observation[0, 0]) - pd_coefs[1] * observation[0, 1]


class InvPendulumPolicyPD(Policy):
    def __init__(self, pd_coefs: np.ndarray, action_min: float, action_max: float):
        super().__init__()

        self.pid_coefs = np.array(pd_coefs).reshape(1, -1)
        self.action_min = action_min
        self.action_max = action_max

    def get_action(self, observation: np.ndarray):
        action = np.clip(
            (self.pid_coefs * observation).sum(),
            self.action_min,
            self.action_max,
        )
        return np.array([[action]])


class InvertedPendulumEnergyBased(Policy):
    def __init__(self, gain: float, action_min: float, action_max: float):
        super().__init__()
        self.gain = gain
        self.action_min = action_min
        self.action_max = action_max

    def get_action(self, observation: np.ndarray) -> np.ndarray:

        params = InvertedPendulum._parameters
        m, g, length = params["m"], params["g"], params["l"]

        theta = observation[0, 0]
        theta_vel = observation[0, 1]

        energy_total = (
            m * g * length * (np.cos(theta) - 1) + 0.5 * m * length**2 * theta_vel**2
        )
        energy_control_action = -self.gain * np.sign(theta_vel * energy_total)

        return np.array(
            [
                [
                    np.clip(
                        soft_switch(
                            signal1=energy_control_action,
                            signal2=pd_based_on_sin(observation),
                            gate=np.cos(theta),
                        ),
                        self.action_min,
                        self.action_max,
                    )
                ]
            ]
        )


class InvPendulumEnergyBasedFrictionCompensation(Policy):

    def __init__(self, gain: float, action_min: float, action_max: float):
        super().__init__()
        self.gain = gain
        self.action_min = action_min
        self.action_max = action_max

    def get_action(self, observation: np.ndarray) -> np.ndarray:

        params = InvertedPendulumWithFriction._parameters
        m, g, length, friction_coef = params["m"], params["g"], params["l"], params["c"]

        theta = observation[0, 0]
        theta_vel = observation[0, 1]

        energy_total = (
            m * g * length * (np.cos(theta) - 1) + 0.5 * m * length**2 * theta_vel**2
        )
        energy_control_action = -self.gain * np.sign(
            theta_vel * energy_total
        ) + friction_coef * m * length * theta_vel * np.abs(theta_vel)

        return np.array(
            [
                [
                    np.clip(
                        soft_switch(
                            signal1=energy_control_action,
                            signal2=pd_based_on_sin(observation),
                            gate=np.cos(theta),
                        ),
                        self.action_min,
                        self.action_max,
                    )
                ]
            ]
        )


class InvPendulumEnergyBasedFrictionAdaptive(Policy):

    def __init__(
        self,
        gain: float,
        action_min: float,
        action_max: float,
        sampling_time: float,
        gain_adaptive: float,
        friction_coef_est_init: float = 0,
    ):
        super().__init__()
        self.gain = gain
        self.action_min = action_min
        self.action_max = action_max
        self.friction_coef_est = friction_coef_est_init
        self.sampling_time = sampling_time
        self.gain_adaptive = gain_adaptive

    def get_action(self, observation: np.ndarray) -> np.ndarray:

        params = InvertedPendulumWithFriction._parameters
        m, g, length = params["m"], params["g"], params["l"]

        theta = observation[0, 0]
        theta_vel = observation[0, 1]

        energy_total = (
            m * g * length * (np.cos(theta) - 1) + 0.5 * m * length**2 * theta_vel**2
        )
        energy_control_action = -self.gain * np.sign(
            theta_vel * energy_total
        ) + self.friction_coef_est * m * length * theta_vel * np.abs(theta_vel)

        # Parameter adaptation using Euler scheme
        self.friction_coef_est += (
            -self.gain_adaptive
            * energy_total
            * m
            * length**2
            * np.abs(theta_vel) ** 3
            * self.sampling_time
        )

        return np.array(
            [
                [
                    np.clip(
                        soft_switch(
                            signal1=energy_control_action,
                            signal2=pd_based_on_sin(observation),
                            gate=np.cos(theta),
                        ),
                        self.action_min,
                        self.action_max,
                    )
                ]
            ]
        )


class CartPolePD(Policy):

    def __init__(self, action_min: float, action_max: float):
        super().__init__()
        self.action_min = action_min
        self.action_max = action_max

        #########################
        ## YOUR CODE GOES HERE ##

        ## Find the coefficients of the PD controller
        ## It should be a list of 4 elements
        # P_x = 1e-3
        # D_x = 1e-4
        
        # P_theta = 1.0
        # D_theta = 1e-3
        
        P_x = 1.1
        D_x = 1
        
        P_theta = 30
        D_theta = 2
        
        self.pd_coefs = np.array(
            [P_theta, P_x, D_theta, D_x]
        )

        ## YOUR CODE ENDS HERE ##
        #########################

    def get_action(self, observation: np.ndarray) -> np.ndarray:
        theta = observation[0, 0]
        x = observation[0, 1]
        omega = observation[0, 2]
        x_dot = observation[0, 3]
        theta = np.arctan2(np.sin(theta), np.cos(theta))

        x_clipped = np.clip(x, -1.0, 1.0)
        x_dot_clipped = np.clip(x_dot, -1.0, 1.0)

        action = (
            np.sin(theta) * self.pd_coefs[0]
            + x_clipped * self.pd_coefs[1]
            + omega * self.pd_coefs[2]
            + x_dot_clipped * self.pd_coefs[3]
        )

        return np.array(
            [
                [
                    np.clip(
                        action,
                        self.action_min,
                        self.action_max,
                    )
                ]
            ]
        )


class CartPoleEnergyBased(Policy):

    def __init__(self, action_min: float, action_max: float):
        super().__init__()
        self.action_max = action_max
        self.action_min = action_min

        #########################
        ## YOUR CODE GOES HERE ##

        ## Define hyperparameters if needed

        # self.hyperparameter1 = ...
        # self.hyperparameter2 = ...
        self.lambda_ = 4.0
        self.k_ = 1.
        
        P_x = 4.0
        D_x = 5.0
        
        P_theta = 90
        D_theta = 5
        
        self.pd_coefs = np.array(
            [P_theta, P_x, D_theta, D_x]
        )
        
        # self.PD_regulator = CartPolePD(action_min, action_max)

        ## YOUR CODE ENDS HERE ##
        #########################

    def get_action(self, observation: np.ndarray) -> np.ndarray:
        params = MyCartPole._parameters
        m_c, m_p, g, l = (
            params["m_c"],
            params["m_p"],
            params["g"],
            params["l"],
        )
        theta = observation[0, 0]
        omega = observation[0, 2]
        vel = observation[0, 3]
        sin_theta = np.sin(theta)
        cos_theta = np.cos(theta)

        ########################
        # YOUR CODE GOES HERE #
        x = observation[0, 1]

        I_p = 4/3 * m_p * l**2

        energy = 1/2 * I_p * omega**2 + m_p*g*l*(cos_theta - 1)

        vel_x_dot = self.k_*(energy*omega*cos_theta - self.lambda_*vel)

        if (theta == np.pi) and (omega == 0):
            # Make force if theta equals to pi and now angular velocity
            action = 100
        else:
            action = (
                vel_x_dot*(m_c + m_p*(1 - 3/4 * cos_theta**2))
                - m_p*l*omega**2*sin_theta
                + 3/4 * m_p * g * sin_theta * cos_theta
            )
        
        # pd_action = self.PD_regulator.get_action(observation)
        # omega = observation[0, 2]
        # theta = observation[0, 0]
        # theta = np.arctan2(np.sin(theta), np.cos(theta))

        x_clipped = np.clip(x, -1.0, 1.0)
        vel_clipped = np.clip(vel, -1.0, 1.0)

        pd_action = (
            np.sin(theta) * self.pd_coefs[0]
            + x_clipped * self.pd_coefs[1]
            + omega * self.pd_coefs[2]
            + vel_clipped * self.pd_coefs[3]
        )
        
        action = soft_switch(
            signal1=action,
            signal2=pd_action,
            gate=np.cos(theta),
            loc=np.cos(np.pi / 4),
        )

        # YOUR CODE ENDS HERE #
        #######################

        return np.array(
            [
                [
                    np.clip(
                        action,
                        self.action_min,
                        self.action_max,
                    )
                ]
            ]
        )
