from numpy.core.multiarray import array as array
from regelum.policy import Policy
import numpy as np
from src.system import CartPoleWithFriction
from typing import Union, Any


def cartpole_pd_action(
    observation: np.ndarray, pd_coefs: Union[np.ndarray, list[float]]
) -> float:
    angle = observation[0, 0]
    x_position = observation[0, 1]
    angle_vel = observation[0, 2]
    vel = observation[0, 3]

    x_position_clipped = np.clip(x_position, -1.0, 1.0)

    action = (
        np.sin(angle) * pd_coefs[0]
        + x_position_clipped * pd_coefs[1]
        + angle_vel * pd_coefs[2]
        + vel * pd_coefs[3]
    )

    return action


def hard_switch(action1: Any, action2: Any, condition: bool):
    if condition:
        return action1
    else:
        return action2


class CartPolePD(Policy):
    def __init__(self):
        super().__init__()
        #########################
        ## YOUR CODE GOES HERE ##

        P_angle = 3.2e3
        D_angle = 1.8e2
        
        P_position = 4.2e2
        D_position = 2.8e2

        ## Find the coefficients of the PD controller
        ## It should be a list of 4 elements
        self.pd_coefs: list[float] = [
            P_angle,
            P_position,
            D_angle,
            D_position
        ]

        ## YOUR CODE ENDS HERE ##
        #########################

    def get_action(self, observation: np.ndarray) -> np.ndarray:
        return np.array(
            [[cartpole_pd_action(observation=observation, pd_coefs=self.pd_coefs)]]
        )


class CartPoleEnergyBasedFrictionAdaptive(Policy):
    def __init__(
        self,
        system: CartPoleWithFriction,
        sampling_time: float,
        friction_coeff_est_init: float = 0.0,
    ):
        super().__init__()
        self.system = system
        self.sampling_time = sampling_time
        self.friction_coeff_est = friction_coeff_est_init

        #########################
        ## YOUR CODE GOES HERE ##

        ## Define hyperparameters

        self.energy_gain: float = ...
        self.velocity_gain: float = ...
        self.friction_coeff_est_learning_rate: float = ...
        self.pd_coefs: list[float] = [
            ...,
            ...,
            ...,
            ...,
        ]  # for hard or soft switch, (should be a list of 4 elements)

        ## YOUR CODE ENDS HERE ##
        #########################

    def cartpole_energy_based_control_function_friction_compensation(
        self,
        angle: float,
        angle_vel: float,
        vel: float,
        friction_coeff: float,
    ) -> float:
        mass_pole = self.system._parameters["m_p"]
        mass_cart = self.system._parameters["m_c"]
        grav_const = self.system._parameters["g"]
        length_pole = self.system._parameters["l"]

        #########################
        ## YOUR CODE GOES HERE ##

        force = ...

        ## YOUR CODE ENDS HERE ##
        #########################
        return force

    def euler_update_friction_coeff_estimate(
        self,
        angle: float,
        angle_vel: float,
        vel: float,
    ) -> None:

        mass_pole = self.system._parameters["m_p"]
        mass_cart = self.system._parameters["m_c"]
        grav_const = self.system._parameters["g"]
        length_pole = self.system._parameters["l"]

        #########################
        ## YOUR CODE GOES HERE ##

        self.friction_coeff_est = ...

        ## YOUR CODE ENDS HERE ##
        #########################
        return

    def get_action(self, observation: np.ndarray) -> np.ndarray:
        energy_based_action = (
            self.cartpole_energy_based_control_function_friction_compensation(
                angle=observation[0, 0],
                angle_vel=observation[0, 2],
                vel=observation[0, 3],
                friction_coeff=self.friction_coeff_est,
            )
        )
        self.euler_update_friction_coeff_estimate(
            angle=observation[0, 0],
            angle_vel=observation[0, 2],
            vel=observation[0, 3],
        )

        pd_action = cartpole_pd_action(observation=observation, pd_coefs=self.pd_coefs)

        #########################
        ## YOUR CODE GOES HERE ##

        # Implement hard or soft switch to PD regulator here
        action = ...

        ## YOUR CODE ENDS HERE ##
        #########################

        return np.array([[action]])
