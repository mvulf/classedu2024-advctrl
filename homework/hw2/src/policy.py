from numpy.core.multiarray import array as array
from regelum.policy import Policy
import numpy as np
from src.system import CartPoleWithFriction
from typing import Union


def cartpole_pd_action(
    observation: np.ndarray, pd_coefs: Union[np.ndarray, list[float]]
) -> float:
    angle = observation[0, 0]
    x_position = observation[0, 1]
    angle_vel = observation[0, 2]
    vel = observation[0, 3]

    x_position_clipped = np.clip(x_position, -1.0, 1.0)
    vel_clipped = np.clip(vel, -1.0, 1.0)

    action = (
        np.sin(angle) * pd_coefs[0]
        + x_position_clipped * pd_coefs[1]
        + angle_vel * pd_coefs[2]
        + vel_clipped * pd_coefs[3]
    )

    return action


class CartPolePD(Policy):
    def __init__(
        self, system: CartPoleWithFriction, action_min: float, action_max: float
    ):
        super().__init__()
        self.system = system
        self.action_min = action_min
        self.action_max = action_max

        #########################
        ## YOUR CODE GOES HERE ##

        ## Find the coefficients of the PD controller
        ## It should be a list of 4 elements
        self.pd_coefs: list[float] = ...

        ## YOUR CODE ENDS HERE ##
        #########################

    def get_action(self, observation: np.ndarray) -> np.ndarray:
        return np.array(
            [
                [
                    np.clip(
                        cartpole_pd_action(
                            observation=observation, pd_coefs=self.pd_coefs
                        ),
                        self.action_min,
                        self.action_max,
                    )
                ]
            ]
        )


class CartPoleEnergyBasedFrictionCompensation(Policy):
    def __init__(self, system: CartPoleWithFriction):
        super().__init__()
        self.system = system

        #########################
        ## YOUR CODE GOES HERE ##

        ## Define hyperparameters

        self.energy_gain: float = ...
        self.velocity_gain: float = ...
        self.pd_coefs: list[float] = (
            ...
        )  # for hard or soft switch (should be a list of 4 elements)

        ## YOUR CODE ENDS HERE ##
        #########################

    def cartpole_energy_based_control_function_friction_compensation(
        self,
        angle: float,
        angle_vel: float,
        vel: float,
        friction_coeff: float,
    ) -> float:
        mass_pole = self.system["m_p"]
        mass_cart = self.system["m_c"]
        length_pole = self.system["l"]

        #########################
        ## YOUR CODE GOES HERE ##
        ...

        force = ...

        ## YOUR CODE ENDS HERE ##
        #########################
        return force

    def get_action(self, observation: np.ndarray) -> np.ndarray:
        energy_based_action = (
            self.cartpole_energy_based_control_function_friction_compensation(
                angle=observation[0, 0],
                angle_vel=observation[0, 2],
                vel=observation[0, 3],
                friction_coeff=self.system["friction_coeff"],
            )
        )
        pd_action = cartpole_pd_action(observation=observation, pd_coefs=self.pd_coefs)

        #########################
        ## YOUR CODE GOES HERE ##

        # Implement hard or soft switch to PD regulator here
        action = ...

        ## YOUR CODE ENDS HERE ##
        #########################

        return np.array([[action]])


class CartPoleEnergyBasedFrictionAdaptive(CartPoleEnergyBasedFrictionCompensation):
    def __init__(
        self,
        system: CartPoleWithFriction,
        sampling_time: float,
        friction_coeff_est_init: float = 0.0,
    ):
        super().__init__(system)
        self.sampling_time = sampling_time
        self.friction_coeff_est = friction_coeff_est_init

        #########################
        ## YOUR CODE GOES HERE ##

        ## Define hyperparameters

        self.energy_gain: float = ...
        self.velocity_gain: float = ...
        self.friction_coeff_est_learning_rate: float = ...
        self.pd_coefs: list[float] = (
            ...
        )  # for hard or soft switch, (should be a list of 4 elements)

        ## YOUR CODE ENDS HERE ##
        #########################

    def euler_update_friction_coeff_estimate(
        self,
        angle: float,
        angle_vel: float,
        vel: float,
    ) -> None:

        mass_pole = self.system["m_p"]
        mass_cart = self.system["m_c"]
        length_pole = self.system["l"]

        #########################
        ## YOUR CODE GOES HERE ##
        ...

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
