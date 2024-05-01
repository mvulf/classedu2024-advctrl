from numpy.core.multiarray import array as array
from regelum.policy import Policy
import numpy as np
from src.system import CartPoleWithMotor
from typing import Union


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


class CartPolePD(Policy):
    def __init__(self):
        super().__init__()
        #########################
        ## YOUR CODE GOES HERE ##

        # P_angle = 3.2e3
        # D_angle = 1.8e2
        
        # P_position = 4.2e2
        # D_position = 2.8e2
        
        P_angle = 50
        D_angle = 7
        
        P_position = 5.0
        D_position = 3.0

        ## Find the coefficients of the PD controller
        ## It should be a list of 4 elements
        self.pd_coefs: list[float] = [
            P_angle,
            P_position,
            D_angle,
            D_position,
        ]

        ## YOUR CODE ENDS HERE ##
        #########################

    def get_action(self, observation: np.ndarray) -> np.ndarray:
        return np.array(
            [[cartpole_pd_action(observation=observation, pd_coefs=self.pd_coefs)]]
        )


class CartPoleBackstepping(Policy):
    def __init__(
        self,
        system: CartPoleWithMotor,
        sampling_time: float,
    ):
        super().__init__()
        self.system = system
        self.sampling_time = sampling_time

        #########################
        ## YOUR CODE GOES HERE ##

        ## Define hyperparameters
        self.backstepping_gain: float = ...
        self.energy_gain: float = ...
        self.velocity_gain: float = ...
        self.pd_coefs: list[float] = [
            ...,
            ...,
            ...,
            ...,
        ]  # for hard or soft switch, (should be a list of 4 elements)

        ## YOUR CODE ENDS HERE ##
        #########################

    def cartpole_energy_based_force_control_function(
        self,
        angle: float,
        angle_vel: float,
        vel: float,
    ) -> float:
        mass_pole = self.system._parameters["m_p"]
        mass_cart = self.system._parameters["m_c"]
        grav_const = self.system._parameters["g"]
        length_pole = self.system._parameters["l"]

        #########################
        ## YOUR CODE GOES HERE ##
        ...

        force = ...

        ## YOUR CODE ENDS HERE ##
        #########################

        return force

    def cartpole_backstepping(
        self,
        old_energy_based_force: float,
        force: float,
    ) -> float:
        #########################
        ## YOUR CODE GOES HERE ##
        backstepping_action = force - self.backstepping_gain * (
            force - old_energy_based_force
        )
        ## YOUR CODE ENDS HERE ##
        #########################

        return backstepping_action

    def get_action(self, observation: np.ndarray) -> np.ndarray:
        energy_based_force = self.cartpole_energy_based_force_control_function(
            angle=observation[0, 0],
            angle_vel=observation[0, 2],
            vel=observation[0, 3],
        )

        pd_action = cartpole_pd_action(observation=observation, pd_coefs=self.pd_coefs)

        backstepping_action = self.cartpole_backstepping(
            old_energy_based_force=energy_based_force,
            force=observation[0, 4],
        )

        #########################
        ## YOUR CODE GOES HERE ##

        # Implement hard or soft switch to PD regulator here
        action = ...

        ## YOUR CODE ENDS HERE ##
        #########################

        return np.array([[action]])
