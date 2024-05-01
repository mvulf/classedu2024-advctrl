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
        
        P_angle = 3.2e3
        D_angle = 1.8e2
        
        P_position = 4.2e2
        D_position = 2.8e2

        self.energy_gain: float = 4.
        self.velocity_gain: float = 1.
        self.friction_coeff_est_learning_rate: float = 2.
        self.pd_coefs: list[float] = [
            P_angle,
            P_position,
            D_angle,
            D_position
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

        cos_angle = np.cos(angle)
        sin_angle = np.sin(angle)
        sin_2angle = np.sin(2*angle)
        
        pole_inertia = 4/3 * mass_pole * length_pole**2

        effective_masses = mass_cart + mass_pole - 3/4*mass_pole*cos_angle**2
        
        pole_energy = (
            1/2 * pole_inertia * angle_vel**2
            + mass_pole*grav_const*length_pole*(cos_angle - 1)
        )

        total_energy = (
            pole_energy*angle_vel*cos_angle
            - self.velocity_gain*vel
        )

        force = (
            self.energy_gain*effective_masses*total_energy
            + 3/8*mass_pole*grav_const*sin_2angle
            - mass_pole*length_pole*angle_vel**2*sin_angle
            + friction_coeff*vel
        )

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
        
        cos_angle = np.cos(angle)
        sin_angle = np.sin(angle)
        sin_2angle = np.sin(2*angle)
        
        pole_inertia = 4/3 * mass_pole * length_pole**2

        effective_masses = mass_cart + mass_pole - 3/4*mass_pole*cos_angle**2
        
        pole_energy = (
            1/2 * pole_inertia * angle_vel**2
            + mass_pole*grav_const*length_pole*(cos_angle - 1)
        )

        total_energy = (
            pole_energy*angle_vel*cos_angle
            - self.velocity_gain*vel
        )

        friction_coeff_est_derivative = (
            mass_pole*length_pole*total_energy*vel/effective_masses
        )

        self.friction_coeff_est += (
            self.friction_coeff_est_learning_rate
            * friction_coeff_est_derivative
            * self.sampling_time
        )

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

        angle=observation[0, 0]
        
        # action = energy_based_action
        
        # Implement hard or soft switch to PD regulator here
        action = hard_switch(
            action1=pd_action,
            action2=energy_based_action,
            condition=(np.cos(angle) > np.cos(np.pi / 8))
        )

        ## YOUR CODE ENDS HERE ##
        #########################

        return np.array([[action]])
