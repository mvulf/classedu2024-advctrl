from regelum.utils import rg
from regelum.system import CartPole


class CartPoleWithMotor(CartPole):
    _dim_observation = 5
    _dim_state = 5
    _parameters = {
        "m_c": 0.1,
        "m_p": 2.0,
        "g": 9.81,
        "l": 0.5,
        "friction_coeff": 100.0,
        "tau_motor": 0.02,
    }
    _action_bounds = [[-100.0, 100.0]]

    _observation_naming = _state_naming = [
        "angle [rad]",
        "x [m]",
        "angle_dot [rad/s]",
        "x_dot [m/s]",
        "force [N]",
    ]
    _inputs_naming = ["Motor Torque [N]"]

    def _compute_state_dynamics(self, time, state, inputs):
        Dstate = rg.zeros(
            self.dim_state,
            prototype=(state, inputs),
        )

        mass_cart, mass_pole, grav_const, length_pole, motor_moment = (
            self.parameters["m_c"],
            self.parameters["m_p"],
            self.parameters["g"],
            self.parameters["l"],
            self._parameters["tau_motor"],
        )

        angle = state[0]
        angle_vel = state[2]
        vel = state[3]
        force = state[4]
        sin_angle = rg.sin(angle)
        cos_angle = rg.cos(angle)
        control_variable = inputs[0]

        #########################
        ## YOUR CODE GOES HERE ##

        force_part = (
            force # state
            + mass_pole*length_pole*angle_vel**2*sin_angle
        )

        Dstate[0] = angle_vel
        Dstate[1] = vel
        Dstate[2] = (
            grav_const*sin_angle*(mass_cart + mass_pole)
            - cos_angle*force_part
        )/(
            4/3*length_pole*(mass_cart + mass_pole)
            - mass_pole*length_pole*cos_angle**2
        )
        Dstate[3] = (
            force_part - 3/8 * mass_pole * grav_const * rg.sin(2*angle)
        )/(
            mass_cart + mass_pole - 3/4 * mass_pole * cos_angle**2
        )
        Dstate[4] = 1/motor_moment * (control_variable - force)

        ## YOUR CODE ENDS HERE ##
        #########################

        return Dstate

    def _get_observation(self, time, state, inputs):
        return state
