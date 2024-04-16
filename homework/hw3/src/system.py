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

        Dstate[0] = ...
        Dstate[1] = ...
        Dstate[2] = ...
        Dstate[3] = ...
        Dstate[4] = ...

        ## YOUR CODE ENDS HERE ##
        #########################

        return Dstate

    def _get_observation(self, time, state, inputs):
        return state
