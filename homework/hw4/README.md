# Homework 4. Model-predictive control.

The homework is designed for the "Advanced Control Methods-2024" course. It includes a set of practical tasks for students to implement.

## Homework 4 assignments

The following sections detail the assignments for Homework 4. 
Each task involves coding, which must be enclosed within designated markers. 
To ensure that your submission meets the requirements, adhere to the provided code structure and validate your solutions with the corresponding tests.

### Code Structure

Place your code between the following markers in the provided files:

```python
#########################
## YOUR CODE GOES HERE ##

...

## YOUR CODE ENDS HERE ##
#########################
```

### Assignment Validation

For each task, a test is available to verify its successful completion. A task is considered complete if and only if its corresponding test executes without any errors.


### Task 1: Implementation of the cartpole (15% of the grade)

You are to implement the cartpole system dynamics. The system's differential equations are provided in [notes.pdf](./notes.pdf)

Within [src/system.py](./src/system.py), locate the `CartPoleWithMotor` class and insert the correct expressions for the system's state derivatives:
```python
Dstate[0] = ... # derivative of \vartheta 
Dstate[1] = ... # derivative of x
Dstate[2] = ... # derivative of \omega
Dstate[3] = ... # derivative of v_x
```

Validate your implementation by running the test command:

```shell
pytest test.py::test_cartpole_system -v --disable-warnings  
```

Once the test passes, proceed to Task 2.


### Task 2: Derive the MPC controller for Cartpole Swingup Problem (85% of the grade)

The goal is to design an model-predictive controller to swing up the pole, and then position the cart at the origin. 

Please consult exercises 1 in [notes.pdf](./notes.pdf) for a foundational understanding of the subject.

Once you are ready find the `CartPoleMPC` class in [src/policy.py](./src/policy.py) and update it with the appropriate code.

To initiate the swing-up procedure, use:

```shell
python run.py policy=cartpole_mpc initial_conditions=cartpole_swingup system=cartpole --interactive --fps=10 
```

Assess your implementation with the following test:

```shell
pytest test.py::test_cartpole_swingup -v --disable-warnings 
```

## Submission Guide

Follow these simple steps to submit your homework:

1. Execute `bash prepare_for_submit.sh` in your terminal. You'll get a `src-hw4.tar.gz` file.
2. Visit our [Telegram bot](https://t.me/aida_att_bot).
3. Type `/submit_hw4` in the chat and upload your `src-hw4.tar.gz` file.
4. The bot will test your homework and provide a score.
5. Happy with your score? Upload `src-hw4.tar.gz` to the LMS for official grading.
6. Want to improve? Adjust your homework and resubmit with `/submit_hw4`.

That's it! Good luck with your homework! 

## Hw4 structure

- [`run.py`](./run.py): The main executable script.
- [`src/`](./src/): Contains the source code of the hw1.
    - [`policy.py`](./src/policy.py): Implements the PD and energy-based adaptive policies for cartpole system.
    - [`system.py`](./src/system.py): Implements the cartpole system.
- [`presets/`](./presets/): Houses configuration files.
    - [`common/`](./presets/common): General configurations.
        - [`common.yaml`](./presets/common/common.yaml): Settings for common variables (like sampling time)
    - [`policy/`](./presets/policy/): Policy-specific configurations.
        - [`cartpole_mpc.yaml`](./presets/policy/cartpole_mpc.yaml): Settings for the mpc policy for cartpole system.
        - [`cartpole_pd.yaml`](./presets/policy/cartpole_pd.yaml): Settings for the Proportional-Derivative (PD) regulator for cartpole system.
    - [`scenario/`](./presets/scenario/): Scenario configurations.
        - [`scenario.yaml`](./presets/scenario/scenario.yaml): Main orchestrator settings.
    - [`simulator/`](./presets/simulator/): Simulator configurations.
        - [`casadi.yaml`](./presets/simulator/casadi.yaml): Configurations for the [CasADi](https://web.casadi.org/) [RK](https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods) simulator.
    - [`system/`](./presets/system/): System specific configurations
        - [`cartpole.yaml`](./presets/system/cartpole.yaml): Configuration for cartpole system.