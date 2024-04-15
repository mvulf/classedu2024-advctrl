import pytest
import regelum as rg
from pathlib import Path
import sys
import subprocess
import os
import pandas as pd
from regelum import callback
from src.system import MyCartPole
import numpy as np
import os

test_results = []
if os.path.exists("test_results.csv"):
    os.remove("test_results.csv")


def save_and_assert(assert_condition: bool, test_id: str, weight: int):
    global test_results
    test_results.append(
        {"test_name": test_id, "result": weight if assert_condition else 0}
    )

    pd.DataFrame.from_records(test_results).to_csv("test_results.csv", index=False)

    assert assert_condition


def test_cartpole_system():
    system: MyCartPole = callback.detach(MyCartPole)()
    save_and_assert(
        np.allclose(
            system.compute_state_dynamics(
                None, np.array([[1, 2, 3, 4]]), np.array([[5]])
            ),
            np.array([[3.0, 4.0, 9.51362395, 3.53952662]]),
        ),
        test_id="test_cartpole_system",
        weight=15,
    )


def test_cartpole_hold():
    run_py = str(Path(__file__).parent / "run.py")
    subprocess.run(
        "python "
        + str(run_py)
        + " "
        + "policy=cartpole_pd "
        + "initial_conditions=cartpole_hold "
        + "system=cartpole",
        shell=True,
    )
    outputs = Path(__file__).parent / "regelum_data" / "outputs"
    latest_date = sorted(list(os.listdir(str(outputs))))[-1]
    latest_time = sorted(list(os.listdir(str(outputs / latest_date))))[-1]

    df = pd.read_hdf(
        outputs
        / latest_date
        / latest_time
        / "0"
        / ".callbacks"
        / "HistoricalDataCallback"
        / "observations_actions_it_00001.h5"
    )

    save_and_assert(
        df.iloc[:, 6:].tail(100).mean(axis=0).abs().max() < 0.05,
        test_id="test_cartpole_hold",
        weight=35,
    )


def test_cartpole_swingup():
    run_py = str(Path(__file__).parent / "run.py")
    subprocess.run(
        "python "
        + str(run_py)
        + " "
        + "policy=cartpole_energy_based "
        + "initial_conditions=cartpole_swingup "
        + "system=cartpole",
        shell=True,
    )
    outputs = Path(__file__).parent / "regelum_data" / "outputs"
    latest_date = sorted(list(os.listdir(str(outputs))))[-1]
    latest_time = sorted(list(os.listdir(str(outputs / latest_date))))[-1]

    df = pd.read_hdf(
        outputs
        / latest_date
        / latest_time
        / "0"
        / ".callbacks"
        / "HistoricalDataCallback"
        / "observations_actions_it_00001.h5"
    ).tail(100)
    save_and_assert(
        max(
            np.abs(np.sin(df.iloc[:, 6]).mean()),
            np.abs(np.cos(df.iloc[:, 6]).mean() - 1),
            df.iloc[:, 7:].mean(axis=0).abs().max(),
        )
        < 0.05,
        test_id="test_cartpole_swingup",
        weight=50,
    )
