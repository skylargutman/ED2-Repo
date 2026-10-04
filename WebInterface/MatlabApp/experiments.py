import json
import os

from dataclasses import dataclass, field


@dataclass(slots=True)
class Experiment:
    tag: str = ""
    name: str = ""
    parameters: dict = field(default_factory=dict)


def load_experiment_from_file(experiment, exp_name):
    exp_file = f"experiments/{exp_name}"

    with open(exp_file, "r", encoding="utf-8") as experiment_json_file:
        exp_dict = json.load(experiment_json_file)
        if exp_dict.get("id") is None:
            raise ValueError("Getting name from dict is returning None.")
        experiment.tag = exp_dict["id"]
        experiment.name = exp_dict["name"]

        for param in exp_dict["parameters"]:
            experiment.parameters[param] = exp_dict["parameters"][param]


def load_experiment_files():
    experiment_files = os.listdir("experiments")
    experiments_dict = {}

    for experiment_file_name in experiment_files:
        experiment = Experiment()
        load_experiment_from_file(experiment, experiment_file_name)
        experiments_dict[experiment.tag] = experiment

    return experiments_dict


EXPERIMENTS = load_experiment_files()
