"""Shared SI configuration. MJCF inertias are checked against these assumptions."""
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[2]


def load_configuration():
    config = {}
    for name in ('quad_nominal', 'controller_gains', 'docking_mechanism', 'experiments'):
        with (ROOT / 'configs' / f'{name}.yaml').open() as stream:
            config.update(yaml.safe_load(stream))
    return config
