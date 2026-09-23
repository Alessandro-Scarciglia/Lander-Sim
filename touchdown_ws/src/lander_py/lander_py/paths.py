import os
from pathlib import Path


def workspace_root():
    configured_root = os.environ.get('TOUCHDOWN_WS_ROOT')
    if configured_root:
        return Path(configured_root).resolve()

    current_path = Path(__file__).resolve()
    for parent in current_path.parents:
        if (parent / 'cfg' / 'simulation.yaml').is_file() and (parent / 'src' / 'worlds').is_dir():
            return parent

    raise RuntimeError('Unable to locate the touchdown workspace root.')


def simulation_config_path():
    configured_path = os.environ.get('TOUCHDOWN_CONFIG_FILE')
    if configured_path:
        return Path(configured_path).resolve()

    return workspace_root() / 'cfg' / 'simulation.yaml'