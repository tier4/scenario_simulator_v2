# Scenario expansion: .yaml (t4v2, possibly with ScenarioModifiers) into one
# plain .xosc per permutation. Pure Python -- no ROS graph required.

import sys
from pathlib import Path

from scenario_test_runner.scenario import substitute_ros_package


def expand_scenarios(labeled_scenarios, expand_dir: Path):
    """Expand [(label, path)] into [(label, xosc_path)] permutations."""
    from openscenario_utility.conversion import convert

    expanded = []
    for label, raw in labeled_scenarios:
        path = substitute_ros_package(Path(raw)).resolve()
        if not path.exists():
            sys.exit(f"error: scenario not found: {path}")
        if path.suffix == ".xosc":
            expanded.append((label, path))
        else:
            out = expand_dir / label
            out.mkdir(parents=True, exist_ok=True)
            permutations = [Path(p) for p in convert(path, out, False)]
            for xosc in permutations:
                name = label if len(permutations) == 1 else f"{label}_{xosc.stem}"
                expanded.append((name, xosc))
    if not expanded:
        sys.exit("error: scenario expansion produced no .xosc files "
                 "(check the conversion errors above)")
    return expanded


def label_scenario_paths(scenario_paths):
    """Derive unique labels for plain file path inputs."""
    labeled = []
    seen = set()
    for raw in scenario_paths:
        path = substitute_ros_package(Path(raw))
        # Files pulled from Web.Auto are all named "scenario.yml"; fall back to
        # parent directory names (scenario id / version) to keep labels unique.
        label = path.stem
        if label in seen or label == "scenario":
            label = f"{path.parent.parent.name[:8]}_{path.parent.name}_{path.stem}"
        seen.add(label)
        labeled.append((label, path))
    return labeled
