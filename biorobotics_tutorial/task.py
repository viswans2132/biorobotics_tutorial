#!/usr/bin/env python3
from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, Sequence, Tuple


Vec3 = Tuple[float, float, float]
Feet = Mapping[str, Vec3]
Keyframe = Mapping[str, object]


def make_keyframes(z_stance: float, z_swing: float) -> List[Dict[str, object]]:
    return [
        {"name": "start",
         "lf": (0.18,  0.17, z_stance), "rf": (0.18, -0.17, z_stance),
         "lh": (-0.18, 0.17, z_stance), "rh": (-0.18, -0.17, z_stance)},

        # shift_1 (+3 cm)
    ]


def duration_for(name: str, base_s: float) -> float:
    scale = 0.8 if "lift" in name or "down" in name else (1.2 if "shift" in name else 1.0)
    return max(0.05, base_s * scale)


def cmd_for_frame(commander: Path, namespace: str, frame: Mapping[str, object], base_duration_s: float) -> List[str]:
    name = str(frame["name"])
    feet = {k: frame[k] for k in ("lf", "rf", "lh", "rh")}
    dur = duration_for(name, base_duration_s)

    def fmt(flag: str, v: object) -> List[str]:
        x, y, z = v  # type: ignore[misc]
        return [flag, f"{float(x):.4f}", f"{float(y):.4f}", f"{float(z):.4f}"]

    ns_args = ["--namespace", namespace] if namespace else []
    return (
        [sys.executable, str(commander), "--duration", f"{dur:.3f}"]
        + ns_args
        + fmt("--lf", feet["lf"]) + fmt("--rf", feet["rf"]) + fmt("--lh", feet["lh"]) + fmt("--rh", feet["rh"])
    )


def run(commander: Path, namespace: str, base_duration_s: float, frames: Sequence[Mapping[str, object]]) -> None:
    for f in frames:
        print(f"[walk] {f['name']} (base={base_duration_s:.2f}s)")
        subprocess.run(cmd_for_frame(commander, namespace, f, base_duration_s), check=True)


def parse(argv: Sequence[str]) -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--namespace", default="")
    p.add_argument("--duration", type=float, default=0.30)
    p.add_argument("--z-stance", type=float, default=-0.30)
    p.add_argument("--z-swing", type=float, default=-0.27)  # <-- key fix (was -0.25)
    p.add_argument("--commander", default=str(Path(__file__).with_name("quadruped_IK_commander.py")))
    return p.parse_args(list(argv))


def main(argv: Sequence[str]) -> None:
    a = parse(argv)
    run(
        commander=Path(a.commander).expanduser().resolve(),
        namespace=str(a.namespace),
        base_duration_s=float(a.duration),
        frames=make_keyframes(float(a.z_stance), float(a.z_swing)),
    )


if __name__ == "__main__":
    main(sys.argv[1:])
