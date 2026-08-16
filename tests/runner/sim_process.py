"""Gestion du cycle de vie de la pile simulée pour un scénario.

Chaque scénario tourne dans son propre ROS_DOMAIN_ID. En P1 les scénarios
s'exécutent en série (domain_id = 0) ; le parallélisme réel arrive en P6 avec
les campagnes Monte-Carlo — voir la note dans SimStack._env().
"""

from __future__ import annotations

import os
import signal
import subprocess
import time
from dataclasses import dataclass, field
from pathlib import Path


def repo_root() -> Path:
    """Racine du dépôt, déduite de l'emplacement de ce fichier."""
    return Path(__file__).resolve().parents[2]


def px4_dir() -> Path:
    """Emplacement de PX4, cloné hors du dépôt (docs/adr/0008-px4-hors-depot.md)."""
    return Path(os.environ.get("DRONOTO_PX4_DIR", Path.home() / "px4" / "PX4-Autopilot"))


@dataclass
class SimStack:
    """Pile simulée : PX4 + Gazebo + agent uXRCE-DDS + nœuds embarqués."""

    domain_id: int = 0
    headless: bool = True
    params_file: Path | None = None
    autostart: bool = True
    px4_boot_wait_s: float = 12.0
    _procs: list[subprocess.Popen] = field(default_factory=list)

    def _env(self) -> dict[str, str]:
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = str(self.domain_id)
        env["HEADLESS"] = "1" if self.headless else "0"
        env["DRONOTO_REPO"] = str(repo_root())
        env["DRONOTO_PX4_DIR"] = str(px4_dir())
        env["PX4_UXRCE_DDS_PORT"] = str(8888 + self.domain_id)
        env["GZ_PARTITION"] = f"dronoto{self.domain_id}"
        # LIMITE CONNUE : isoler complètement deux simulations concurrentes
        # demande aussi de configurer le port côté CLIENT PX4 (paramètre
        # UXRCE_DDS_PRT), ce qui n'est pas fait ici. En P1 les scénarios
        # s'exécutent en série, donc domain_id vaut toujours 0. Le parallélisme
        # réel arrive en P6, où cette limite devra être levée.
        return env

    def start(self) -> None:
        env = self._env()

        sim = subprocess.Popen(
            ["bash", str(repo_root() / "infrastructure" / "scripts" / "run_sim.sh")],
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        self._procs.append(sim)
        time.sleep(self.px4_boot_wait_s)

        cmd = [
            "ros2", "launch", "dronoto_bringup", "bringup_drone.launch.py",
            f"autostart:={'true' if self.autostart else 'false'}",
        ]
        if self.params_file is not None:
            cmd.append(f"params_file:={self.params_file}")

        drone = subprocess.Popen(
            cmd,
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        self._procs.append(drone)

    def stop(self) -> None:
        # Tuer le GROUPE de processus : PX4 et Gazebo lancent des enfants qui
        # survivraient à un kill du seul parent et bloqueraient l'exécution
        # suivante — un orphelin rend le test suivant non reproductible, ce qui
        # est pire qu'un échec franc.
        for proc in reversed(self._procs):
            if proc.poll() is not None:
                continue
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            except (ProcessLookupError, PermissionError):
                continue
        time.sleep(2.0)
        for proc in reversed(self._procs):
            if proc.poll() is None:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except (ProcessLookupError, PermissionError):
                    pass
        self._procs.clear()

        for pattern in (
            "px4_sitl_default/bin/px4",
            "gz sim",
            "MicroXRCEAgent",
            "bringup_drone.launch.py",
        ):
            subprocess.run(["pkill", "-9", "-f", pattern], check=False)
        time.sleep(1.0)

    def __enter__(self) -> SimStack:
        self.start()
        return self

    def __exit__(self, *exc_info) -> None:
        self.stop()
