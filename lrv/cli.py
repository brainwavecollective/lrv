import argparse
import sys
from pathlib import Path
from importlib.metadata import version, PackageNotFoundError
from packaging import version as pkg_version
from .daemon import run_daemon

# Handle tomllib/tomli compatibility
try:
    import tomllib  # Python 3.11+
except ImportError:
    import tomli as tomllib  # Python 3.10 and below

from .daemon import run_daemon




# Error messages for lerobot version issues
LEROBOT_NOT_INSTALLED = """
lerobot is required but not installed. did you activate your environment?

Please install lerobot v0.2.0 or higher.
"""

LEROBOT_VERSION_TOO_OLD = """
WARNING: LeRobot version {installed_version} is installed, but version {required_version} or higher is required. LeRobot version 0.1.0 can be used, but only after June 1st 2025. It is possible that you already have the latest code, but the version is not recognized because this is an older development installation. You can ignore this warning if you don't encounter any issues.

OPTION 1: Upgrade your LeRobot installation to use a version of the code from after July 21st 2025 
  cd <your lerobot installation directory> 
  git pull 
  pip install -e . 

OPTION 2: If using LeRobot prior to June 1st 2025, you will need to use lrvd v0.1.0
    pip install lrvd==0.1.0 --force-reinstall
    
Please feel free to create an issue if you have questions

  https://github.com/brainwavecollective/lrv/issues
  
"""

def _check_lerobot_version():
    """Check that lerobot meets minimum version requirement."""
    # Read requirement from pyproject.toml
    try:
        pyproject_path = Path(__file__).parent.parent / "pyproject.toml"
        with open(pyproject_path, "rb") as f:
            pyproject = tomllib.load(f)
        required_version = pyproject["tool"]["lrvd"]["requirements"]["lerobot"]
    except (FileNotFoundError, KeyError, tomllib.TOMLDecodeError):
        # Fallback if we can't read the requirement
        required_version = "0.2.0"
    
    # Check if lerobot is importable
    try:
        import lerobot
    except ImportError:
        print(LEROBOT_NOT_INSTALLED)
        sys.exit(1)
    
    # Check version using importlib.metadata
    try:
        installed_version = version('lerobot')
    except PackageNotFoundError:
        # Fallback to __version__ attribute
        if hasattr(lerobot, '__version__'):
            installed_version = lerobot.__version__
        else:
            # Can't determine version, assume it's okay
            return
    
    # Compare versions
    if pkg_version.parse(installed_version) < pkg_version.parse(required_version):
        print(LEROBOT_VERSION_TOO_OLD.format(
            installed_version=installed_version,
            required_version=required_version
        ))
        sys.exit(1)

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="LRV daemon - LeRobo-Vous robot telepresence")
    p.add_argument("--poste", required=True, choices=["teleop", "robot"], 
                   help="Mode: 'teleop' (control robots) or 'robot' (be controlled)")
    p.add_argument("--mode", default="solo", 
                   help="Session mode (default: solo)")
    p.add_argument("--teleop.type", dest="teleop_type", default="so101_leader",
                   help="Teleoperator type (default: so101_leader)")
    p.add_argument("--teleop.port", dest="teleop_port", default="/dev/ttyACM_LEADER",
                   help="Teleoperator port (default: /dev/ttyACM_LEADER)")
    p.add_argument("--teleop.id", dest="teleop_id", help="Teleoperator ID")
    p.add_argument("--robot.id", dest="robot_id", help="Robot ID")
    p.add_argument("--robot.type", dest="robot_type", default="so101_follower",
                   help="Robot type (default: so101_follower)")
    p.add_argument("--robot.port", dest="robot_port", default="/dev/ttyACM_FOLLOWER",
                   help="Robot port (default: /dev/ttyACM_FOLLOWER)")
    p.add_argument("--robot.cameras", dest="robot_cameras", help="Full camera config dict as JSON")
    p.add_argument("--fps", type=int, default=30,
                   help="Control loop FPS (default: 30)")
    p.add_argument("--reset-secret", action="store_true",
                   help="Clear saved device credentials")
    p.add_argument("--log-level", choices=["DEBUG", "INFO", "WARNING", "ERROR"], 
                   default="INFO", help="Set logging level (default: INFO)")
               
    return p
    
def main() -> None:
    print("""
    
    LeRobo-Vous  
    a Brain Wave Collective project
    """, flush=True)
    
    _check_lerobot_version()
    
    args = build_parser().parse_args()
    run_daemon(args)