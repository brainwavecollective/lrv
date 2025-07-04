import argparse
from .daemon import run_daemon

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

    args = build_parser().parse_args()
    run_daemon(args)