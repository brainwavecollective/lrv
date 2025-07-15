## LeRobo-Vous Daemon (LRVD) - The Robot Rendezvous 
Welcome to the web of robots.  

> Connect your robot to the world.

This little daemon is how robots find and meet each other across the internet.  Running `lrvd` will connect your robot and enable you to participate in peer-to-peer robotic telepresence. Simply spin up the daemons and your robots will auto-discover and auto-connect. You'll be given a URL to see the live video feed.  

## Features
- 🌐 Global network - connect with robots worldwide
- 📡 Real-time telepresence control
- 🚀 Easy setup and operation
- 🔄 One-time device registration, automatic discovery, automatic pairing
- 🔒 Secure peer-to-peer connections 

LeRobo-Vous connects you to a network of robots and teleoperators. This package only supports [LeRobot](https://github.com/huggingface/lerobot) robots.

LRV is designed for simplicity, allowing you to connect with just a single command. The first time you start the daemon there's a quick one-time registration. After that you will be automatically connected to a matching robot anytime you run the service. Default behavior restricts connectivity to your own robots.

> Why the name "LeRobo-Vous"?

“Rendezvous” is the French word for "meet"  
LeRobot Robots + Rendezvous = **LeRobo-Vous**

## Limitations 
Currently restricted to SO101 arms. I expect this will work with all LeRobot arms but I'd like to test it first before turning it on for everyone. If you're willing to be the first to try then please contact me on the LeRobot Discord ([I am LeDaniel](https://discord.com/users/769583125579169812)).

## Pre-Requisite Requirements
- [LeRobot](https://github.com/huggingface/lerobot) already installed and working with compatible robot hardware such as the [SO101](https://github.com/TheRobotStudio/SO-ARM100)  

That's it! LeRobot is all you need.  

## Installation

*Le Robot Rendezvous* is an homage to the LeRobot project by Hugging Face 🤗. Viva la open source! Thank you to the team for teaching us so much about robotics and AI.  

Before you start you'll need to activate your LeRobot Python environment. If you followed the [official instructions](https://huggingface.co/docs/lerobot/en/getting_started_real_world_robot) then you will run `conda activate lerobot`, but you may have also setup your environment differently using `uv venv lerobot`, `.venv/bin/activate`, `poetry shell`, etc..  

**STEP 1: Install this package:**  
```bash
pip install lrvd
```

**STEP 2: Run the LeRobo-Vous daemon and connect the station of your choosing:**   
```bash
# OPTION 1 - TELEOPERATOR (leader)
lrvd \
  --poste=teleop \
  --teleop.type=so101_leader \
  --teleop.port=<YOUR-TELEOP-PORT>
```   

```bash
# OPTION 2 - ROBOT (follower)
lrvd \
  --poste=robot \
  --robot.type=so101_follower \
  --robot.port=<YOUR-ROBOT-PORT> \
  --robot.cameras='{"front": {"index_or_path": "/dev/video0"}}'
```

## What happens next?
1. The first time you run the daemon you will need to [claim your robot](http://brainwavecollective.ai/lrv/robots?claim=)
2. Your device will pair with an available robot or teleoperator, and telepresence will begin automatically (currently locked to your telops/robots only)
3. With the session established you will be able to control the remote robot, or watch the remote teleoperation
4. When you want to disconnect, simply stop the `lrvd` daemon process 

You can view your connected robots and navigate to your active sessions at:  
[http://brainwavecollective.ai/lrv/robots](http://brainwavecollective.ai/lrv/robots)

Only you can see your connected robots.  Connected partners will only see your location and robot name.  

## Design Notes
Connecting two robots across the world (or across the room) requires a complex multi-step negotiation process.  

The robots can't just dial each other directly. Devices aren't aware that each other exists, let alone know who to connect to, or how to connect. In order to make all of this happen intermediate services are necessary to exchange contact information, identify connection paths, and ensure both sides have what they need to communicate directly with each other.  

This daemon is tightly integrated with that process. It abstracts the underlying complexity of matching, NAT traversal and signaling protocols.  It's the best of both worlds; you don't need to know or care how the complicated things happen, but you get to see exactly what code is running locally.  At this time relay servers are not provided but can be made avaialable if you have more complex networking needs.  

![LeRobo-Vous Overview](lrvd_overview.png)  

## Misc.

Although I think this version is ready to be released to the world, please keep in mind that this is a BRAND NEW approach and there is still a ton to be learned. This is NOT a mature process and you should be aware that you are connecting to an experiment in progress. I'll update all the formalities as we make more progess but in the meantime, feel free to contact me or submit issues if you encounter any problems.    

If you think this project is fun please leave a star ⭐ and tell your friends!  

Daniel  
The Brain Wave Collective    
