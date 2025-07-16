#!/usr/bin/env python3
"""
LeRobo-Vous - the Robot Rendezvous
A robot-robot bridge for LeRobot

A project by the:
~~~ B R A I N  W A V E  C O L L E C T I V E ~~~
https://github.com/brainwavecollective/lrv

This daemon enables automatic discovery, connectivity, and telepresense with other LeRobot participants around the world. 

You can choose to be a teleoperator if you want to control other robots (leader), or as a robot if you want to allow others to teleop your robot (follower). See README for more info.

Authored by Daniel Ritchie
@LeDaniel[quantumpoet] on the LeRobot Discord
@brainwavecoder9 on GitHub

Use examples:

TELEOPERATOR STATION (leader sending the control signals):
lrvd \
  --poste=teleop \
  --teleop.type=so101_leader \
  --teleop.port=<YOUR-TELEOP-PORT>```   

ROBOT STATION (robot to be controlled and broadcast video):
lrvd \
  --poste=robot \
  --robot.type=so101_follower \
  --robot.port=<YOUR-ROBOT-PORT> \
  --robot.cameras='{"gripper": {"index_or_path": "/dev/<YOUR-CAMERA>"}}'```

"""

import argparse
import asyncio
import json
import logging
import platform
import sys
import time
from dataclasses import asdict, dataclass
from fractions import Fraction
from pathlib import Path
from typing import Optional, Dict, Any

import aiohttp
import av
import cv2
import numpy as np
import websockets
from aiortc import RTCPeerConnection, RTCDataChannel, RTCConfiguration, RTCIceServer, RTCSessionDescription, RTCIceCandidate
from aiortc.contrib.media import MediaStreamTrack

from lerobot.robots import Robot, RobotConfig, make_robot_from_config
from lerobot.teleoperators import Teleoperator, TeleoperatorConfig, make_teleoperator_from_config
from lerobot.utils.utils import init_logging, move_cursor_up

try:
    from importlib.metadata import version
    DAEMON_VERSION = version("lrv")
except Exception:
    DAEMON_VERSION = "0.0.0"  
   
# Gateway configuration
GATEWAY_URL = "https://api.brainwavecollective.ai/lrv/gateway"

# Phase execution result
@dataclass
class PhaseResult:
    context: dict
    next_phase: Optional[str] = None
    complete: bool = False

@dataclass
class LeConfig:
    poste: str 
    robot: Optional[RobotConfig] = None
    teleop: Optional[TeleoperatorConfig] = None
    fps: int = 30
    gateway_url: str = GATEWAY_URL
    mode: str = "solo"

# ===== LOGGING UTILITIES =====
class NoNewlineFormatter(logging.Formatter):
    """Custom formatter that supports no_newline extra flag and dots_only flag"""
    def format(self, record):
        # If it’s a no-newline log or a dots-only log, just emit the raw message
        if getattr(record, 'no_newline', False) or getattr(record, 'dots_only', False):
            return record.getMessage()
        # Otherwise use the active format string
        return super().format(record)

class DynamicFormatter(NoNewlineFormatter):
    """
    Switch format strings based on the root logger’s level.
    - If the root level is DEBUG: include module names
    - Otherwise: drop module names
    """
    def __init__(self, fmt_info, fmt_debug, datefmt=None):
        super().__init__(fmt_info, datefmt=datefmt)
        self.fmt_info  = fmt_info
        self.fmt_debug = fmt_debug

    def format(self, record):
        root_level = logging.getLogger().getEffectiveLevel()
        if root_level == logging.DEBUG:
            self._style._fmt = self.fmt_debug
        else:
            self._style._fmt = self.fmt_info
        return super().format(record)

class ProgressHandler(logging.StreamHandler):
    """Custom handler that supports progressive output (dots, etc.)"""
    def emit(self, record):
        try:
            msg = self.format(record)
            stream = self.stream
            if getattr(record, 'no_newline', False):
                stream.write(msg)
                stream.flush()
            else:
                stream.write(msg + self.terminator)
                stream.flush()
        except Exception:
            self.handleError(record)
            
class ContextAdapter(logging.LoggerAdapter):
    """Simplified logger adapter - context injection only when explicitly needed"""
    
    def __init__(self, logger, context_provider=None):
        super().__init__(logger, {})
        self.context_provider = context_provider
    
    def process(self, msg, kwargs):
        # No automatic context injection - just pass through
        return msg, kwargs
    
    def log_with_context(self, level, msg, *args, **kwargs):
        """Explicitly inject context when needed"""
        if self.context_provider and callable(self.context_provider):
            try:
                context = self.context_provider()
                if context:
                    # Add key context fields to the message
                    context_parts = []
                    for key in ['session_id', 'robot_instance_id', 'phase', 'robot_id']:
                        if key in context and context[key]:
                            context_parts.append(f"{key}={context[key]}")
                    
                    if context_parts:
                        msg = f"[{','.join(context_parts)}] {msg}"
            except Exception:
                pass  # Don't let context injection break logging
        
        self.logger.log(level, msg, *args, **kwargs)
   
def setup_custom_logging(log_level: str):
    """Setup custom logging with level‐dependent formatting and progress support"""
    import logging
    # Remove any existing handlers
    root = logging.getLogger()
    for h in root.handlers[:]:
        root.removeHandler(h)

    # Create and configure our progress handler
    handler = ProgressHandler()
    fmt_info  = '%(asctime)s - %(levelname)s - %(message)s'
    fmt_debug = '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    formatter = DynamicFormatter(fmt_info, fmt_debug, datefmt='%Y-%m-%d %H:%M:%S')
    handler.setFormatter(formatter)
    root.addHandler(handler)

    # Set global level based on user choice
    root.setLevel(getattr(logging, log_level))

    # ─── Drop all INFO‐level logs from aioice.ice ───
    ice_logger = logging.getLogger("aioice.ice")
    ice_logger.addFilter(lambda record: record.levelno != logging.INFO)
    # ───────────────────────────────────────────────

    # Reduce noise from other third-party libraries
    logging.getLogger('aiohttp').setLevel(logging.WARNING)
    logging.getLogger('websockets').setLevel(logging.WARNING)

    logging.getLogger('aiortc').setLevel(logging.WARNING)
    logging.getLogger('aiortc.rtcpeerconnection').setLevel(logging.WARNING)
    logging.getLogger('aiortc.rtcicetransport').setLevel(logging.WARNING)
    logging.getLogger('aiortc.rtcdtlstransport').setLevel(logging.WARNING)
    logging.getLogger('aiortc.mediastreams').setLevel(logging.WARNING)

    logging.getLogger('lerobot.teleoperators').setLevel(logging.INFO)
    logging.getLogger('lerobot.robots').setLevel(logging.INFO)

    
# ===== TRANSPORT LAYER =====

class LeGatewayPortier:
    """Connect your robot to the world"""

    def __init__(self, config: LeConfig):
        self.config = config
        self.session = None
        self.device_secret = None
        self.logger = logging.getLogger(f"{__name__}.LeGatewayPortier")

    async def __aenter__(self):
        """Async context manager entry"""
        self.session = aiohttp.ClientSession()
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        """Async context manager exit"""
        if self.session:
            await self.session.close()

    def set_device_secret(self, secret: str):
        """Set device secret for authentication"""
        self.device_secret = secret

    async def pulse(self, action: str, data: dict = None) -> dict:
        """Send pulse to gateway with unified format"""
        payload = {"pulse": action}
        if data:
            payload["data"] = data

        headers = {
            "Content-Type": "application/json",
            "X-Daemon-Version": DAEMON_VERSION
        }

        # Add auth header for authenticated actions
        if self.device_secret and action not in ["register", "verify"]:
            headers["Authorization"] = f"Bearer {self.device_secret}"

        self.logger.debug(f"Sending pulse: {action} to {self.config.gateway_url}")

        try:
            async with self.session.post(
                f"{self.config.gateway_url}/pulse",
                json=payload,
                headers=headers,
                timeout=aiohttp.ClientTimeout(total=10)
            ) as resp:
                if resp.status != 200:
                    error_text = await resp.text()
                    raise Exception(f"HTTP {resp.status}: {error_text}")

                response = await resp.json()

                # detailed debug on first-phase actions
                if action in ["signaling", "register", "verify"]:
                    self.logger.debug(f"Full response for {action}: {response}")
                else:
                    self.logger.debug(f"Response: success={response.get('success')}")

                return response

        except asyncio.TimeoutError:
            # soft-fail on timeouts: log a warning and return a failed pulse
            self.logger.warning(f"Pulse timed out, continuing...")
            return {"success": False, "timeout": True}

        except Exception:
            # everything else still gets logged as an exception
            self.logger.exception(f"🚩 Pulse error for action '{action}'")
            raise


# ===== SUPPORTING CLASSES =====

class LeVault:
    """Manages persistent device identity (persists beyond sessions)"""

    def __init__(self, config: LeConfig):
        self.config = config
        self.config_dir = Path.home() / ".lerobo-vous" / "secrets"
        robot_type = self._get_robot_type()
        self.secret_file = self.config_dir / f"{robot_type}.json"
        self.logger = logging.getLogger(f"{__name__}.LeVault")

    def _get_robot_type(self) -> str:
        """Get robot type identifier for secret storage"""
        if self.config.poste == "robot" and self.config.robot:
            class_name = self.config.robot.__class__.__name__.lower()
            return f"robot_{class_name}"
        elif self.config.poste == "teleop" and self.config.teleop:
            class_name = self.config.teleop.__class__.__name__.lower()
            return f"teleop_{class_name}"
        else:
            return f"{self.config.poste}_unknown"

    def save_device_secret(self, device_secret: str, robot_instance_id: str, robot_id: str):
        """Save device identity to disk"""
        try:
            self.config_dir.mkdir(parents=True, exist_ok=True)
            data = {
                "device_secret": device_secret,
                "robot_instance_id": robot_instance_id,
                "robot_id": robot_id,
                "robot_type": self._get_robot_type(),
                "saved_at": time.time()
            }
            with open(self.secret_file, 'w') as f:
                json.dump(data, f, indent=2)
            self.logger.info(f"Device identity saved")
            self.logger.debug(f"Identity file location {self.secret_file}")
        except Exception as e:
            self.logger.exception("Failed to save device secret")

    def load_device_secret(self) -> Optional[Dict[str, str]]:
        """Load device identity from disk"""
        try:
            if not self.secret_file.exists():
                self.logger.debug(f"No secret file found at {self.secret_file}")
                return None

            with open(self.secret_file, 'r') as f:
                data = json.load(f)

            required_fields = ["device_secret", "robot_instance_id", "robot_id"]
            if all(field in data for field in required_fields):
                stored_type = data.get("robot_type")
                current_type = self._get_robot_type()

                if stored_type != current_type:
                    self.logger.info(f"Robot type changed from {stored_type} to {current_type}, will re-register")
                    return None

                self.logger.debug(f"Loaded device secret from {self.secret_file}")
                return data
            else:
                self.logger.warning(f"Secret file missing required fields: {required_fields}")
                return None

        except Exception as e:
            self.logger.exception("Failed to load device secret")
            return None

    def clear_device_secret(self):
        """Clear saved device identity"""
        try:
            if self.secret_file.exists():
                self.secret_file.unlink()
                self.logger.info(f"✅ Device identity cleared from {self.secret_file}")
            else:
                self.logger.debug("No device secret to clear")
        except Exception as e:
            self.logger.exception("Failed to clear device secret")

class LeCamera(MediaStreamTrack):
    """Video track that reads from LeRobot camera"""
    kind = "video"
    
    def __init__(self, camera, camera_name="unknown"):
        super().__init__()
        self.camera = camera
        self.camera_name = camera_name
        self.counter = 0
        self.last_frame = None
        self.frame_rate = 30
        self.frame_duration = 1.0 / self.frame_rate
        self.logger = logging.getLogger(f"{__name__}.LeCamera.{camera_name}")
        self.consecutive_errors = 0

    async def recv(self):
        try:
            self.counter += 1
            
            try:
                frame = self.camera.read()
                if frame is not None:
                    self.last_frame = frame
                    self.consecutive_errors = 0  # Reset error count on success
            except Exception as cam_error:
                self.consecutive_errors += 1
                self.logger.warning(
                    f"Camera read error (frame {self.counter}, consecutive errors: {self.consecutive_errors}): {cam_error}"
                )
                if self.last_frame is not None:
                    frame = self.last_frame
                    self.logger.debug(f"Using last good frame for frame {self.counter}")
                else:
                    frame = np.zeros((480, 640, 3), dtype=np.uint8)
                    self.logger.debug(f"No previous frame, using black frame for frame {self.counter}")
            
            if frame is None:
                self.logger.warning(f"Camera returned None for frame {self.counter}")
                frame = np.zeros((480, 640, 3), dtype=np.uint8)
            
            if len(frame.shape) == 3 and frame.shape[2] == 3:
                if hasattr(cv2, 'COLOR_BGR2RGB'):
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                format_name = "rgb24"
            elif len(frame.shape) == 2:
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)
                format_name = "rgb24"
            else:
                format_name = "rgb24"
            
            av_frame = av.VideoFrame.from_ndarray(frame, format=format_name)
            av_frame.pts = self.counter
            av_frame.time_base = Fraction(1, self.frame_rate)
            
            return av_frame
            
        except Exception as e:
            self.logger.exception(f"Frame processing error for frame {self.counter}")
            error_frame = np.zeros((480, 640, 3), dtype=np.uint8)
            error_frame[240:250, :] = [255, 0, 0]  # Red stripe
            
            av_frame = av.VideoFrame.from_ndarray(error_frame, format="rgb24")
            av_frame.pts = self.counter
            av_frame.time_base = Fraction(1, self.frame_rate)
            return av_frame

# ===== MANAGER CLASSES =====

class RegistrationManager:
    """Domain-specific command processor for registration"""

    def __init__(self, gateway: LeGatewayPortier, vault: LeVault, config: LeConfig):
        self.gateway = gateway
        self.vault = vault
        self.config = config
        self.logger = logging.getLogger(f"{__name__}.RegistrationManager")
        self.dots_printed = 0  # Track dots for verification polling

    async def execute(self, context: dict) -> PhaseResult:
        """Execute registration phase - handles internal flow autonomously"""
        
        # Step 1: Check existing credentials first
        if not context.get("hello_checked"):
            result = await self._check_hello(context)
            if result["hello_success"]:
                return PhaseResult(context=result, next_phase="match", complete=True)
            context.update(result)
            context["hello_checked"] = True
            return PhaseResult(context=context)  # Continue registration
        
        # Step 2: Create registration if needed
        if not context.get("registration_created"):
            result = await self._register_device(context)
            context.update(result)
            return PhaseResult(context=context)  # Continue to verification
        
        # Step 3: Wait for verification
        result = await self._await_verification(context)
        context.update(result)
        
        if context.get("verified"):
            # Clear the dots line
            if self.dots_printed > 0:
                self.logger.info("")  # New line after dots
            return PhaseResult(context=context, next_phase="match", complete=True)
        
        return PhaseResult(context=context)  # Continue waiting

    async def _check_hello(self, context: dict) -> dict:
        """Check if existing credentials are valid"""
        self.logger.debug("Checking existing credentials...")
        
        # Load saved credentials
        saved_data = self.vault.load_device_secret()
        if not saved_data:
            self.logger.debug("No saved credentials found, will need to register")
            context["hello_success"] = False
            context["needs_registration"] = True
            return context
        
        # Set credentials in gateway
        self.gateway.set_device_secret(saved_data["device_secret"])
        context.update(saved_data)
        
        # Test with gateway
        try:
            response = await self.gateway.pulse("hello")
            if response.get("success"):
                context["hello_success"] = True
                context["verified"] = True
                self.logger.info("Credentials are valid")
            else:
                context["hello_success"] = False
                context["needs_registration"] = True
                self.vault.clear_device_secret()
                self.logger.warning("❌ Existing credentials were rejected by gateway")
        except Exception as e:
            context["hello_success"] = False
            context["needs_registration"] = True
            self.logger.warning(f"❌ Hello check failed, will re-register: {e}")
            
        return context

    async def _register_device(self, context: dict) -> dict:
        """Create new device registration"""
        self.logger.info("✅ Creating new device registration...")
        
        robot_type = "SO101"
        if self.config.robot and hasattr(self.config.robot, '__class__'):
            robot_type = self.config.robot.__class__.__name__.upper()
        
        try:
            response = await self.gateway.pulse("register", {
                "poste": self.config.poste,
                "type": robot_type
            })
            
            if not response.get("success"):
                raise Exception(f"Registration failed: {response.get('error')}")
            
            # Update context with registration info
            context["robot_instance_id"] = response["robot_instance_id"]
            context["polling_token"] = response["polling_token"]
            context["temp_code"] = response["temp_code"]
            context["registration_created"] = True
            
            # Show URL to user - using logger for consistency
            url = f"https://brainwavecollective.ai/lrv/robots?claim={context['temp_code']}&type={self.config.poste.capitalize()}"
            self.logger.info("=" * 60)
            self.logger.info("DEVICE REGISTRATION REQUIRED")
            self.logger.info("=" * 60)
            self.logger.info(f"Please claim your device at: {url}")
            self.logger.info("=" * 60)
            self.logger.info("This is a temporary registration code.")
            
            self.dots_printed = 0  # Reset dots counter
            return context
            
        except Exception as e:
            context["registration_error"] = str(e)
            self.logger.exception("❌ Device registration failed")
            raise

    async def _await_verification(self, context: dict) -> dict:
        """Check if user has claimed the device"""
        if not context.get("polling_token"):
            raise Exception("No polling token - registration not started")
        
        try:
            response = await self.gateway.pulse("verify", {
                "polling_token": context["polling_token"]
            })
            
            if not response.get("success"):
                error_msg = response.get("error", "Verification failed")
                if "expired" in error_msg.lower():
                    context["registration_expired"] = True
                    self.logger.error("Registration has expired, please restart")
                    raise Exception("Registration expired")
                raise Exception(f"Verification error: {error_msg}")
            
            current_state = response.get("current_state")
            
            if current_state == "verified":
                # Save credentials
                device_secret = response["device_secret"]
                robot_id = response["robot_id"]
                
                self.vault.save_device_secret(
                    device_secret,
                    context["robot_instance_id"],
                    robot_id
                )
                
                # Update gateway and context
                self.gateway.set_device_secret(device_secret)
                context["device_secret"] = device_secret
                context["robot_id"] = robot_id
                context["verified"] = True
                
                # Clear the dots line before success message
                if self.dots_printed > 0:
                    self.logger.info("")  # New line to clear dots
                self.logger.info("✅ Device registration verified successfully!")
                
            elif current_state == "verifying":
                # Show polling dots - use no_newline to keep extending the line
                self.dots_printed += 1
                
                if self.dots_printed == 1:
                    # First dot - show the initial message with timestamp
                    self.logger.info(" Waiting for device to be claimed...", extra={'no_newline': True})
                
                # Add dots progressively with no formatting
                self.logger.info(".", extra={'no_newline': True, 'dots_only': True})
                context["waiting_for_verification"] = True
                
            return context
            
        except Exception as e:
            context["verification_error"] = str(e)
            if "expired" not in str(e):
                self.logger.exception("❌ Verification check failed")
            raise
            
class MatchManager:
    """Domain-specific command processor for matching"""

    def __init__(self, gateway: LeGatewayPortier, config: LeConfig):
        self.gateway = gateway
        self.config = config
        self.logger = logging.getLogger(f"{__name__}.MatchManager")
        self.dots_printed = 0

    async def execute(self, context: dict) -> PhaseResult:
        """Execute matching phase - handles internal flow autonomously"""
        
        # Step 1: Send ready state if not sent
        if not context.get("ready_sent"):
            result = await self._send_ready(context)
            context.update(result)
            return PhaseResult(context=context)  # Continue to seeking
        
        # Step 2: Seek partner
        result = await self._seek_partner(context)
        context.update(result)
        
        if context.get("match_found"):
            # Clear the dots line
            if self.dots_printed > 0:
                self.logger.info("")  # New line after dots
            return PhaseResult(context=context, next_phase="webrtc", complete=True)
        
        return PhaseResult(context=context)  # Continue seeking

    async def _send_ready(self, context: dict) -> dict:
        """Send ready state with configuration"""
        self.logger.debug("Sending ready state to gateway...")
        
        configuration = {
            "poste": self.config.poste,
            "daemon_version": DAEMON_VERSION,
            "timestamp": time.time()
        }
        
        try:
            self.logger.debug("Calling pulse with ready configuration...")
            response = await self.gateway.pulse("ready", {
              "configuration": configuration,  
              "mode": self.config.mode        
            })
            self.logger.debug(f"Ready pulse completed: {response}")
            success = response.get("success", False)
            context["ready_sent"] = success
            
            if success:
                self.logger.info("✅ Ça roule")
                self.dots_printed = 0  # Reset dots counter
            else:
                self.logger.warning("❌ Failed to send ready state")
                
            return context
        except Exception as e:
            self.logger.exception("Failed to send ready state")
            raise

    async def _seek_partner(self, context: dict) -> dict:
        response = await self.gateway.pulse("seeking")
        if not response.get("success", False):
            self._print_seeking_dot(context)
            return context

        state = response.get("current_state", "seeking")
        if state in ("match_found", "matched_pending"):
            context["match_found"] = True
            context["session_id"] = response.get("session_id")
            context["role"]       = response.get("role")
            context["poste"]      = response.get("poste")

            if self.dots_printed > 0:
                import sys
                sys.stdout.write("\n")
                sys.stdout.flush()
                self.dots_printed = 0

            self.logger.info("Partner found!")
            self.logger.info("Establishing connection...")
            self.logger.debug(f"Session ID: {context['session_id']}")
            self.logger.debug(f"Role:       {context['role']}")
        else:
            self._print_seeking_dot(context)

        return context

    def _print_seeking_dot(self, context: dict):
        self.dots_printed += 1
        if self.dots_printed == 1:
            self.logger.info(" LeRobot is seeking...", extra={'no_newline': True})
        self.logger.info(".", extra={'no_newline': True, 'dots_only': True})
        context["seeking"] = True
    
# ===== WEBRTC LOCAL DOMAIN ORCHESTRATOR =====
async def connect_with_retry(websocket_url, max_duration=600):
    start = time.time()
    backoff = 0.1 
    attempt = 1
    logger = logging.getLogger(f"{__name__}.connect_with_retry")
    
    while time.time() - start < max_duration:
        try:
            logger.debug(f"WebSocket connection attempt {attempt}")
            return await websockets.connect(websocket_url)
            
        except websockets.exceptions.InvalidStatusCode as e:
            if e.status_code == 409:
                elapsed = time.time() - start
                logger.debug(f"Conflict (409), retrying in {backoff*1000:.0f}ms (elapsed: {elapsed:.1f}s)")
                await asyncio.sleep(backoff)
                backoff = min(backoff * 2, 2.0) 
                attempt += 1
                continue
            else:
                raise
        except Exception as e:
            logger.exception(f"WebSocket connection error on attempt {attempt}")
            raise
    
    raise Exception(f"WebSocket connection timed out after {max_duration}s")
    
@dataclass
class RendezProtocole:
    """Configuration for a WebRTC connection"""
    connection_type: str  
    credentials: dict
    needs_data_channel: bool = False
    needs_video_track: bool = False
    video_direction: str = "recvonly"  
    
class RendezLien:
    """Unified WebRTC connection handler"""
    
    def __init__(self, config: RendezProtocole, manager: 'RendezMaitre'):
        self.config = config
        self.manager = manager
        self.type = config.connection_type
        self.logger = logging.getLogger(f"{__name__}.RendezLien.{self.type}")
        
        # Connection objects
        self.pc: Optional[RTCPeerConnection] = None
        self.ws = None
        self.data_channel = None
        self.message_queue = asyncio.Queue() if config.needs_data_channel else None
        
        # State tracking
        self.signaling_connected = False
        self.negotiation_complete = False
        self.ice_gathering_complete = False
        self.connection_attempting = False
        self.connection_established = False
        self.data_channels_ready = False
        self.disconnected = False
        
        # Events for state changes
        self.signaling_event = asyncio.Event()
        self.negotiation_event = asyncio.Event()
        self.ice_gathering_event = asyncio.Event()
        self.connection_attempt_event = asyncio.Event()
        self.connection_established_event = asyncio.Event()
        self.data_channels_event = asyncio.Event()
        self.disconnect_event = asyncio.Event()

    async def setup(self):
        """Setup the WebRTC connection"""
        await self.manager.update_state(f"{self.type}_state", "connecting")
        
        # Create peer connection
        self.pc = RTCPeerConnection(configuration=RTCConfiguration(
            iceServers=[RTCIceServer(urls=[
                "stun:stun.l.google.com:19302",
                "stun:global.stun.twilio.com:3478"
            ])]
        ))
        
        # Setup event handlers
        self._setup_handlers()
        
        # Connect to signaling server
        creds = self.config.credentials
        
        # DEBUG: Log available credential keys
        self.logger.debug(f"Available credential keys: {list(creds.keys())}")
        
        # Handle different possible token field names
        token = None
        possible_token_fields = ['robot_token', 'token', 'auth_token', 'session_token']
        
        for field in possible_token_fields:
            if field in creds:
                token = creds[field]
                self.logger.debug(f"Found token in field: {field}")
                break
        
        if not token:
            available_fields = list(creds.keys())
            raise Exception(f"No token field found in credentials. Available fields: {available_fields}")
        
        ws_url = (f"{creds['signaling_server']}"
                 f"?session_id={creds['session_id']}"
                 f"&token={token}&type={self.type}")
        
        self.logger.debug("Connecting to signaling server...")
        self.ws = await connect_with_retry(ws_url)
        
        # Set signaling connected
        self.signaling_connected = True
        self.signaling_event.set()
        await self.manager.update_state(f"{self.type}_state", "signaling_connected")
        
        # Start signaling handler
        asyncio.create_task(self._handle_signaling())
        
        # Create data channel if needed (for control connections on teleop side)
        if self.config.needs_data_channel and self.manager.config.poste == "teleop":
            self.data_channel = self.pc.createDataChannel(
                self.type,
                ordered=False,
                maxRetransmits=0
            )
            self._setup_data_channel_handlers()

    def _setup_handlers(self):
        """Setup WebRTC event handlers"""
        @self.pc.on("connectionstatechange")
        async def on_connection_change():
            state = self.pc.connectionState
            self.logger.debug(f"Connection state: {state}")
            
            if state == "connecting":
                self.connection_attempting = True
                self.connection_attempt_event.set()
                await self.manager.update_state(f"{self.type}_state", "connecting_peer")
            elif state == "connected":
                self.connection_established = True
                self.connection_established_event.set()
                await self.manager.update_state(f"{self.type}_state", "connected")
                self.logger.debug("Contact made.")
            elif state in ("failed", "disconnected", "closed"):
                if not self.disconnected:
                    self.disconnected = True
                    self.disconnect_event.set()
                    await self.manager.update_state(f"{self.type}_state", state)
                    
                    await self.manager.handle_connection_disconnect(self.type, state)
                    
                    if state == "failed":
                        self.logger.warning(f"❌ {self.type.title()} connection failed")
                    else:
                        self.logger.info(f"{self.type.title()} connection {state}")

        @self.pc.on("icegatheringstatechange")
        async def on_ice_gathering_change():
            state = self.pc.iceGatheringState
            self.logger.debug(f"ICE gathering state: {state}")
            
            if state == "complete":
                self.ice_gathering_complete = True
                self.ice_gathering_event.set()
                if self.type == "video":
                    await self.manager.update_state(f"{self.type}_state", "gathering_complete")

        # Data channel handler (only for control connections)
        if self.config.needs_data_channel:
            @self.pc.on("datachannel")
            def on_datachannel(channel):
                if channel.label == self.type:
                    self.data_channel = channel
                    self.logger.debug(f"Data channel received: {channel.label}")
                    self._setup_data_channel_handlers()
                    
                    # CRITICAL: Check if already open (race condition fix)
                    if channel.readyState == "open":
                        self.logger.debug("✅ Data channel was already open")
                        self.data_channels_ready = True
                        self.data_channels_event.set()

    def _setup_data_channel_handlers(self):
        """Setup data channel handlers"""
        @self.data_channel.on("open")
        def on_open():
            self.logger.debug("✅ Data channel opened")
            self.data_channels_ready = True
            self.data_channels_event.set()
        
        @self.data_channel.on("message")
        def on_message(message):
            if self.message_queue:
                asyncio.create_task(self.message_queue.put(json.loads(message)))

    async def _handle_signaling(self):
        try:
            async for message in self.ws:
                data = json.loads(message)
                await self._process_signaling_message(data)
        except websockets.exceptions.ConnectionClosed:
            self.logger.debug("Signaling WebSocket connection closed")
        except asyncio.CancelledError:
            self.logger.debug("Signaling handler cancelled")
        except Exception as e:
            self.logger.debug(f"Signaling handler error: {e}")

    async def _process_signaling_message(self, data: dict):
        """Process WebRTC signaling messages"""
        msg_type = data.get("type")
        self.logger.debug(f"Signaling message: {msg_type}")
        
        if msg_type == "connected":
            self.logger.debug("Signaling connected")
            
        elif msg_type == "start-webrtc":
            if self.manager.role == "lead":
                await self._create_offer()
                
        elif msg_type == "offer":
            await self._handle_offer(data)
            
        elif msg_type == "answer":
            await self._handle_answer(data)
            
        elif msg_type == "ice-candidate":
            await self._handle_ice_candidate(data)

    async def _create_offer(self):
        """Create and send WebRTC offer"""
        self.logger.debug("Creating WebRTC offer...")
        
        # Add transceivers if needed (video-specific behavior)
        if self.config.needs_video_track:
            self.pc.addTransceiver("video", direction=self.config.video_direction)
        
        # Create offer
        offer = await self.pc.createOffer()
        await self.pc.setLocalDescription(offer)
        
        # Set negotiation event
        self.negotiation_complete = True
        self.negotiation_event.set()
        await self.manager.update_state(f"{self.type}_state", "negotiating")
        
        # Wait for ICE gathering
        while self.pc.iceGatheringState != "complete":
            await asyncio.sleep(0.05)
        
        await self.manager.update_state(f"{self.type}_state", "gathering")
        
        # Send offer
        await self.ws.send(json.dumps({
            "type": "offer",
            "offer": {
                "type": self.pc.localDescription.type,
                "sdp": self.pc.localDescription.sdp
            }
        }))

    async def _handle_offer(self, data: dict):
        """Handle received WebRTC offer"""
        self.logger.debug("✅ Handling WebRTC offer...")
        
        # Set remote description
        offer = RTCSessionDescription(data["offer"]["sdp"], data["offer"]["type"])
        await self.pc.setRemoteDescription(offer)
        
        # Video-specific: Add video track if this is a robot video connection
        if (self.config.needs_video_track and 
            self.config.video_direction == "sendonly" and 
            self.manager.config.poste == "robot"):
            
            video_track = self.manager._create_robot_video_track()
            if video_track:
                await self._attach_video_track(video_track)
        
        # Create answer
        answer = await self.pc.createAnswer()
        await self.pc.setLocalDescription(answer)
        
        # Set negotiation event
        self.negotiation_complete = True
        self.negotiation_event.set()
        await self.manager.update_state(f"{self.type}_state", "negotiating")
        
        # Wait for ICE gathering
        while self.pc.iceGatheringState != "complete":
            await asyncio.sleep(0.1)
        
        await self.manager.update_state(f"{self.type}_state", "gathering")
        
        # Send answer
        await self.ws.send(json.dumps({
            "type": "answer",
            "answer": {
                "type": self.pc.localDescription.type,
                "sdp": self.pc.localDescription.sdp
            }
        }))

    async def _attach_video_track(self, video_track):
        """Attach video track to connection"""
        # Find video transceiver and attach track
        attached = False
        for transceiver in self.pc.getTransceivers():
            if transceiver.kind == "video":
                try:
                    transceiver.sender.replaceTrack(video_track)
                    transceiver.direction = "sendonly"
                    attached = True
                    self.logger.debug("Attached video track to existing transceiver")
                    break
                except Exception as e:
                    self.logger.warning(f"Failed to attach track to transceiver: {e}")
                    continue
        
        if not attached:
            try:
                new_transceiver = self.pc.addTransceiver("video", direction="sendonly")
                new_transceiver.sender.replaceTrack(video_track)
                self.logger.debug("Added new video transceiver with track")
            except Exception as e:
                self.logger.exception("Failed to add new video transceiver")

    async def _handle_answer(self, data: dict):
        """Handle received WebRTC answer"""
        self.logger.debug("✅ Handling WebRTC answer...")
        answer = RTCSessionDescription(data["answer"]["sdp"], data["answer"]["type"])
        await self.pc.setRemoteDescription(answer)
        
        # Set negotiation event
        self.negotiation_complete = True
        self.negotiation_event.set()

    async def _handle_ice_candidate(self, data: dict):
        """Handle received ICE candidate"""
        cand = data.get("candidate")
        if cand is None:
            return
        
        from aiortc.sdp import candidate_from_sdp
        sdp_str = cand["candidate"]
        if sdp_str.startswith("candidate:"):
            sdp_str = sdp_str[10:]
        ice = candidate_from_sdp(sdp_str)
        ice.sdpMid = cand.get("sdpMid")
        ice.sdpMLineIndex = cand.get("sdpMLineIndex")
        
        if self.pc.remoteDescription:
            await self.pc.addIceCandidate(ice)

    # Event waiting methods
    async def wait_for_negotiation(self, timeout: float = 30.0):
        await asyncio.wait_for(self.negotiation_event.wait(), timeout=timeout)
    
    async def wait_for_ice_gathering(self, timeout: float = 30.0):
        await asyncio.wait_for(self.ice_gathering_event.wait(), timeout=timeout)
    
    async def wait_for_connection_attempt(self, timeout: float = 30.0):
        await asyncio.wait_for(self.connection_attempt_event.wait(), timeout=timeout)
    
    async def wait_for_connection_established(self, timeout: float = 30.0):
        await asyncio.wait_for(self.connection_established_event.wait(), timeout=timeout)
    
    async def wait_for_data_channels(self, timeout: float = 30.0):
        await asyncio.wait_for(self.data_channels_event.wait(), timeout=timeout)

    async def wait_for_disconnect(self, timeout: Optional[float] = None):
        if timeout:
            await asyncio.wait_for(self.disconnect_event.wait(), timeout=timeout)
        else:
            await self.disconnect_event.wait()

    # Communication methods
    async def send_message(self, message: dict):
        """Send message through data channel"""
        if self.data_channel and self.data_channel.readyState == "open":
            self.data_channel.send(json.dumps(message))
    
    async def receive_message(self) -> Optional[dict]:
        """Receive message from data channel"""
        if self.message_queue:
            return await self.message_queue.get()
        return None

    async def close(self):
        """Close the connection"""
        try:
            if self.ws and not self.ws.closed:
                await asyncio.wait_for(self.ws.close(), timeout=5.0)
        except Exception:
            pass 
        
        try:
            if self.pc and self.pc.connectionState != "closed":
                await asyncio.wait_for(self.pc.close(), timeout=5.0)
        except Exception:
            pass  

class RendezMaitre:
    """WebRTC manager with browser-signaled video connection"""

    def __init__(self, gateway, config):
        self.gateway = gateway
        self.config = config
        self.credentials = None
        self.role = None
        self.conductor = None
        self.connection_started = False
        self.logger = logging.getLogger(f"{__name__}.RendezMaitre")
        
        self.session_ended = False
        self.session_end_event = asyncio.Event()

        # Connections
        self.control_connection: Optional[RendezLien] = None
        self.video_connection: Optional[RendezLien] = None
        
        # Browser video coordination
        self.video_polling_active = False
        self.video_status_task = None

    async def _autonomous_connection_manager(self):
        """Autonomous WebRTC connection manager - control first, video on browser signal"""
        try:
            self.logger.debug("Starting WebRTC connection establishment...")
            
            # 1. ALWAYS establish control connection first
            await self._establish_control_connection()
            
            # 2. For robots: start monitoring for browser video readiness
            if self.config.poste == "robot":
                self.logger.debug("Control established - monitoring for browser video readiness...")
                self.video_status_task = asyncio.create_task(self._monitor_video_session_status())
            
            # 3. Report control ready immediately (session can start)
            await self._report_control_ready()
            
        except Exception as e:
            self.logger.exception("❌ WebRTC connection establishment failed")
            await self.update_state("control_state", "failed")
            if self.video_status_task:
                self.video_status_task.cancel()

    async def _establish_control_connection(self):
        """Establish control connection - this must succeed for session to work"""
        control_config = RendezProtocole(
            connection_type="control",
            credentials=self.credentials["control"],
            needs_data_channel=True,
            needs_video_track=False
        )
        self.control_connection = RendezLien(control_config, self)
        
        # Setup control connection
        await self.control_connection.setup()
        
        # Wait for all control phases with standard timeouts
        self.logger.debug("Waiting for control connection negotiation...")
        await self.control_connection.wait_for_negotiation(timeout=30)
        
        self.logger.debug("Waiting for ICE gathering...")
        await self.control_connection.wait_for_ice_gathering(timeout=30)
        
        self.logger.debug("Waiting for connection establishment...")
        await self.control_connection.wait_for_connection_established(timeout=30)
        
        self.logger.debug("Waiting for data channels...")
        await self.control_connection.wait_for_data_channels(timeout=30)
        
        await self.update_state("control_state", "established")
        self.logger.debug("✅ Control connection established!")

    def _create_robot_video_track(self):
        """Create video track from robot camera"""
        video_logger = logging.getLogger(f"{__name__}.RendezMaitre.video_track")
        
        try:
            if not self.conductor or not self.conductor.robot:
                video_logger.warning("No conductor or robot available for video track")
                return None

            robot = self.conductor.robot
            if not hasattr(robot, "cameras") or not robot.cameras:
                video_logger.warning("Robot has no cameras configured")
                return None

            # Use the robot's camera (typically named "front" or similar)
            camera = robot.cameras.get("front")
            camera_name = "front"
            
            if not camera:
                # Try to get any available camera
                camera_names = list(robot.cameras.keys())
                if camera_names:
                    camera_name = camera_names[0]
                    camera = robot.cameras[camera_name]
                    video_logger.debug(f"Using camera: {camera_name}")
                else:
                    video_logger.warning("No cameras available in robot")
                    return None
            else:
                video_logger.debug(f"Using front camera")

            video_logger.debug(f"Creating video track from {type(camera).__name__}")
            return LeCamera(camera, camera_name)
            
        except Exception as e:
            video_logger.exception("Error creating robot video track")
            return None

    async def _monitor_video_session_status(self):
        """Poll the gateway until the browser signals it’s ready for video."""
        video_logger = logging.getLogger(f"{__name__}.RendezMaitre.video_poll")
        poll_count = 0
        self.video_polling_active = True
        try:
            while self.video_polling_active:
                poll_count += 1
                try:
                    response = await self.gateway.pulse("active")
                    video_status = response.get("video_session_status")
                    
                    video_logger.debug(f"RAW active response: {response!r}")
                    video_status = response.get("video_session_status")
                    
                    if video_status and not self.video_connection:
                        await self._initiate_video_connection()
                    elif poll_count % 30 == 0:
                        video_logger.debug(f"Patently waiting for browser to connect... ({poll_count}s)")
                except Exception as e:
                    video_logger.warning(f"Error polling video status: {e}")
                await asyncio.sleep(1.0)
        except asyncio.CancelledError:
            video_logger.debug("Video status polling cancelled")

    async def _initiate_video_connection(self):
        """Initiate video connection once browser is ready"""
        video_logger = logging.getLogger(f"{__name__}.RendezMaitre.video_init")
        try:
            video_logger.debug("Initiating video connection with browser...")
            
            video_config = RendezProtocole(
                connection_type="video",
                credentials=self.credentials["video"],
                needs_data_channel=False,
                needs_video_track=True,
                video_direction="sendonly"
            )
            self.video_connection = RendezLien(video_config, self)
            await self.video_connection.setup()
            
            video_logger.debug("Waiting for video connection establishment...")
            await self.video_connection.wait_for_connection_established(timeout=45)
            
            await self.update_state("video_state", "established")
            video_logger.info("Video camera is transmitting live")

        except asyncio.TimeoutError:
            video_logger.warning("❌ Video connection timed out")
            await self.update_state("video_state", "timeout")
        except Exception as e:
            video_logger.exception("❌ Video connection failed")
            await self.update_state("video_state", "failed")

    async def _report_control_ready(self):
        webrtc_stats = {
            "control_ready": True,
            "video_ready": False,
            "video_status": "awaiting_browser",
            "timestamp": time.time()
        }
        await self.gateway.pulse("webrtc_established", {"webrtc_stats": webrtc_stats})
        self.logger.info("Teleoperator has control")

    async def execute(self, context: dict) -> PhaseResult:
        # Initial connection attempt
        if not self.connection_started:
            await self._start_connection_process(context)
            self.connection_started = True
            return PhaseResult(context=context)
        
        # Handle timeout errors and retry
        if context.get("webrtc_error") and "timeout" in str(context.get("webrtc_error", "")).lower():
            # Clear the error and retry
            context.pop("webrtc_error", None)
            self.logger.info("🔄 Retrying WebRTC credential request after timeout...")
            await self._start_connection_process(context)
            return PhaseResult(context=context)
        
        # Handle other errors (don't retry these)
        if context.get("webrtc_error"):
            self.logger.error(f"❌ WebRTC error (not retrying): {context['webrtc_error']}")
            return PhaseResult(context=context)  # Stay in webrtc phase, don't advance
        
        # Once control is up, we can move on
        if self.is_control_ready() and not context.get("webrtc_established"):
            context["webrtc_established"] = True
            return PhaseResult(context=context, next_phase="session", complete=True)
        
        return PhaseResult(context=context)

    async def update_state(self, state_type: str, state_value: str):
        """Update state with connection-aware error handling"""
        try:
            await self.gateway.pulse("update_instance", {"type": state_type, "state": state_value})
        except Exception as e:
            self.logger.debug(f"Gateway pulse failed (expected during disconnect): {e}")

    async def _start_connection_process(self, context: dict):
        self.logger.debug("✅ Requesting WebRTC credentials from gateway...")
        try:
            response = await self.gateway.pulse("signaling")
            
            # DEBUG: Log the credential structure we received
            self.logger.debug(f"Signaling response keys: {list(response.keys())}")
            
            if "control_webrtc" in response:
                self.logger.debug(f"Control WebRTC keys: {list(response['control_webrtc'].keys())}")
            if "video_webrtc" in response:
                self.logger.debug(f"Video WebRTC keys: {list(response['video_webrtc'].keys())}")
            
            if "control_webrtc" in response and "video_webrtc" in response:
                self.credentials = {
                    "control": response["control_webrtc"],
                    "video": response["video_webrtc"]
                }
                self.role = context.get("role")
                self.conductor = context.get("conductor")
                self.logger.debug("✅ WebRTC credentials received, starting connections...")
                asyncio.create_task(self._autonomous_connection_manager())
                context["webrtc_connecting"] = True
                # Clear any previous errors
                context.pop("webrtc_error", None)
            elif response.get("timeout"):
                # Flag timeout for retry by execute()
                context["webrtc_error"] = "credential_request_timeout"
                self.logger.warning("⏱️ WebRTC credential request timed out, will retry...")
            else:
                context["webrtc_error"] = "No credentials received"
                self.logger.error(f"❌ No WebRTC credentials received from gateway. Response: {response}")
        except Exception as e:
            self.logger.exception("Failed to start WebRTC connection process")
            context["webrtc_error"] = str(e)
            
    def is_control_ready(self) -> bool:
        return self.control_connection and self.control_connection.data_channels_ready

    async def send_control(self, action: dict):
        if self.control_connection:
            await self.control_connection.send_message(action)

    async def receive_control(self) -> Optional[dict]:
        if self.control_connection:
            return await self.control_connection.receive_message()
        return None

    async def handle_connection_disconnect(self, connection_type: str, disconnect_reason: str):
        """Handle disconnection - set flags for SessionManager to detect"""
        disconnect_logger = logging.getLogger(f"{__name__}.RendezMaitre.disconnect")
        
        if self.session_ended:
            disconnect_logger.debug(f"{connection_type} disconnect ignored - session already ended")
            return
            
        # For control connections, always end session (control is critical)
        if connection_type == "control":
            disconnect_logger.info(f"🔌 Control connection {disconnect_reason}")
            await self._signal_session_end("control_disconnect", f"Control connection {disconnect_reason}")
            
        # For video connections, only end session if it's a hard failure
        elif connection_type == "video" and disconnect_reason == "failed":
            disconnect_logger.info(f"📹 Video connection failed")
            await self._signal_session_end("video_failure", f"Video connection {disconnect_reason}")
        else:
            disconnect_logger.debug(f"📹 Video connection {disconnect_reason} - session continues")

    async def _signal_session_end(self, reason: str, details: str = ""):
        """Signal that session should end - sets flags for SessionManager to detect"""
        if self.session_ended:
            return
            
        self.session_ended = True
        self.session_end_event.set()
        
        self.session_end_reason = reason
        self.session_end_details = details
        
        # Stop video polling if active
        self.video_polling_active = False
        if self.video_status_task:
            self.video_status_task.cancel()
        
        # SessionManager will detect this flag and bubble up to orchestrator
        self.logger.debug(f"Session end signaled: {reason}")

    async def close(self):
        self.video_polling_active = False
        if self.video_status_task:
            self.video_status_task.cancel()
        if self.control_connection:
            await self.control_connection.close()
        if self.video_connection:
            await self.video_connection.close()
            
# Update the session handler to handle the control-only initial state
class MaitreDuPulse:
    """Central flow controller and decision maker"""

    def __init__(self, config: LeConfig):
        self.config = config
        self.gateway = None
        self.vault = None
        self.logger = logging.getLogger(f"{__name__}.MaitreDuPulse")
        
        # Managers
        self.registration_manager = None
        self.match_manager = None
        self.webrtc_manager = None
        self.session_manager = None
        
        # Hardware
        self.robot = None
        self.teleop = None
        
        # State
        self.current_phase = "registration"
        self.context = {
            "poste": config.poste,
            "daemon_version": DAEMON_VERSION,
            "phase": "registration"
        }

    async def run(self):
        """Main orchestration loop"""
        self.logger.info(f"Initializing LeRobo-Vous pulse daemon v{DAEMON_VERSION}")
        self.logger.debug(f"Poste: {self.config.poste}")
        
        try:
            # Setup
            await self._setup()
            
            # Start concurrent tasks
            pulse_task = asyncio.create_task(self._orchestration_loop())
            session_task = asyncio.create_task(self._session_handler())
            
            # Wait for either to complete
            done, pending = await asyncio.wait(
                [pulse_task, session_task], 
                return_when=asyncio.FIRST_COMPLETED
            )
            
            # Cancel remaining tasks
            for task in pending:
                task.cancel()
                    
        except KeyboardInterrupt:
            self.logger.info("Shutdown requested by user")
        except Exception as e:
            self.logger.exception("Unexpected error in main run loop")
        finally:
            await self._cleanup()

    async def _setup(self):
        """Setup gateway, managers, and hardware"""
        self.logger.debug("Setting up daemon components...")
        
        # Transport layer
        self.gateway = LeGatewayPortier(self.config)
        await self.gateway.__aenter__()
        
        # Vault
        self.vault = LeVault(self.config)
        
        # Managers
        self.registration_manager = RegistrationManager(self.gateway, self.vault, self.config)
        self.match_manager = MatchManager(self.gateway, self.config)
        self.webrtc_manager = RendezMaitre(self.gateway, self.config)
        self.session_manager = SessionManager(self.gateway, self.config)
        
        # Hardware
        await self._setup_hardware()
        
        self.context["conductor"] = self
        self.context["webrtc_manager"] = self.webrtc_manager
        
        self.logger.info("Initial setup complete")

    async def _setup_hardware(self):
        """Setup hardware connections"""
        hardware_logger = logging.getLogger(f"{__name__}.MaitreDuPulse.hardware")
        
        if self.config.robot:
            try:
                self.robot = make_robot_from_config(self.config.robot)
                self.robot.connect()
                robot_type = type(self.robot).__name__
                hardware_logger.info(f"Robot ready: {robot_type}")
                
                # Log camera info if available
                if hasattr(self.robot, 'cameras') and self.robot.cameras:
                    camera_names = list(self.robot.cameras.keys())
                    hardware_logger.debug(f"Available cameras: {camera_names}")
                
            except Exception as e:
                hardware_logger.exception("Failed to connect robot")
                raise

        if self.config.teleop:
            try:
                self.teleop = make_teleoperator_from_config(self.config.teleop)
                self.teleop.connect()
                teleop_type = type(self.teleop).__name__
                hardware_logger.info(f"Teleoperator ready: {teleop_type}")
            except Exception as e:
                hardware_logger.exception("Failed to connect teleoperator")
                raise
                
    def _clear_session_context(self, context: dict):
        """Clear session-related data from context"""
        session_keys_to_clear = [
            "session_id", "role", "webrtc_credentials", "session_active",
            "webrtc_established", "webrtc_connecting", "session_end_reason",
            "session_end_details", "disconnect_detected", "remote_session_ended"
        ]
        
        for key in session_keys_to_clear:
            context.pop(key, None)

    def _clear_match_context(self, context: dict):
        """Clear match-related state to ensure fresh start"""
        match_keys_to_clear = [
            "match_found", "ready_sent", "seeking", 
            "webrtc_established", "webrtc_connecting"
        ]
        
        for key in match_keys_to_clear:
            context.pop(key, None)
            
        self.logger.debug("🧹 Match context cleared for fresh seeking")
              
    async def _handle_session_end(self, context: dict):
        """Handle session end - send pulse and clean up context"""
        session_logger = logging.getLogger(f"{__name__}.MaitreDuPulse.session_end")
        
        # Determine session end reason and details
        reason = context.get("session_end_reason", "normal")
        details = context.get("session_end_details", "")
        session_id = context.get("session_id", "unknown")
        
        session_logger.info(f"🔚 Session ending: {reason}")
        if details:
            session_logger.debug(f"Details: {details}")
        
        # Send session_ended pulse to gateway
        try:
            pulse_data = {
                "reason": reason,
                "details": details,
                "session_id": session_id,
                "poste": self.config.poste,
                "timestamp": time.time()
            }
            
            response = await self.gateway.pulse("session_ended", pulse_data)
            
            if response.get("success"):
                session_logger.info("📡 Session end notification sent to gateway")
            else:
                session_logger.warning(f"⚠️ Failed to notify gateway of session end: {response.get('error')}")
                
        except Exception as e:
            # Don't let gateway communication failure block session cleanup
            session_logger.warning(f"⚠️ Error sending session end notification: {e}")
        
        # Clear session-related context
        self._clear_session_context(context)
        
        session_logger.debug("🧹 Session context cleared, returning to match phase")

    async def _orchestration_loop(self):
        """Main orchestration loop with session end handling"""
        self.logger.debug("Starting orchestration loop...")
        
        while True:
            try:
                # Update phase in context
                self.context["phase"] = self.current_phase
                
                # ALWAYS provide fresh conductor reference
                self.context["conductor"] = self
                
                # For webrtc phase, create fresh WebRTC manager
                if self.current_phase == "webrtc" and not self.context.get("webrtc_manager"):
                    # Create fresh WebRTC manager for new session
                    self.webrtc_manager = RendezMaitre(self.gateway, self.config)
                    self.context["webrtc_manager"] = self.webrtc_manager
                
                # Get manager for current phase
                manager = self._get_manager_for_phase(self.current_phase)
                
                # Execute phase
                result = await manager.execute(self.context)
                
                # Update context
                self.context = result.context
                
                # Handle phase transitions
                if result.next_phase:
                    self.logger.debug(f"Phase transition: {self.current_phase} → {result.next_phase}")
                    
                    # Handle session end when leaving session phase
                    if self.current_phase == "session" and result.next_phase == "match":
                        await self._handle_session_end(result.context)
                    
                    # Clean up old WebRTC manager when leaving session
                    if self.current_phase == "session" and result.next_phase == "match":
                        old_webrtc_manager = self.context.get("webrtc_manager")
                        if old_webrtc_manager:
                            try:
                                await asyncio.wait_for(old_webrtc_manager.close(), timeout=2.0)
                            except Exception:
                                pass
                        # Remove from context - will be recreated fresh next time
                        self.context.pop("webrtc_manager", None)
                        
                        self._clear_match_context(self.context)
                    
                    self.current_phase = result.next_phase
                    self.context["phase"] = result.next_phase
                
                await asyncio.sleep(2)
                
            except KeyboardInterrupt:
                if self.current_phase == "session":
                    self.context["session_end_reason"] = "user_interrupted"
                    self.context["session_end_details"] = "User pressed Ctrl+C"
                    await self._handle_session_end(self.context)
                else:
                    if self.current_phase in ["match", "webrtc"]:
                        await self.gateway.pulse("session_ended", {
                            "reason": "user_interrupted",
                            "details": "Daemon shutdown during connection phase"
                        })
                raise
                
            except Exception as e:
                self.logger.exception(f"Error in orchestration loop (phase: {self.current_phase})")
                await asyncio.sleep(1)

    def _get_manager_for_phase(self, phase: str):
        """Get manager for current phase"""
        managers = {
            "registration": self.registration_manager,
            "match": self.match_manager,
            "webrtc": self.webrtc_manager,
            "session": self.session_manager
        }
        
        manager = managers.get(phase)
        if not manager:
            raise Exception(f"Unknown phase: {phase}")
        
        return manager

    async def _cleanup(self):
        """Cleanup resources"""
        cleanup_logger = logging.getLogger(f"{__name__}.MaitreDuPulse.cleanup")
        cleanup_logger.info("Starting cleanup...")
        
        try:
            if self.gateway and hasattr(self.gateway, 'device_secret') and self.gateway.device_secret:
                cleanup_logger.debug("Notifying gateway of disconnection...")
                await asyncio.wait_for(
                    self.gateway.pulse("update_instance", {
                        "field": "status",
                        "state": "disconnected"
                    }),
                    timeout=5.0
                )
                cleanup_logger.debug("? Gateway notified of disconnection")
        except asyncio.TimeoutError:
            cleanup_logger.warning("?? Gateway disconnect notification timed out")
        except Exception as e:
            cleanup_logger.warning(f"?? Failed to notify gateway of disconnection: {e}")
        
        
        try:
            if self.webrtc_manager:
                await asyncio.wait_for(self.webrtc_manager.close(), timeout=3.0)
                cleanup_logger.debug("✅ WebRTC connections closed")
        except Exception:
            cleanup_logger.debug("WebRTC cleanup timeout/error")
            
        if self.gateway:
            await self.gateway.__aexit__(None, None, None)
            cleanup_logger.debug("Transport layer closed")
            
        if self.robot:
            self.robot.disconnect()
            cleanup_logger.debug("Robot disconnected")

        if self.teleop:
            self.teleop.disconnect()
            cleanup_logger.debug("Teleoperator disconnected")
            
        cleanup_logger.info("✅ Cleanup complete")
            
    async def _session_handler(self):
        """Handle real-time session communication - restart for each session"""
        BUFFER_LIMIT = 500    
        INTERVAL = 0.04
        session_logger = logging.getLogger(f"{__name__}.MaitreDuPulse.session_handler")
        
        session_logger.debug("Starting session handler...")

        while True:  # ? Keep running forever
            try:
                # Wait for session phase
                while self.current_phase != "session":
                    await asyncio.sleep(0.5)
                session_logger.debug("Session phase entered.")

                # Wait for control connection
                while not (self.webrtc_manager and self.webrtc_manager.is_control_ready()):
                    session_logger.debug("Waiting for control connection...")
                    await asyncio.sleep(0.5)

                session_logger.debug("? Control ready - session communication active")
                if self.config.poste == "robot":
                    session_logger.debug("Video will connect when browser is ready")

                # Handle session until phase changes
                while self.current_phase == "session":
                    try:
                        # Teleop side: read & send latest action
                        if (self.config.poste == "teleop" and 
                            self.teleop and 
                            self.webrtc_manager.is_control_ready()):

                            action = self.teleop.get_action()
                            if action and action != getattr(self, '_last_sent_action', None):
                                dc = self.webrtc_manager.control_connection.data_channel
                                if dc.bufferedAmount < BUFFER_LIMIT:
                                    await self.webrtc_manager.send_control({
                                        "action": action,
                                        "timestamp": time.time()
                                    })
                                    self._last_sent_action = action
                                else:
                                    self.logger.warning(f"Data channel buffer full: {dc.bufferedAmount}")

                        # Robot side: drain and apply only the freshest command
                        elif (self.config.poste == "robot" and 
                              self.robot and 
                              self.webrtc_manager.is_control_ready()):
                            
                            latest = None
                            while True:
                                try:
                                    pkt = await asyncio.wait_for(
                                        self.webrtc_manager.receive_control(),
                                        timeout=0.01
                                    )
                                    latest = pkt
                                except asyncio.TimeoutError:
                                    break

                            if latest and "action" in latest:
                                self.robot.send_action(latest["action"])

                        await asyncio.sleep(INTERVAL)
                        
                    except Exception as e:
                        session_logger.exception("Error in session handler")
                        await asyncio.sleep(0.1)

                session_logger.debug("Session handler - waiting for next session")
                
            except asyncio.CancelledError:
                session_logger.debug("Session handler cancelled")
                break
            except Exception as e:
                session_logger.exception("Error in session handler outer loop")
                await asyncio.sleep(1)  # Brief pause before retry
                
class SessionManager:
    """Domain-specific command processor for active sessions"""

    def __init__(self, gateway: LeGatewayPortier, config: LeConfig):
        self.gateway = gateway
        self.config = config
        self.logger = logging.getLogger(f"{__name__}.SessionManager")

    async def execute(self, context: dict) -> PhaseResult:
        """Execute session phase - detect end conditions and signal transitions"""
        
        webrtc_manager = context.get("webrtc_manager")
        
        # Check for session end conditions
        session_end_detected = self._check_session_end_conditions(context, webrtc_manager)
        
        if session_end_detected:
            # Session should end - bubble up to orchestrator
            return PhaseResult(context=context, next_phase="match", complete=True)
        
        # Step 1: Start session if not active
        if not context.get("session_active"):
            result = await self._start_session(context)
            context.update(result)
            return PhaseResult(context=context)  # Continue to heartbeat
        
        # Step 2: Send heartbeat and check for remote session end
        result = await self._session_heartbeat(context)
        context.update(result)
        
        return PhaseResult(context=context)  # Continue session

    def _check_session_end_conditions(self, context: dict, webrtc_manager) -> bool:
        """Check all possible session end conditions and set appropriate context"""
        
        # WebRTC connection ended
        if webrtc_manager and webrtc_manager.session_ended:
            if not context.get("session_end_reason"):
                context["session_end_reason"] = "connection_lost"
                context["session_end_details"] = "WebRTC connection ended"
            return True
        
        # Remote session end (from heartbeat)
        if context.get("remote_session_ended"):
            if not context.get("session_end_reason"):
                context["session_end_reason"] = "remote_initiated"
                context["session_end_details"] = "Session ended by remote party"
            return True
        
        # Connection disconnect detected
        if context.get("disconnect_detected"):
            if not context.get("session_end_reason"):
                context["session_end_reason"] = "connection_lost"
                context["session_end_details"] = "Network connection lost"
            return True
        
        # Multiple heartbeat failures
        if context.get("consecutive_heartbeat_failures", 0) >= 3:
            if not context.get("session_end_reason"):
                context["session_end_reason"] = "heartbeat_failure"
                context["session_end_details"] = "Multiple heartbeat failures"
            return True
        
        return False

    async def _start_session(self, context: dict) -> dict:
        """Start active session"""
        self.logger.debug("Starting active session...")
        
        # First call: webrtc_established to signal session is ready
        webrtc_data = {
            "webrtc_stats": {
                "control_ready": True,
                "video_ready": False,  # Will be updated later when video connects
                "timestamp": time.time()
            }
        }
        
        response = await self.gateway.pulse("webrtc_established", webrtc_data)
        if response.get("success"):
            context["session_active"] = True
            self.logger.info("✅ C'est parti!")
            
            session_id = context.get('session_id')
            self.logger.debug(f"Session ID: {session_id}")

            if self.config.poste == "robot":
                self.logger.info("The robot is being remotely controlled...")
            else:
                url = f"https://brainwavecollective.ai/lrv/teleop?session={session_id}"
                self.logger.info("")
                self.logger.info("=" * 60)
                self.logger.info("")
                self.logger.info("  TELEPRESENCE ESTABLISHED")
                self.logger.info("")
                self.logger.info("  You are now there.")
                self.logger.info("")
                self.logger.info("=" * 60)
                self.logger.info("")
                self.logger.info(f"  {url}")
                self.logger.info("")
                self.logger.info("=" * 60)   
                self.logger.info(f"You have control of the robot...")
        else:
            self.logger.warning("⚠️ Failed to start session")
        
        return context

    async def _session_heartbeat(self, context: dict) -> dict:
        """Send session heartbeat and check for remote session end"""
        session_data = {
            "session_metrics": {
                "timestamp": time.time(),
                "session_id": context.get("session_id"),
                "active": True
            }
        }
        
        try:
            response = await self.gateway.pulse("active", session_data)
            context["heartbeat_sent"] = response.get("success", False)
            
            # Reset heartbeat failure counter on success
            if response.get("success"):
                context["consecutive_heartbeat_failures"] = 0
            
            # Check if the gateway indicates the session should end
            session_status = response.get("session_status")
            if session_status in ("ended", "terminated"):
                context["remote_session_ended"] = True
                self.logger.info("🔚 Remote session end detected from heartbeat")
            
        except Exception as e:
            self.logger.warning(f"⚠️ Heartbeat failed: {e}")
            # Track consecutive failures
            heartbeat_failures = context.get("consecutive_heartbeat_failures", 0) + 1
            context["consecutive_heartbeat_failures"] = heartbeat_failures
            
            if heartbeat_failures >= 3:
                context["disconnect_detected"] = True
                self.logger.warning("🔌 Multiple heartbeat failures detected")
        
        return context

# ===== CLI INTERFACE =====
def parse_args():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(description="LeRobo-Vous Daemon")

    parser.add_argument("--poste", choices=["teleop", "robot"], required=True)
    parser.add_argument("--mode", default="solo", help="Availability mode (optional, default: solo)")
    parser.add_argument("--robot.type", help="Robot type")
    parser.add_argument("--robot.port", help="Robot port")
    parser.add_argument("--robot.id", help="Robot ID")
    parser.add_argument("--robot.cameras", help="Robot cameras as JSON string")
    parser.add_argument("--teleop.type", help="Teleoperator type")
    parser.add_argument("--teleop.port", help="Teleoperator port")
    parser.add_argument("--teleop.id", dest="teleop_id", help="Teleoperator ID", default=None)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--reset-secret", action="store_true", help="Clear saved device secret")
    parser.add_argument("--log-level", choices=["DEBUG", "INFO", "WARNING", "ERROR"], 
                        default="INFO", help="Set logging level")

    return parser.parse_args()

def load_camera_map_from_json(json_str, fps=30):
    """Load camera configuration from JSON string"""
    from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig, ColorMode, Cv2Rotation

    parsed = json.loads(json_str)
    result = {}
    for name, cfg in parsed.items():
        rotation = Cv2Rotation[cfg.get("rotation", "NO_ROTATION")]
        config = OpenCVCameraConfig(
            index_or_path=cfg.get("index_or_path", 0),
            color_mode=ColorMode(cfg.get("color_mode", "bgr")),
            warmup_s=cfg.get("warmup_s", 1.0),
            rotation=rotation,
        )
        config.width = cfg.get("width", 640)
        config.height = cfg.get("height", 480)
        config.fps = cfg.get("fps", fps)
        result[name] = config
    return result

def create_config_from_args(args) -> LeConfig:
    """Create LeConfig from parsed args"""
    robot_config = None
    teleop_config = None
    config_logger = logging.getLogger(f"{__name__}.create_config")

    if args.poste == "robot":
        if getattr(args, 'robot_type', None) == 'so101_follower':
            from lerobot.robots.so101_follower import SO101FollowerConfig
            robot_config = SO101FollowerConfig(
                port=getattr(args, 'robot_port', None),
                id=getattr(args, 'robot_id', None),
            )
            
            if hasattr(args, 'robot_cameras') and args.robot_cameras:
                config_logger.debug(f"Configuring cameras from JSON: {args.robot_cameras}")
                robot_config.cameras = load_camera_map_from_json(args.robot_cameras, fps=args.fps)
                config_logger.debug(f"Configured cameras: {list(robot_config.cameras.keys())}")

    elif args.poste == "teleop":
        if getattr(args, 'teleop_type', None) == 'so101_leader':
            from lerobot.teleoperators.so101_leader import SO101LeaderConfig
            teleop_config = SO101LeaderConfig(
                port=getattr(args, 'teleop_port', None),
                id=args.teleop_id,
            )

    return LeConfig(
        poste=args.poste,
        robot=robot_config,
        teleop=teleop_config,
        fps=args.fps,
        mode=args.mode
    )
    
async def main(args):
    """Main entry point for both `lrvd` and direct execution"""
    # basic LeRobot logging
    init_logging()
    # our custom progress‐style logging
    setup_custom_logging(args.log_level)

    if args.reset_secret:
        vault = LeVault(LeConfig(poste="unknown"))
        vault.clear_device_secret()
        logging.getLogger(__name__).info("✅ Device secret cleared")
        return

    config = create_config_from_args(args)
    conductor = MaitreDuPulse(config)
    await conductor.run()


def run_daemon(args):
    """Entry point used by the `lrvd` wrapper."""
    # same logging setup as `main`
    init_logging()
    setup_custom_logging(args.log_level)

    if args.reset_secret:
        vault = LeVault(LeConfig(poste="unknown"))
        vault.clear_device_secret()
        logging.getLogger(__name__).info("✅ Device secret cleared")
        return

    # hand off to our async main, passing the parsed args
    asyncio.run(main(args))


if __name__ == "__main__":
    # parse once, then dispatch to run_daemon
    run_daemon(parse_args())
