"""
MuJoCo Arm Simulator with DualSense Controller Support
Real-time simulation of SO arm controlled by PlayStation DualSense controller
"""

import mujoco
import mujoco.viewer
import numpy as np
import time
from pathlib import Path
from typing import Optional

try:
    from .controller import DualSenseController, ControllerMapper
    from .recorder import MotionRecorder
except ImportError:
    from controller import DualSenseController, ControllerMapper
    from recorder import MotionRecorder


class ArmSimulator:
    """Main simulator class for SO arm in MuJoCo"""
    
    def __init__(self, model_path: str = "mujoco_model/so_arm.xml", 
                 use_controller: bool = True):
        """
        Initialize arm simulator
        
        Args:
            model_path: Path to MuJoCo model XML
            use_controller: Whether to enable DualSense controller input
        """
        self.model_path = Path(model_path)
        
        # Load MuJoCo model
        if not self.model_path.exists():
            raise FileNotFoundError(f"Model file not found: {self.model_path}")
        
        self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        self.data = mujoco.MjData(self.model)

        # Track only arm joints that are driven by actuators.
        self.actuated_joint_ids = self.model.actuator_trnid[:, 0].astype(int)
        self.actuated_qpos_adr = self.model.jnt_qposadr[self.actuated_joint_ids].astype(int)
        self.actuated_qvel_adr = self.model.jnt_dofadr[self.actuated_joint_ids].astype(int)

        # Position actuator targets: integrate joystick deltas so neutral stick holds pose.
        self.target_ctrl = np.zeros(self.model.nu)
        self.command_scale = np.array([1.8, 1.8, 2.0, 2.0, 2.5, 1.5], dtype=float)
        self.command_scale = self.command_scale[:self.model.nu]
        self._button_latch = {'triangle': False, 'cross': False}
        
        # Setup controller
        self.use_controller = use_controller
        self.controller: Optional[DualSenseController] = None
        self.controller_mapper: Optional[ControllerMapper] = None
        
        if self.use_controller:
            self.controller = DualSenseController()
            self.controller_mapper = ControllerMapper(num_joints=self.model.nu)
        
        # Setup recording
        self.recorder = MotionRecorder()
        self.recording_enabled = False
        
        # Simulation state
        self.sim_time = 0.0
        self.running = True
        self.pause = False
        self.record_mode = False
        
        # UI state
        self.show_info = True
        self.show_help = False
        
        print("Simulator initialized successfully")
        print(f"Number of joints: {self.model.nq}")
        print(f"Number of actuators: {self.model.nu}")

        self._sync_targets_from_state()

    def _sync_targets_from_state(self):
        """Initialize or resync actuator targets from current joint positions."""
        for i in range(self.model.nu):
            jnt_id = int(self.model.actuator_trnid[i, 0])
            qpos_adr = int(self.model.jnt_qposadr[jnt_id])
            self.target_ctrl[i] = self.data.qpos[qpos_adr]

        ctrl_limited = self.model.actuator_ctrllimited.astype(bool)
        self.target_ctrl[ctrl_limited] = np.clip(
            self.target_ctrl[ctrl_limited],
            self.model.actuator_ctrlrange[ctrl_limited, 0],
            self.model.actuator_ctrlrange[ctrl_limited, 1],
        )
        self.data.ctrl[:] = self.target_ctrl
    
    def reset(self):
        """Reset simulation to home position"""
        mujoco.mj_resetData(self.model, self.data)
        self.sim_time = 0.0
        self._sync_targets_from_state()
        print("Simulator reset to home position")
    
    def get_joint_states(self) -> tuple[np.ndarray, np.ndarray]:
        """Get controlled arm joint positions and velocities only."""
        positions = self.data.qpos[self.actuated_qpos_adr].copy()
        velocities = self.data.qvel[self.actuated_qvel_adr].copy()
        return positions, velocities
    
    def set_joint_commands(self, commands: np.ndarray):
        """Integrate joystick commands into position-actuator targets."""
        # Clamp commands to valid range
        commands = np.clip(commands, -1.0, 1.0)
        dt = float(self.model.opt.timestep)

        if len(commands) != self.model.nu:
            commands = commands[:self.model.nu]

        # Neutral stick keeps current target; non-zero input moves target smoothly.
        self.target_ctrl += commands * self.command_scale * dt

        ctrl_limited = self.model.actuator_ctrllimited.astype(bool)
        self.target_ctrl[ctrl_limited] = np.clip(
            self.target_ctrl[ctrl_limited],
            self.model.actuator_ctrlrange[ctrl_limited, 0],
            self.model.actuator_ctrlrange[ctrl_limited, 1],
        )
        self.data.ctrl[:] = self.target_ctrl
    
    def step(self, dt: Optional[float] = None):
        """
        Advance simulation by one step
        
        Args:
            dt: Time step (uses model default if None)
        """
        if dt is not None:
            self.model.opt.timestep = dt
        
        mujoco.mj_step(self.model, self.data)
        self.sim_time += self.model.opt.timestep
    
    def update_from_controller(self):
        """Update simulation from DualSense controller input"""
        if not self.controller or not self.controller.connected:
            return False
        
        # Update controller state
        if not self.controller.update():
            return False
        
        # Get controller state
        state = self.controller.get_state_dict()
        
        # Map to arm commands
        commands = self.controller_mapper.map_to_arm_commands(state)
        
        # Handle recording toggle (Triangle button, edge-triggered)
        if state['buttons']['triangle'] and not self._button_latch['triangle']:
            self._toggle_recording()

        # Handle reset (Cross button, edge-triggered)
        if state['buttons']['cross'] and not self._button_latch['cross']:
            self.reset()
        
        self._button_latch['triangle'] = bool(state['buttons']['triangle'])
        self._button_latch['cross'] = bool(state['buttons']['cross'])
        
        # Set joint commands
        self.set_joint_commands(commands)
        
        return True
    
    def _toggle_recording(self):
        """Toggle motion recording"""
        if self.recorder.is_recording:
            self.recorder.stop_recording()
            self.record_mode = False
            print("Recording stopped")
        else:
            metadata = {
                'sim_time': self.sim_time,
                'description': 'DualSense controlled motion'
            }
            self.recorder.start_recording(metadata)
            self.record_mode = True
            print("Recording started - Press Triangle again to stop")
    
    def record_frame(self):
        """Record current frame if recording is enabled"""
        if self.recorder.is_recording:
            positions, velocities = self.get_joint_states()
            commands = self.data.ctrl.copy()
            self.recorder.record_frame(self.sim_time, positions, velocities, commands)
    
    def get_status_string(self) -> str:
        """Get status string for display"""
        status = []
        status.append(f"Time: {self.sim_time:.3f}s")
        
        if self.use_controller:
            status.append(f"Controller: {'Connected' if self.controller.connected else 'Disconnected'}")
        
        status.append(f"Record: {'ON' if self.recorder.is_recording else 'OFF'}")
        status.append(f"Pause: {'YES' if self.pause else 'NO'}")
        
        # Joint angles
        pos, vel = self.get_joint_states()
        if len(pos) >= 6:
            status.append(f"Joints: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}, {pos[3]:.2f}, {pos[4]:.2f}, {pos[5]:.2f}]")
        
        return " | ".join(status)
    
    def run_interactive(self):
        """Run simulator with interactive viewer"""
        print("\n=== SO Arm Simulator ===")
        print("Controls:")
        print("  Left Stick X: Shoulder Pan")
        print("  Left Stick Y: Shoulder Lift")
        print("  Right Stick X: Wrist Roll")
        print("  Right Stick Y: Elbow")
        print("  L2: Wrist Pitch Up")
        print("  R2: Wrist Pitch Down")
        print("  L1: Gripper Close")
        print("  R1: Gripper Open")
        print("  Triangle: Toggle Recording")
        print("  Cross: Reset")
        print("\n")
        
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            # Configure viewer
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = True
            
            prev_time = time.time()
            frame_count = 0
            
            while viewer.is_running() and self.running:
                current_time = time.time()
                elapsed = current_time - prev_time
                prev_time = current_time
                
                # Update from controller
                if self.use_controller:
                    self.update_from_controller()
                
                # Step simulation
                if not self.pause:
                    self.step()
                    self.record_frame()
                
                # Update viewer (don't set label - not supported in newer MuJoCo)
                viewer.sync()
                frame_count += 1
                
                # Print status every second
                if frame_count % 100 == 0:
                    print(self.get_status_string())
        
        print("Simulation ended")
    
    def run_headless(self, duration: float = 60.0):
        """
        Run simulator without viewer (headless mode)
        Useful for training or batch processing
        
        Args:
            duration: Simulation duration in seconds
        """
        print(f"Running headless simulation for {duration}s...")
        
        start_time = time.time()
        last_print = time.time()
        
        while self.sim_time < duration and self.running:
            # Update from controller
            if self.use_controller:
                self.update_from_controller()
            
            # Step simulation
            self.step()
            self.record_frame()
            
            # Print status every second
            if time.time() - last_print > 1.0:
                print(self.get_status_string())
                last_print = time.time()
        
        print("Headless simulation ended")


def main():
    """Main entry point"""
    import argparse
    
    parser = argparse.ArgumentParser(description="SO Arm MuJoCo Simulator")
    parser.add_argument("--headless", action="store_true", help="Run in headless mode (no viewer)")
    parser.add_argument("--duration", type=float, default=60.0, help="Simulation duration in headless mode (seconds)")
    parser.add_argument("--no-controller", action="store_true", help="Disable DualSense controller")
    
    args = parser.parse_args()
    
    # Create simulator
    sim = ArmSimulator(
        model_path="mujoco_model/so_arm.xml",
        use_controller=not args.no_controller
    )
    
    # Run simulation
    if args.headless:
        sim.run_headless(duration=args.duration)
    else:
        sim.run_interactive()


if __name__ == "__main__":
    main()
