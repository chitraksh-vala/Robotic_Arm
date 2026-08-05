"""
DualSense Controller Interface
Handles input from Sony PlayStation DualSense controller
"""

import pygame
import numpy as np
from typing import Dict, Optional


class DualSenseController:
    """
    Interface for DualSense controller input
    Maps analog sticks and triggers to arm control
    """
    
    # DualSense button mapping
    BUTTON_SQUARE = 0
    BUTTON_CROSS = 1
    BUTTON_CIRCLE = 2
    BUTTON_TRIANGLE = 3
    BUTTON_L1 = 4
    BUTTON_L2 = 6  # Analog trigger
    BUTTON_R1 = 5
    BUTTON_R2 = 7  # Analog trigger
    BUTTON_SHARE = 8
    BUTTON_OPTIONS = 9
    BUTTON_L3 = 10
    BUTTON_R3 = 11
    BUTTON_PS = 12
    BUTTON_TOUCHPAD = 13
    
    AXIS_LEFT_STICK_X = 0
    AXIS_LEFT_STICK_Y = 1
    AXIS_RIGHT_STICK_X = 2
    AXIS_RIGHT_STICK_Y = 3
    AXIS_L2_TRIGGER = 4
    AXIS_R2_TRIGGER = 5
    
    def __init__(self, joystick_id: int = 0, deadzone: float = 0.1):
        """
        Initialize DualSense controller
        
        Args:
            joystick_id: Pygame joystick index
            deadzone: Analog stick deadzone threshold (0-1)
        """
        self.joystick_id = joystick_id
        self.deadzone = deadzone
        self.joystick: Optional[pygame.joystick.Joystick] = None
        self.connected = False
        self.button_states = {}
        self.axis_values = {}
        
        self._initialize_joystick()
    
    def _initialize_joystick(self):
        """Initialize pygame joystick"""
        pygame.init()
        pygame.joystick.init()
        
        joystick_count = pygame.joystick.get_count()
        if joystick_count > self.joystick_id:
            self.joystick = pygame.joystick.Joystick(self.joystick_id)
            self.joystick.init()
            self.connected = True
            print(f"DualSense Controller connected: {self.joystick.get_name()}")
        else:
            print(f"No joystick found at index {self.joystick_id}")
            self.connected = False
    
    def _apply_deadzone(self, value: float) -> float:
        """Apply deadzone to analog stick values"""
        if abs(value) < self.deadzone:
            return 0.0
        return value
    
    def update(self) -> bool:
        """
        Update controller state from pygame events
        Must be called regularly in main loop
        
        Returns:
            True if controller is still connected
        """
        if not self.connected:
            return False
        
        pygame.event.pump()
        
        # Update axis values (analog sticks and triggers)
        if self.joystick:
            axes_count = self.joystick.get_numaxes()
            for i in range(axes_count):
                value = self.joystick.get_axis(i)
                # Apply deadzone to stick axes only (not triggers)
                if i < 4:  # Left and right stick axes
                    value = self._apply_deadzone(value)
                self.axis_values[i] = value
            
            # Update button states
            buttons_count = self.joystick.get_numbuttons()
            for i in range(buttons_count):
                self.button_states[i] = self.joystick.get_button(i)
        
        return self.connected
    
    def get_left_stick(self) -> tuple[float, float]:
        """Get normalized left analog stick position (-1 to 1)"""
        x = self.axis_values.get(self.AXIS_LEFT_STICK_X, 0.0)
        y = -self.axis_values.get(self.AXIS_LEFT_STICK_Y, 0.0)  # Invert Y
        return (x, y)
    
    def get_right_stick(self) -> tuple[float, float]:
        """Get normalized right analog stick position (-1 to 1)"""
        x = self.axis_values.get(self.AXIS_RIGHT_STICK_X, 0.0)
        y = -self.axis_values.get(self.AXIS_RIGHT_STICK_Y, 0.0)  # Invert Y
        return (x, y)
    
    def get_l2_trigger(self) -> float:
        """Get L2 trigger value (0 to 1)"""
        # Convert from -1 to 1 to 0 to 1
        value = self.axis_values.get(self.AXIS_L2_TRIGGER, -1.0)
        return (value + 1.0) / 2.0
    
    def get_r2_trigger(self) -> float:
        """Get R2 trigger value (0 to 1)"""
        # Convert from -1 to 1 to 0 to 1
        value = self.axis_values.get(self.AXIS_R2_TRIGGER, -1.0)
        return (value + 1.0) / 2.0
    
    def is_button_pressed(self, button: int) -> bool:
        """Check if a button is pressed"""
        return self.button_states.get(button, False)
    
    def get_state_dict(self) -> Dict:
        """Get complete controller state as dictionary"""
        left_stick = self.get_left_stick()
        right_stick = self.get_right_stick()
        
        return {
            'connected': self.connected,
            'left_stick': left_stick,
            'right_stick': right_stick,
            'l2_trigger': self.get_l2_trigger(),
            'r2_trigger': self.get_r2_trigger(),
            'buttons': {
                'square': self.is_button_pressed(self.BUTTON_SQUARE),
                'cross': self.is_button_pressed(self.BUTTON_CROSS),
                'circle': self.is_button_pressed(self.BUTTON_CIRCLE),
                'triangle': self.is_button_pressed(self.BUTTON_TRIANGLE),
                'l1': self.is_button_pressed(self.BUTTON_L1),
                'r1': self.is_button_pressed(self.BUTTON_R1),
                'l3': self.is_button_pressed(self.BUTTON_L3),
                'r3': self.is_button_pressed(self.BUTTON_R3),
            }
        }


class ControllerMapper:
    """Maps DualSense input to arm joint commands"""
    
    def __init__(self, num_joints: int = 6):
        """
        Initialize controller mapper
        
        Args:
            num_joints: Number of arm joints (6 for the official SO arm model)
        """
        self.num_joints = num_joints
        self.joint_speed = 1.0  # Speed multiplier for joint movements
        self.deadzone = 0.1
    
    def map_to_arm_commands(self, controller_state: Dict) -> np.ndarray:
        """
        Map controller state to arm joint commands
        
        Control scheme (6 DOF SO arm):
        - Left stick X: Shoulder pan
        - Left stick Y: Shoulder lift
        - Right stick Y: Elbow flex
        - L2/R2 triggers: Wrist flex
        - Right stick X: Wrist roll
        - L1: Gripper close
        - R1: Gripper open
        
        Args:
            controller_state: Dictionary from controller.get_state_dict()
        
        Returns:
            Array of joint velocity commands (6 joints)
        """
        commands = np.zeros(self.num_joints)
        
        left_stick = controller_state['left_stick']
        right_stick = controller_state['right_stick']
        buttons = controller_state['buttons']
        l2 = controller_state['l2_trigger']
        r2 = controller_state['r2_trigger']
        
        # Joint 0: Shoulder pan
        commands[0] = left_stick[0] * self.joint_speed

        # Joint 1: Shoulder lift
        commands[1] = left_stick[1] * self.joint_speed

        # Joint 2: Elbow flex
        commands[2] = right_stick[1] * self.joint_speed

        # Joint 3: Wrist flex
        commands[3] = (l2 - r2) * self.joint_speed

        # Joint 4: Wrist roll
        commands[4] = right_stick[0] * self.joint_speed

        # Joint 5: Gripper
        if buttons['l1']:
            commands[5] = 0.5
        elif buttons['r1']:
            commands[5] = -0.5
        
        return commands
