#!/usr/bin/env python3
"""Update controller.py for 6 DOF SO arm model"""
import re

# Read the controller file
with open('src/controller.py', 'r', encoding='utf-8') as f:
    content = f.read()

# Replace docstring comments
content = content.replace(
    '        Control scheme:',
    '        Control scheme (6 DOF SO arm):'
)
content = content.replace(
    '        - Left stick X: Shoulder pan (joint 0)',
    '        - Left stick X: Shoulder pan'
)
content = content.replace(
    '        - Left stick Y: Shoulder lift (joint 1)',
    '        - Left stick Y: Shoulder lift'
)
content = content.replace(
    '        - Right stick X: Wrist roll (joint 4)',
    '        - Right stick X: Wrist roll'
)
content = content.replace(
    '        - Right stick Y: Elbow (joint 2)',
    '        - Right stick Y: Elbow flex'
)
content = content.replace(
    '        - L2 trigger: Wrist pitch up (joint 5)',
    '        - L2 trigger: Wrist flex up'
)
content = content.replace(
    '        - R2 trigger: Wrist pitch down (joint 5)',
    '        - R2 trigger: Wrist flex down'
)
content = content.replace(
    '        - L1: Gripper close (joint 6)',
    '        - L1: Gripper (close)'
)
content = content.replace(
    '        - R1: Gripper open (joint 7)',
    '        - R1: Gripper (open)'
)
content = content.replace(
    '            Array of joint velocity commands',
    '            Array of joint velocity commands (6 DOF)'
)

# Replace command assignments with flexible whitespace handling
content = re.sub(
    r'# Left stick: shoulder pan and lift\s+commands\[0\] = left_stick\[0\] \* self\.joint_speed  # Pan\s+commands\[1\] = left_stick\[1\] \* self\.joint_speed  # Lift',
    '''# Joint 0: Shoulder pan (left stick X)
        commands[0] = left_stick[0] * self.joint_speed
        
        # Joint 1: Shoulder lift (left stick Y)
        commands[1] = left_stick[1] * self.joint_speed''',
    content,
    flags=re.DOTALL
)

content = re.sub(
    r'# Right stick: elbow and wrist roll\s+commands\[2\] = right_stick\[1\] \* self\.joint_speed  # Elbow\s+commands\[4\] = right_stick\[0\] \* self\.joint_speed  # Wrist roll\s+# L2/R2 triggers: wrist pitch\s+commands\[5\] = \(l2 - r2\) \* self\.joint_speed  # Wrist pitch',
    '''# Joint 2: Elbow flex (right stick Y)
        commands[2] = right_stick[1] * self.joint_speed
        
        # Joint 3: Wrist flex (L2/R2 triggers)
        commands[3] = (l2 - r2) * self.joint_speed
        
        # Joint 4: Wrist roll (right stick X)
        commands[4] = right_stick[0] * self.joint_speed''',
    content,
    flags=re.DOTALL
)

content = re.sub(
    r'# Buttons: gripper\s+if buttons\[\'l1\'\]:\s+commands\[6\] = 0\.5  # Gripper left close\s+if buttons\[\'r1\'\]:\s+commands\[7\] = -0\.5  # Gripper right open',
    '''# Joint 5: Gripper (single joint)
        if buttons['l1']:
            commands[5] = 0.5  # Close
        elif buttons['r1']:
            commands[5] = -0.5  # Open''',
    content,
    flags=re.DOTALL
)

# Write back
with open('src/controller.py', 'w', encoding='utf-8') as f:
    f.write(content)

print('✓ controller.py updated successfully')
