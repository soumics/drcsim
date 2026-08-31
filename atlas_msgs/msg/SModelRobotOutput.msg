# rACT: Action request (activation bit).
# 0x0 - Decativate gripper.
# 0x1 - Activate gripper.

uint8 r_act

# rMOD: Changes the gripper grasping mode.
# 0x0 - Basic mode.
# 0x1 - Pinch mode.
# 0x2 - Wide mode.
# 0x3 - Scissor mode.

uint8 r_mod

# rGTO: "Go To" action moves the gripper fingers to the requested position.
# 0x0 - Stop.
# 0x1 - Go to the requested position.

uint8 r_gto

# rATR: Automatic release routine.
# 0x0 - Stop.
# 0x1 Emergency auto-release.

uint8 r_atr

# rICF: Individual Control of Fingers mode. Each finger receives its own command
# unless the gripper is in the Scissor Grasping Mode and the Independent Control
# of Scissor (rICS) is not activated.
# 0x0 - Normal.
# 0x1 - Enable Individual Control of Fingers A, B and C.

uint8 r_icf

# rICS: Individual Control of Scissor. The scissor axis moves independenly from
# the grasping mode. When this option is selected, the rMOD bits (grasping mode)
# are ignored as the scissor axis position is defined by the rPRS.
# 0x0 - Normal.
# 0x1 - Enable Individual Control of Scissor. Disable Mode Selection.

uint8 r_ics

# rPRA: Target position of the fingers (or finger A only if bit rICF is set).
# 0x00 Minimum position (open).
# 0xFF Maximum position (close).

uint8 r_pra

# rSPA: Gripper closing or opening speed (or finger A only if bit rICF is set).
# Setting a speed will not initiate a motion.
# 0x00 Minimum speed.
# 0xFF Maximum speed.

uint8 r_spa

# rFRA: Final grasping force of the gripper (or finger A only if bit rICF is
# set).
# 0x00 Minimum force.
# 0xFF Maximum force.

uint8 r_fra

# rPRB: Finger B target position. It is only available if bit rICF is set.
# 0x00 Minimum position (open).
# 0xFF Maximum position (close).

uint8 r_prb

# rSPB: Finger B speed. It is only available if bit rICF is set.
# Setting a speed will not initiate a motion.
# 0x00 Minimum speed.
# 0xFF Maximum speed.

uint8 r_spb

# rFRB: Finger B force. It is only available if bit rICF is set.
# 0x00 Minimum force.
# 0xFF Maximum force.

uint8 r_frb

# rPRC: Finger C target position. It is only available if bit rICF is set.
# 0x00 Minimum position (open).
# 0xFF Maximum position (close).

uint8 r_prc

# rSPC: Finger C speed. It is only available if bit rICF is set.
# Setting a speed will not initiate a motion.
# 0x00 Minimum speed.
# 0xFF Maximum speed.

uint8 r_spc

# rFRC: Finger C force. It is only available if bit rICF is set.
# 0x00 Minimum force.
# 0xFF Maximum force.

uint8 r_frc

# rPRS: Scissor axis target position. It is only applied if the Individual
# Control of Scissor option is selected (bit rICS is set).
# 0x00 Minimum position (open).
# 0xFF Maximum position (close).

uint8 r_prs

# rSPS: Scissor axis speed. It is only applied if the Individual
# Control of Scissor option is selected (bit rICS is set).
# 0x00 Minimum speed.
# 0xFF Maximum speed.

uint8 r_sps

# rFRS: Scissor axis force. It is only applied if the Individual
# Control of Scissor option is selected (bit rICS is set).
# 0x00 Minimum force.
# 0xFF Maximum force.

uint8 r_frs
