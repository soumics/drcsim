# gACT : Initialization status, echo of the rACT bit (activation bit).
# 0x0 - Gripper reset.
# 0x1 - Gripper activation.

uint8 g_act

# gMOD : Operation mode status, echo of the rMOD bits (grasping mode requested).
# 0x00 - Basic mode.
# 0x01 - Pinch mode.
# 0x02 - Wide mode.
# 0x03 - Scissor mode.

uint8 g_mod

# gGTO : Action status. echo of the rGTO bit (go to bit).
# 0x0 - Stopped (or performing activation / grasping mode change / automatic release)
# 0x1 - Go to Position Request

uint8 g_gto

# gIMC : Gripper status, returns the current status of the Gripper.
# 0x00 - Gripper is in reset (or automatic release) state. See Fault status if Gripper is activated.
# 0x01 - Activation is in progress.
# 0x02 - Mode change is in progress.
# 0x03 - Activation and mode change are completed.

uint8 g_imc

# gSTA : Motion status, returns the current motion of the Gripper fingers.
# 0x00 - Gripper is in motion towards requested position (only meaningful if gGTO = 1)
# 0x01 - Gripper is stopped. One or two fingers stopped before requested position
# 0x02 - Gripper is stopped. All fingers stopped before requested position
# 0x03 - Gripper is stopped. All fingers reached requested position

uint8 g_sta

# gDTA : Finger A object detection status returns information on possible object contact from finger A.
# 0x00 - Finger A is in motion (only meaningful if gGTO = 1).
# 0x01 - Finger A has stopped due to a contact while opening.
# 0x02 - Finger A has stopped due to a contact while closing.
# 0x03 - Finger A is at requested position.

uint8 g_dta

# gDTB : Finger B object detection status returns information on possible object contact from finger B.
# 0x00 - Finger B is in motion (only meaningful if gGTO = 1).
# 0x01 - Finger B has stopped due to a contact while opening.
# 0x02 - Finger B has stopped due to a contact while closing.
# 0x03 - Finger B is at requested position.

uint8 g_dtb

# gDTC : Finger C object detection status returns information on possible object contact from finger A.
# 0x00 - Finger C is in motion (only meaningful if gGTO = 1).
# 0x01 - Finger C has stopped due to a contact while opening.
# 0x02 - Finger C has stopped due to a contact while closing.
# 0x03 - Finger C is at requested position.

uint8 g_dtc

# gDTS : Scissor object detection status returns information on possible object contact from scissor.
# 0x00 - Scissor is in motion (only meaningful if gGTO = 1).
# 0x01 - Scissor has stopped due to a contact while opening.
# 0x02 - Scissor has stopped due to a contact while closing.
# 0x03 - Scissor is at requested position.

uint8 g_dts

# gFLT : Fault status returns general error messages useful for troubleshooting.
#   0x00 - No fault (fault LED off)
#   Priority faults (fault LED off)
#     0x05 - Action delayed, activation (reactivation) must be completed prior to action.
#     0x06 - Action delayed, mode change must be completed prior to action.
#     0x07 - The activation bit must be set prior to action.
#   Minor faults (fault LED continuous red)
#     0x09 - The communication chip is not ready (may be booting).
#     0x0A - Changing mode fault, interferences detected on Scissor (for less than 20 sec).
#     0x0B - Automatic release in progress.
#   Major faults (fault LED blinking red) - Reset is required
#     0x0D - Activation fault, verify that no interference or other error occurred.
#     0x0E - Changing mode fault, interferences detected on Scissor (for more than 20 sec).
#     0x0F - Automatic release completed. Reset and activation is required.

uint8 g_flt


# gPRA : Echo of the requested position of the Gripper (rPRA),
#   0x00 is minimum position (full opening) and
#   0xFF is maximum position (full closing).
# If commanding the Gripper in
#   individual control mode, gPRA is the echo of finger A,
#   otherwise it is the general position requested to all fingers.

uint8 g_pra

# gPOA : Returns the actual position of the Gripper finger A,
#   0x00 is minimum position (full opening) and
#   0xFF is maximum position (full closing).

uint8 g_poa

# gCUA : Returns a value that represents the finger A instantaneous current consumption from 0x00 to 0xFF.

uint8 g_cua

# gPRB : Echo of the requested position of finger B (rPRB),
#   0x00 is minimum position (full opening) and
#   0xFF is maximum position (full closing).

uint8 g_prb

# gPOB : Returns the actual position of the Gripper finger B,
#   0x00 is minimum position (full opening) and
#   0xFF is maximum position (full closing).

uint8 g_pob

# gCUB : Returns a value that represents the finger B instantaneous current consumption from 0x00 to 0xFF.

uint8 g_cub

# gPRC : Echo of the requested position of finger C (rPRC),
#   0x00 is minimum position (full opening) and
#   0xFF is maximum position (full closing).

uint8 g_prc

# gPOC : Returns the actual position of the Gripper finger C,
#   0x00 is minimum position (full opening) and
#   0xFF is maximum position (full closing).

uint8 g_poc

# gCUC : Returns a value that represents the finger C instantaneous current consumption from 0x00 to 0xFF.

uint8 g_cuc

# gPRS : Echo of the requested position of scissor (rPRS),
#   0x00 is minimum position (full opening) and
#   0xFF is maximum position (full closing).

uint8 g_prs

# gPOS : Returns the actual position of the Gripper scissor,
#   0x00 is minimum position (full opening) and
#   0xFF is maximum position (full closing).

uint8 g_pos

# gCUS : Returns a value that represents the scissor instantaneous current consumption from 0x00 to 0xFF.
#   0x00 is ...
#   0xFF is ...

uint8 g_cus
