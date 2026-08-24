# Read-only snapshot of the constrained live hand-control state machine.
# This message exposes no arbitrary joint target or motor-control field.

uint8 DISABLED=0
uint8 IDLE=1
uint8 MOVING_TO_POSE=6

std_msgs/Header header
uint8 state
bool connected
string detail
