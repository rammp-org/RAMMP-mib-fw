# Mirrors rammp::XYTwist (messages/joystick_message.hpp). DDS type name
# rammp_msgs::msg::dds_::XYTwist_, published by the joystick / Jetson on
# /rammp/joystick/xy_twist (DDS topic rt/rammp/joystick/xy_twist).
float32 x        # -1..+1, + = right
float32 y        # -1..+1, + = forward
float32 twist    # -1..+1, + = clockwise
uint32 buttons   # rammp::Buttons bit set; 0x1 = joystick button
