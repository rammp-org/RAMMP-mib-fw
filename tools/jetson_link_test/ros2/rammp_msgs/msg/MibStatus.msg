# Mirrors MIB::MibStatus (messages/mib_message.hpp). DDS type name
# rammp_msgs::msg::dds_::MibStatus_, published by the MIB on /rammp/mib/status
# (DDS topic rt/rammp/mib/status). Field order is the wire order; do not reorder.
uint8 system_state        # MIB::MibSystemState: 0 INITIALIZING, 1 IDLE, 2 ENABLED, 3 ERROR
uint8 active_profile      # MIB::DriveProfile: 0 LOW, 1 NORMAL, 2 HIGH
SeatState current_seat_state
string error_message
int64 epoch_s             # Unix time, 0 = clock not known
float32 speed             # m/s
int16 utc_offset_min
uint8 seq                 # +1 per message, wraps
string status_text
string error_footer
