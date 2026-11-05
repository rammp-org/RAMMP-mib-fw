# Sourced by the pixi tasks: the built interfaces, and ROS 2 on Fast DDS pinned to the
# interface that reaches the MIB (see fastdds_mib.xml).
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$here/ws/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
MIB_IP="${MIB_IP:-192.168.4.1}"
# This host's address on the MIB's subnet: the source address of a route to the MIB.
if [ "$(uname)" = Darwin ]; then
  iface="$(route -n get "$MIB_IP" | awk '/interface:/ {print $2}')"
  MIB_HOST_IP="$(ipconfig getifaddr "$iface")"
else
  MIB_HOST_IP="$(ip -4 route get "$MIB_IP" | sed -n 's/.* src \([0-9.]*\).*/\1/p')"
fi
mkdir -p "$here/.pixi"
sed -e "s/MIB_HOST_IP/$MIB_HOST_IP/" -e "s/MIB_IP/$MIB_IP/" "$here/fastdds_mib.xml" \
  > "$here/.pixi/fastdds_mib.xml"
# Fast DDS 2.x (Jazzy) reads the FASTRTPS_ name; 3.x reads FASTDDS_.
export FASTRTPS_DEFAULT_PROFILES_FILE="$here/.pixi/fastdds_mib.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"
echo "ROS 2 on $MIB_HOST_IP ($iface), MIB at $MIB_IP" >&2
