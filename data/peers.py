import rclpy,sys,time
from rclpy.qos import qos_profile_sensor_data
rclpy.init(); n=rclpy.create_node('dhcp_soak_%s'%sys.argv[1])
got=[0]
def cb(m): got[0]+=1
from nav_msgs.msg import Odometry
n.create_subscription(Odometry,'/base/odom',cb,10)
end=time.time()+float(sys.argv[2])
while time.time()<end: rclpy.spin_once(n,timeout_sec=1.0)
print(sys.argv[1],'msgs',got[0])
