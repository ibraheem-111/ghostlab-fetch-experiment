import os

def send_vel_comand(vel_mag=0.0, ang_vel=0.0):

    # rostopic pub /cmd_vel geometry_msgs/Twist -r 3 '[-0.2, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
    # cmd = f"rostopic pub /cmd_vel geometry_msgs/Twists -r 3 - '[{vel_mag:.2f} 0.0 0.0]' '[0.0 0.0 {ang_vel:.2f}]'"
    cmd = "rostopic pub /cmd_vel geometry_msgs/Twist -r 3 '[{}, 0.0, 0.0]' '[0.0, 0.0, {}]'".format(vel_mag, ang_vel)

    os.system(cmd)    