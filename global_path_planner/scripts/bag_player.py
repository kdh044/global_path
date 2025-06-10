#!/usr/bin/env python3

import rospy
import subprocess
import signal

class BagPlayer:
    def __init__(self):
        rospy.init_node('bag_player', anonymous=True)
        rospy.set_param('/use_sim_time', True)
        
        bag_file = rospy.get_param('~bag_file', "/media/danny/DB/JBNU_LIDAR_DATASET_EVAL/2025-01-11-10-57-18.bag")
        play_rate = rospy.get_param('~play_rate', 1.0)
        
        cmd = ["rosbag", "play", "--clock", f"--rate={play_rate}", bag_file]
        
        rospy.loginfo("🎬 Bag 재생 시작")
        self.process = subprocess.Popen(cmd)
        
        signal.signal(signal.SIGINT, lambda s, f: self.process.terminate())
        
    def run(self):
        self.process.wait()
        rospy.loginfo("✅ Bag 재생 완료")

if __name__ == '__main__':
    try:
        player = BagPlayer()
        player.run()
    except:
        pass