# (unix:system (format nil "python /home/affonso/stopper.py ~A &" (unix:getpid)))

import argparse
import os
import rospy
import signal
from jsk_gui_msgs.msg import VoiceMessage

class StopSubscriber(object):
    def __init__(self, pid):
        self.pid = pid
        self.sound_sub = rospy.Subscriber('Tablet/voice', VoiceMessage,
                                          self.callback)
    def callback(self, msg):
        for text in msg.texts:
            if 'stop' in text:
                os.kill(self.pid, signal.SIGINT)
                return
            if 'continue' in text:
                os.kill(self.pid, signal.SIGHUP)
                return

def main():
    p = argparse.ArgumentParser()
    p.add_argument("pid", type=int, help="euslisp process id")
    args = p.parse_args()

    rospy.init_node("stopper",anonymous=True)

    StopInstance = StopSubscriber(args.pid)
    print "Running..."
    rospy.spin()

if __name__ == '__main__':
    main()
