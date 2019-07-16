# (unix:system (format nil "python /home/affonso/stopper.py ~A &" (unix:getpid)))

import argparse
import os
import rospy
import signal
from interrupt import InterruptController
from jsk_gui_msgs.msg import VoiceMessage

LOG_LEVELS = {
    "debug": rospy.DEBUG,
    "info": rospy.INFO,
    "warn": rospy.WARN,
    "error": rospy.ERROR,
    "fatal": rospy.FATAL,
}

class StopSubscriber(object):
    def __init__(self, pid, controllers):
        self.pid = pid
        self.sound_sub = rospy.Subscriber('Tablet/voice', VoiceMessage,
                                          self.callback)
        self.controllers = [InterruptController(x) for x in controllers]

    def callback(self, msg):
        for text in msg.texts:
            if 'stop' in text:
                os.kill(self.pid, signal.SIGINT)
                for c in self.controllers:
                    c.interrupt()
                return
            if 'continue' in text:
                for c in self.controllers:
                    c.resume()
                os.kill(self.pid, signal.SIGHUP)
                return

    def spin(self):
        while True:
            try:
                os.kill(self.pid, 0)
            except OSError:
                rospy.logwarning("...Closing")
                exit()

def main():
    p = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument("pid", type=int, help="euslisp process id")
    p.add_argument("--controller", "-c", action='append',
                   help="motor controllers")
    p.add_argument("--log-level", "-l", choices=LOG_LEVELS.keys(),
                   default="debug", help="Log level")
    args = p.parse_args()

    log_level = LOG_LEVELS[args.log_level]
    rospy.init_node("stopper", anonymous=True, log_level=log_level)

    StopInstance = StopSubscriber(args.pid, args.controller)
    rospy.loginfo("Monitoring pid %d..." % args.pid)
    StopInstance.spin()

if __name__ == '__main__':
    main()
