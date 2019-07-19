import rospy
from actionlib_msgs.msg import GoalID
from control_msgs.msg import FollowJointTrajectoryAction
from std_msgs.msg import Empty

class ActionCommand(object):
    def __init__(self, goal_msg):
        self.id = goal_msg.goal_id.id
        self.goal = goal_msg.goal
        self.feedback = None

    def update(self, feedback_msg):
        self.feedback = feedback_msg.feedback


class InterruptAction(object):
    def __init__(self, ns, ActionSpec):
        self.active_commands = []
        self.interrupt_commands = []

        if not ns.endswith('/'):
            ns += '/'

        inst = ActionSpec()
        self.goal_type = type(inst.action_goal)

        self.goal_sub = rospy.Subscriber(
            ns + 'goal', type(inst.action_goal), self._goal_cb)
        self.feedback_sub = rospy.Subscriber(
            ns + 'feedback', type(inst.action_feedback), self._feedback_cb)
        self.result_sub = rospy.Subscriber(
            ns + 'result', type(inst.action_result), self._result_cb)
        self.interrupt_sub = rospy.Subscriber(
            ns + 'interrupt', Empty, self.interrupt)
        self.resume_sub = rospy.Subscriber(
            ns + 'resume', Empty, self.resume)
        self.goal_pub = rospy.Publisher(
            ns + 'goal', self.goal_type, queue_size=1)
        self.cancel_pub = rospy.Publisher(
            ns + 'cancel', GoalID, queue_size=100)
        rospy.loginfo("Ready to take orders from {}".format(ns))

    def _goal_cb(self, msg):
        self.active_commands.append(ActionCommand(msg))

    def _feedback_cb(self, msg):
        for comm in self.active_commands:
            if comm.id == msg.status.goal_id.id:
                comm.update(msg)
                break

    def _result_cb(self, msg):
        self.active_commands = [x for x in self.active_commands
                                if x.id != msg.status.goal_id.id]

    def resume(self, msg=None):
        rospy.logdebug("Resume request received")
        try:
            comm = self.interrupt_commands.pop()
            rospy.logdebug("Comm: \n%s" % comm)
            self.interrupt_commands = []
            goal_msg = self.goal_type(goal=self.resume_goal(comm))

            tm = rospy.get_rostime()
            id = str(tm) + '_' + rospy.get_name()
            goal_msg.header.seq = 1
            goal_msg.header.stamp = tm
            goal_msg.goal_id = GoalID(id=id)
            goal_msg.goal.trajectory.header.stamp = tm

            rospy.logdebug("Resumed Action:\n%s" % goal_msg.goal)

            self.goal_pub.publish(goal_msg)
        except IndexError:
            rospy.logwarn('No commands to resume')

    def interrupt(self, msg=None):
        try:
            comm = self.active_commands[-1]
            # self.cancel_pub.publish(GoalID(stamp=rospy.get_rostime(), id=comm.id))
            self.cancel_pub.publish(GoalID())
            self.interrupt_commands.append(comm)
            rospy.logdebug("Interrupted Action:\n%s" % comm.goal)

        except IndexError:
            rospy.logwarn('No commands to interrupt')

    def resume_goal(self, comm):
        """Return goal to be resumed from previous ActionCommand instance."""
        # Overwrite in child classes
        return comm.goal


class InterruptController(InterruptAction):
    def __init__(self, ns):
        super(InterruptController, self).__init__(
            ns, FollowJointTrajectoryAction)

    def resume_goal(self, comm):
        goal = comm.goal

        # Reset time stamp
        goal.trajectory.header.stamp = rospy.Duration(0)

        try:
            tm_offset = comm.feedback.actual.time_from_start
        except AttributeError:
            rospy.loginfo("No feedback received.")
            rospy.loginfo("Resuming original goal...")
            return goal

        rospy.logdebug("Interrupted time: %s" % tm_offset.to_sec())

        # Remove completed steps
        last_point = goal.trajectory.points[-1]
        goal.trajectory.points = [x for x in goal.trajectory.points
                                  if x.time_from_start > tm_offset]

        # Ensure last waypoint
        if not goal.trajectory.points:
            last_point.time_from_start = tm_offset
            goal.trajectory.points = [last_point]

        # Shift based on time offset
        for p in goal.trajectory.points:
            p.time_from_start -= tm_offset + rospy.Duration(1)

        return goal


class InterruptAllControllers(object):
    def __init__(self, ns, *controllers):
        self.controllers = [InterruptController(x) for x in controllers]

        if not ns.endswith('/'):
            ns += '/'

        self.interrupt_sub = rospy.Subscriber(
            ns + 'interrupt', Empty, self.interrupt_all)
        self.resume_sub = rospy.Subscriber(
            ns + 'resume', Empty, self.resume_all)

    def interrupt_all(self, msg):
        for c in self.controllers:
            c.interrupt()

    def resume_all(self, msg):
        for c in self.controllers:
            c.resume()


if __name__ == '__main__':
    rospy.init_node("interrupt", anonymous=True)

    ###  actionlib_tutorials/Fibonacci
    # InterruptInstance = InterruptController('fibonacci', actionlib_tutorials.msg.FibonacciAction)

    ###  Single controller
    # InterruptInstance = InterruptController('r_arm_controller/follow_joint_trajectory')

    ###  Multiple controllers
    InterruptInstance = InterruptAllControllers(
        'fullbody_controller',
        'l_arm_controller/follow_joint_trajectory',
        'head_traj_controller/follow_joint_trajectory',
        'r_arm_controller/follow_joint_trajectory',
        'torso_controller/follow_joint_trajectory')

    rospy.spin()
