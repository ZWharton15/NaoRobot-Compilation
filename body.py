from naoqi import ALProxy
import time
import almath
from joint import Joint

class Body():
	def __init__(self, robot, motion_proxy, am_proxy):
		self.posture = Posture(robot, motion_proxy)
		self.motion_proxy = motion_proxy
		self.am_proxy = am_proxy
		self.manager_proxy = ALProxy("ALBehaviorManager", robot.ip, robot.port)
		self.master_joint = Joint(motion_proxy)
		self.unrefined_movement = Unrefined_Control(master_joint, motion_proxy)
		self.is_aware = True
		self.robot_upper_joints = ['Head', 'LShoulder', 'LElbow', 'LWrist', 'RShoulder', 'RElbow', 'RWrist','LHand', 'RHand']
		self.previous_behaviour = None
		pass
	
	def toggle_awareness(self, hard_reset=False):
		if hard_reset:
			self.is_aware = False
		
		if self.is_aware:
			self.is_aware = False
			self.am_proxy.setBackgroundStrategy("none")
			print("Awareness Proxy disabled")
		else:
			self.is_aware = True
			self.am_proxy.setBackgroundStrategy("backToNeutral")
			print("Awareness Proxy enabled")
	
	def arm_straight_up(self, l_or_r, speed=0.6):
		if l_or_r == "L":
			self.master_joint.set_pitch("LShoulder", -0.3, speed)
		elif l_or_r == "R":
			self.master_joint.set_pitch("RShoulder", -0.3, speed)
		else:
			raise TypeError("Joint must either be L or R")
	
	def arm_down(self, l_or_r, speed=0.6):
		if l_or_r == "L":
			self.master_joint.set_pitch("LShoulder", 1.6, speed)
		elif l_or_r == "R":
			self.master_joint.set_pitch("RShoulder", 1.6, speed)
		else:
			raise TypeError("Joint must either be L or R")
	
	def arm_half_up(self, l_or_r, speed=0.6):
		if l_or_r == "L":
			self.master_joint.set_pitch("LShoulder", -0.5, speed)
		elif l_or_r == "R":
			self.master_joint.set_pitch("RShoulder", -0.5, speed)
		else:
			raise TypeError("Joint must either be L or R")
	
	def arm_out(self, l_or_r, speed=0.6):
		if l_or_r == "L":
			self.master_joint.set_roll("LShoulder", 1.2, speed)
		elif l_or_r == "R":
			self.master_joint.set_roll("RShoulder", -1.2, speed)
		else:
			raise TypeError("Joint must either be L or R")
	
	def arm_straight_forward(self, l_or_r, speed=0.6):
		if l_or_r == "L":
			self.master_joint.set_roll("LShoulder", -1.0, speed)
			self.master_joint.set_pitch("LShoulder", 0.5, speed)
		elif l_or_r == "R":
			self.master_joint.set_roll("RShoulder", 1.0, speed)
			self.master_joint.set_pitch("RShoulder", 0.5, speed)
		else:
			raise TypeError("Joint must either be L or R")
	
	def elbow_straighten(self, l_or_r, speed=0.6):
		if l_or_r == "L":
			self.master_joint.set_roll("LShoulder", 1.0, speed)
		elif l_or_r == "R":
			self.master_joint.set_roll("RShoulder", -1.0, speed)
		else:
			raise TypeError("Joint must either be L or R")
	
	def elbow_bend(self, l_or_r, speed=0.6):
		if l_or_r == "L":
			self.master_joint.set_roll("LElbow", -1.5, speed)
		elif l_or_r == "R":
			self.master_joint.set_roll("RElbow", 1.5, speed)
		else:
			raise TypeError("Joint must either be L or R")
	
	def arm_out_bend_up(self, l_or_r, speed=0.6):
		if l_or_r == "L":
			self.arm_straight_up("L")
			self.arm_out("L")
			self.elbow_bend("L")
		elif l_or_r == "R":
			self.arm_straight_up("R")
			self.arm_out("R")
			self.elbow_bend("R")
		else:
			raise TypeError("Joint must either be L or R")

	def walk(self, x_dir=0.5, y_dir=0.0, theta=0.0, freq=0.5, walk_duration=3):
		self.master_joint.stiffen("Body")
		self.posture.walk(x_dir, y_dir, theta, freq, walk_duration)
		self.master_joint.release("Body")
	
	def head_turn(self, l_or_r, speed=0.6):
		if l_or_r == "L":
			self.master_joint.set_yaw("Head", 2.0, speed)
		elif l_or_r == "R":
			self.master_joint.set_yaw("Head", -2.0, speed)
		elif l_or_r == "F":
			self.master_joint.set_yaw("Head", 0.0, speed)
		else:
			raise TypeError("Joint must either be L, R, or F")
	
	def head_up_down(self, l_or_r, speed=0.6):
		if l_or_r == "L":
			self.master_joint.set_pitch("Head", 0.5, speed)
		elif l_or_r == "R":
			self.master_joint.set_pitch("Head", -0.5, speed)
		elif l_or_r == "F":
			self.master_joint.set_pitch("Head", 0.0, speed)
		else:
			raise TypeError("Joint must either be L, R, or F")

	def hand_open(self, l_or_r):
		if l_or_r == "L":
			self.motion_proxy.openHand("LHand")
		elif l_or_r == "R":
			self.motion_proxy.openHand("RHand")
		else:
			raise TypeError("Joint must either be L or R")
	
	def hand_close(self, l_or_r):
		if l_or_r == "L":
			self.motion_proxy.closeHand("LHand")
		elif l_or_r == "R":
			self.motion_proxy.closeHand("RHand")
		else:
			raise TypeError("Joint must either be L or R")
			
	def hand_on_hip(self, l_or_r):
		if l_or_r == "L":
			self.unrefined_movement.left_shoulder_roll(50, speed=0.5)
			self.unrefined_movement.left_elbow_roll(-110, speed=0.5)
			self.unrefined_movement.left_elbow_yaw(10.5, speed=0.5)
		elif l_or_r == "R":
			self.unrefined_movement.right_shoulder_roll(-50, speed=0.5)
			self.unrefined_movement.right_elbow_roll(110, speed=0.5)
			self.unrefined_movement.right_elbow_yaw(-10.5, speed=0.5)
	
	def run_behaviour(self, behaviour_name=''):
		if behaviour_name == "stop":
			self.manager_proxy.stopBehavior(self.previous_behaviour)
			self.toggle_awareness(hard_reset=True)
			returns
		
		if behaviour_name in self.get_movements():
			if not self.manager_proxy.isBehaviorRunning(self.previous_behaviour):
                self.manager_proxy.post.runBehavior(behaviour_name)
                self.previous_behaviour = behaviour_name
                time.sleep(0.5)
            else:
                print("Behavior is already running.")
				return
		else:
			print("Behavior not found")
	
	def get_movements(self):
		names = manager_proxy.getInstalledBehaviors()
		print(names)
		return names
	
class Posture():
	
	def __init__(self, robot, motion_proxy):
		self.robot = robot
		self.motion_proxy = motion_proxy
		self.posture_proxy = ALProxy("ALRobotPosture", self.robot.ip, self.robot.port)
		pass
	
	def walk(self, x_dir=0.5, y_dir=0.0, theta=0.0, freq=0.5, walk_duration=3):
		if walk_duration > 20:
			self.robot.speak("I cannot walk for that long! Lower the walk duration value")
			return
		
		self.stand_squat(0.5)
		
		self.motion_proxy.setWalkArmsEnabled(True, True)
		self.motion_proxy.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION", True]])
		self.motion_proxy.setWalkTargetVelocity(x_dir, y_dir, theta, freq)
		time.sleep(walk_duration)
		self.motion_proxy.setWalkTargetVelocity(0,0,0,1)
	
	def sit(self, t=2.0):
		self.posture_proxy.goToPosture("Sit", t)
		pass
		
	def sit_alt(self, t=2.0):
		self.posture_proxy.goToPosture("SitRelax", t)
		pass
	
	def stand(self, t=2.0):
		self.posture_proxy.goToPosture("Stand", t)
		pass
		
	def stand_alt(self, t=2.0):
		self.posture_proxy.goToPosture("StandZero", t)
		pass
	
	def stand_squat(self, t=2.0):
		self.posture_proxy.goToPosture("StandInit", t)
		pass
	
	def lying_front(self, t=2.0):
		self.posture_proxy.goToPosture("LyingBelly", t)
		pass
	
	def lying_back(self, t=2.0):
		self.posture_proxy.goToPosture("LyingBack", t)
		pass
	
	def crouch(self, t=2.0):
		self.posture_proxy.goToPosture("Crouch", t)
		pass
	
	
class Unrefined_Control():

	def __init__(self, master_joint, motion_proxy):
		self.master_joint = master_joint
		self.motion_proxy = motion_proxy
		pass
	
	def right_shoulder_pitch(self, angle=0, speed=0.3):
        self.master_joint.set_pitch("RShoulder", int(angle)*almath.TO_RAD, speed)

    def left_shoulder_pitch(self, angle=0, speed=0.3):
        self.master_joint.set_pitch("LShoulder", int(angle)*almath.TO_RAD, speed)

    def right_shoulder_roll(self, angle=0, speed=0.3):
        self.master_joint.set_roll("RShoulder", int(angle)*almath.TO_RAD, speed)

    def left_shoulder_roll(self, angle=0, speed=0.3):
        self.master_joint.set_roll("LShoulder", int(angle)*almath.TO_RAD, speed)
        
    def right_elbow_roll(self, angle=0, speed=0.3):
        self.master_joint.set_roll("RElbow", int(angle)*almath.TO_RAD,speed)

    def left_elbow_roll(self, angle=0, speed=0.3):
        self.master_joint.set_roll("LElbow", int(angle)*almath.TO_RAD, speed)

    def right_elbow_yaw(self, angle=0, speed=0.3):
        self.master_joint.set_yaw("RElbow", int(angle)*almath.TO_RAD, speed)

    def left_elbow_yaw(self, angle=0, speed=0.3):
        self.master_joint.set_yaw("LElbow", int(angle)*almath.TO_RAD, speed)

    def right_wrist_yaw(self, angle=0, speed=0.3):
        self.master_joint.set_yaw("RWrist", int(angle)*almath.TO_RAD, speed)

    def left_wrist_yaw(self, angle=0, speed=0.3):
        self.master_joint.set_yaw("LWrist", int(angle)*almath.TO_RAD, speed)
	
	def head_yaw(self, angle=0, speed=0.3):
		self.master_joint.set_yaw("Head", int(angle)*almath.TO_RAD, speed)
	
	def head_pitch(self, angle=0, speed=0.3):
		self.master_joint.set_pitch("Head", int(angle)*almath.TO_RAD, speed)
	
	def right_hand_grip(self, angle=0, speed=0.3):
		self.motion_proxy.setStiffness("RHand", 1.0)
		self.motion_proxy.setAngles("RHand", int(angle)*almath.TO_RAD, speed)
	
	def left_hand_grip(self, angle=0, speed=0.3):
		self.motion_proxy.setStiffness("LHand", 1.0)
		self.motion_proxy.setAngles("LHand", int(angle)*almath.TO_RAD, speed)
	