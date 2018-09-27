from naoqi import ALProxy

class Joint():
	def __init__(self, motion_proxy):
		self.motion_proxy = motion_proxy
		pass
	
	def set_pitch(self, joint_name, angle, speed):
		self.motion_proxy.setAngles(joint_name + "Pitch", angle, speed)
		pass
	
	def get_pitch(self):
	
		pass
	
	def set_yaw(self, joint_name, angle, speed):
		self.motion_proxy.setAngles(joint_name + "Yaw", angle, speed)
		pass
	
	def get_yaw(self):
	
		pass
	
	def set_roll(self, joint_name, angle, speed):
		self.motion_proxy.setAngles(joint_name + "Roll", angle, speed)
		pass
	
	def get_roll(self):
	
		pass
	
	def stiffen(self, joint_name)
		self.motion_proxy.stiffnessInterpolation(joint_name, 0.0, 1.0)
	
	def release(self, joint_name)
		self.motion_proxy.stiffnessInterpolation(joint_name, 1.0, 1.0)
		