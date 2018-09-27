#from naoqi import ALProxy
from face import Face
from brain import Brain
from body import Body
from joint import Joint
import sys

class Robot():
	def __init__(self, ip, port):
		self.ip = ip
		self.port = port
		self.name = self.set_name()
		self.face = self.init_face()
		self.body = Body(self, ALProxy("ALMotion", ip, port), ALProxy("ALAutonomousMoves", IP, port))
		self.brain = Brain(self)
	
	#Done - untested
	def init_face(self):
		tts_proxy = ALProxy("ALTextToSpeech", self.ip, self.port)
		video_proxy = ALProxy("ALVideoDevice", self.ip, self.port)
		return Face(tts_proxy, video_proxy)
	
	#Done - untested
	def set_name(self):
		name = ""
		if ip == "1":
			name = "Blue"
		elif ip == "2":
			name = "Red"
		else:
			name = "... Oh God... Who am I? Please help me!"
		return name
	
	#Done - untested
	def look(self, display=True):
		img = self.face.get_image()
		if display:
			cv2.imshow("Camera {}".format(self.face.camera), img)
			cv2.waitKey(1)
			cv2.destroyAllWindows()
		return img

	def speak(self, message):
		self.face.speak(message)

#Done - untested
def parse_args():
	try:
		#IP, PORT
		return sys.argv[1], sys.argv[2]
	except:
		raise Exception("Error: Not enough arguments")


if __name__ == "__main__":
	ip, port = parse_args()
	robbie = Robot(ip, port)
	print("My name is " + robbie.name)
	for x in range(0,50):
		robbie.look()
	robbie.brain.predict_age_gender()
	
	from brain import Secret_Project
	secret = Secret_Project()
	should_run = True
	while should_run:
		img = robbie.look()
		should_run = secret.apply_mask(img)
		