from naoqi import ALProxy
import vision_definitions as vd

class Face():
	
	def __init__(self, tts_proxy, video_proxy, camera_resolution=vd.kQVGA, camera_colour=vd.kBGRColorSpace, camera_fps=24, camera=0):
		self.camera_resolution = camera_resolution
		self.camera_colour = camera_colour
		self.camera_fps = camera_fps
		self.camera = camera
		self.video_proxy = video_proxy
		self.camera_proxy = self.subscribe_to_camera()
		self.mouth_proxy = tts_proxy
		
	#Done - untested
	def speak(self, message, language="English", pitch=100, speed=100, volume=60):
		if language != "English":
			self.mouth_proxy.setLanguage(language)
		self.mouth_proxy.say("\\vct={}\\\\rspd={}\\\\vol={}\\{}".format(pitch, speed, volume, message))
	
	def eye_colour(self):
		pass
	
	
	#Done - untested
	def subscribe_to_camera(self, camera):
		self.camera = camera
		return capure_device("eyes", self.camera, self.camera_resolution, self.camera_colour, self.camera_fps)
	
	#Done - untested
	def get_image(self)
		result = self.video_proxy.getImageRemote(self.camera_proxy)
		width = result[0]
		height = result[1]
		image = np.zeros((height, width, 3), np.uint8)
		if result == None:
			return image
		
		values = map(ord, list(result[6]))
		i = 0
		for y in range(0, height):
            for x in range(0, width):
                image.itemset((y, x, 0), values[i + 0])
                image.itemset((y, x, 1), values[i + 1])
                image.itemset((y, x, 2), values[i + 2])
				i += 3
				
		return image