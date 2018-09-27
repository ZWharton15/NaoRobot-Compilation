import openpose
from subprocess import Popen
import cv2
import numpy as np
import socket
import os
import sys

class Brain():
	def __init__(self, body):
		self.body = body
		self.pose = OpenPose()
		self.object_detection = Object_Detection()
		self.HOI = Human_Object_Interaction(self.pose, self.object_detection)
		self.mirror_pose = Pose_Mirroring(self.pose)
		self.age_gender = Age_Gender()
	
	def predict_age_gender(self):
		self.age_gender.start()
	
	def predict_objects(self):
		img = self.body.look()
		cv2.imwrite("D:/RobotMaster/img/cap.jpg", img)
		objs = self.object_detection.predict()
		for obj in objs:
			print(ob[0])
		print("~~~~~END OF OBJECTS~~~~~")
		
	def predict_interaction():
		img = self.body.look()
		cv2.imwrite("D:/RobotMaster/img/cap.jpg", img)
		relations = self.HOI.calculate_HOI(objs)
		print(relations)

	def predict_pose():
		img = self.body.look()
		pose = self.mirror_pose.get_pose(img)
		print(pose)
	
class OpenPose():
	def __init__(self):
		self.pose_dectection = init_openpose()
		self.joint_names = ['nose','neck','rshoulder','relbow','rwrist','lshoulder','leblow','lwrist','midhip','rhip','rknee','rankle','lhip','lknee','reye','leye', 'rear', 'lear', 'lbigtoe', 'lsmalltoe', 'lheel', 'rbigtoe', 'rsmalltoe', 'rheel', 'background']
		pass
	
	#Done - untested
	def init_openpose(self):
		params = dict()
		params["logging_level"] = 3
		params["output_resolution"] = "-1x-1"
		params["net_resolution"] = "-1x368"
		params["model_pose"] = "BODY_25"
		params["alpha_pose"] = 0.6
		params["scale_gap"] = 0.3
		params["scale_number"] = 1
		params["render_threshold"] = 0.05
		params["num_gpu_start"] = 0
		params["disable_blending"] = False
		params["default_model_folder"] = "../../../models/"
		return OpenPose(params)

	#Done - untested
	def get_pose(self, image):
		image = cv2.imread("D:/RobotMaster/images/cap.jpg")
		keypoints = self.pose_dectection.forward(image, False)
		return keypoints
	
	#Done - untested
	def keypoints_to_joint_names(self, keypoints):
		joints = []
		try:
			for i in range(0, len(keypoints)):
				print(i)
				joints.append([self.joint_names[i], keypoints[i]])
			
			return joints
		except:
			#If there are no joints in the frame, return an empty joint list for 1 person
			joints = []
			print("No Joints")
			for i in range(0, 25):
				joints.append([self.joint_names[i], [0,0,0]])
			return joints
	
	#Done - untested
	def get_keypoint_by_name(self.keypoints, joint_name):
		joints = self.keypoints_to_joint_names(keypoints)
		for name in joints:
			if name[0] == joint_name:
				return name[1]
		print("JOINT NOT FOUND")
		return False

class Object_Detection():
	def __init__(self, model_name="faster_rcnn_resnet101_coco_11_06_2017", class_file_name="D:/RobotMaster/res/classDictionary.txt"):
		self.object_detection_client = init_remote_model()
		self.class_file = open(class_file_name, "r")
		pass
	
	#Done - untested
	def get_name_from_index(self, class_id):
		objects = class_file.read().splitlines()
		return objects[class_id]
	
	#Done - untested
	def init_remote_model(self):
		while True:
			try:
				server_name = 'localhost'
				server_port = 33599
				
				client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                client_socket.connect((server_name, server_port))
			except:
				print("Server didn't respond...\nTrying again")
				
		
		return client_socket

	#Done - untested
	def predict(self):
		self.object_detection_client.send("GO".encode(encoding="utf-8"))
		objects_found = self.object_detection_client.recv(1024).decode()
		print(f"Objects found: {objects_found}")
		
		obj_list = []
		for i in range(int(objects_found)):
			obj_list.append(self.object_detection_client.recv(1024).decode)
		
		names = []
		scores = []
		top_left = []
		bottom_right = []
		for obj in obj_list:
			data = obj.split(",")
			names.append(data[0])
			scores.append(data[1])
			top_left.append(f"data[2],{data[3]}")
			bottom_right.append(f"data[4],{data[5]}")
		
		self.object_detection_client.send("ALL RECIEVED".encode(encoding="utf-8"))
		
		return list(zip(names, scores, top_left, bottom_right))

class Human_Object_Interaction():
	
	def __init__(self, pose, object):
		self.pose = pose
		self.object = object
		self.relations = []
		self.hand_actions = [{"verb": "typing on a ", "joint": [5,8], "objects": ["keyboard", "laptop"]},
               {"verb": "reading a ", "joint": [5,8], "objects": ["book"]},
               {"verb": "holding a ", "joint": [5,8], "objects": ["cell phone","bird","dog","cat","umbrella","backpack","tie","bottle","cup","orange","apple","mouse","remote","scissors","teddy bear","hair drier","toothbrush", "spoon", "fork"]}]
		self.face_actions = [{"verb": "talking on a", "joint": [1], "face": [1], "objects": ["cell phone"]},
               {"verb": "eating a ", "joint": [13], "objects": ["apple", "banana","sandwich","carrot", "orange"]},
               {"verb": "drinking from a ", "joint": [13], "objects": ["cup", "bottle"]}]
		self.misc_actions = [{"verb": "wearing a ", "joint": [3,6], "objects": ["backpack, tie"]}]
	
	#Done - untested
	def check_for_person(self, keypoints, min_joints=2):
		#Require at least x joints to be considered valid
		for joint in keypoints:
			if int(round(joint[0])) != 0:
				min_joints -= 1
				if min_joints == 0:
					return True
		return False
	
	def check_for_hand_interactions(self, object_data, joints):
		left_hand_area = extend_joint_area(self.pose.get_keypoint_by_name(joints, "lwrist"))
		right_hand_area = extend_joint_area(self.pose.get_keypoint_by_name(joints, "rwrist"))
		
		left_hand_occupied = False
		right_hand_occupied = False
		
		for obj in object_data:
			obj_name = obj[0]
			for action in self.hand_actions:
				for applicable_object in action['objects']:
					if obj_name == applicable_object:
						if not left_hand_occupied:
							if intersects(list(zip(obj[2], obj[3], obj[4], obj[5])), left_hand_area):
								object_data.remove(obj)
								left_hand_occupied = True
								self.relations.append("You are " + action['verb'] + " " + obj_name + " in your left hand")
								print("LEFT HAND INTERSECTS WITH " + obj_name)
						if not right_hand_occupied:
							if intersects(list(zip(obj[2], obj[3], obj[4], obj[5])), right_hand_area):
								right_hand_occupied = True
								self.relations.append("You are " + action['verb'] + " " + obj_name + " in your right hand")
								print("LEFT HAND INTERSECTS WITH " + obj_name)
						
						if left_hand_occupied and right_hand_occupied:
							print("BOTH HANDS BUSY, GIVING UP SEARCH")
							return object_data
		return object_data
	
	def check_for_face_interactions(self, object_data, joints):
		face_area = extend_joint_area(self.pose.get_keypoint_by_name(joints, "nose"))
		
		for obj in object_data:
			obj_name = obj[0]
			for action in self.face_actions:
				for applicable_object in action['objects']:
					if obj_name == applicable_object:
						if intersects(list(zip(obj[2], obj[3], obj[4], obj[5])), face_area):
							object_data.remove(obj)
							self.relations.append("You are " + action['verb'] + " " + obj_name)
							print("FACE INTERSECTS WITH " + obj_name)
		return object_data
	
	def check_for_misc_interactions(self, object_data, joints):
		#How Do?
		for action in self.misc_actions:
			for joint in action['joint']:
				current_joint_area = extend_joint_area(joints[joint])
				for obj in object_data:
					obj_name = obj[0]
					for applicable_object in action['objects']:
						if obj_name == applicable_object:
							if intersects(list(zip(obj[2], obj[3], obj[4], obj[5])), current_area):
								object_data.remove(obj)
								self.relations.append("You are " + action['verb'] + " " + obj_name)
								print(str(joint) + " INTERSECTS WITH " + obj_name)
		return object_data
	
	def intersects(obj_area, joint_area):
		#make new area to decide if it intersects
		#joint - left, top, right, bottom
		#object - left, right, top, bottom
		if joint_area[0] > obj_area[0] or joint_area[2] < obj_area[1]:
			x_intersect = True
		if joint_area[1] > obj_area[3] or joint_area[3] < obj_area[2]:
			y_intersect = True
		return x_intersect and y_intersect
	
	def extend_joint_area(self, joint, radius=65):
		x = int(round(joint[0])
		y = int(round(joint[1])
		
		left = x - radius
		top = y - radius
		right = x + radius
		bottom = y + radius
		#cropped_img = image[y-radius:y+radius, x-radius:x+radius]
		return list(zip(left, top, right, bottom))
	
	def calculate_HOI(self):
		self.relations = []
		
		joints = self.pose.get_pose()
		
		if not check_for_person(joints):
			print("No person found")
		
		object_data = self.object.predict() #Image should be saved elsewhere
		
		#Each check returns the object list any objects that were found removed from the next check (stops items being held and eaten at the same time)
		object_data = check_for_face_interactions(object_data, joints)
		object_data = check_for_hand_interactions(object_data, joints)
		object_data = check_for_misc_interactions(object_data, joints)
		
		print("Unused objects: " + object_data)
		
		return self.relations

class Pose_Mirroring():

	def __init__(self, pose, model_dir="D:/RobotMaster/Models", model_name="InceptionV3.hdf5"):
		self.pose = pose
		self.model = init_custom_pose_model(model_dir, model_name)
		self.part_pairs = [1,2,   1,5,   2,3,   3,4,   5,6,   6,7,   1,0,   0,15,   15,17]
		self.colours = [[255, 0, 0], [255, 85, 0], [255, 170, 0], [255, 255, 0], [170, 255, 0], [85, 255, 0], [0, 255, 0], \
          [0, 255, 85], [0, 255, 170], [0, 255, 255], [0, 170, 255], [0, 85, 255], [0, 0, 255], [85, 0, 255], \
          [170, 0, 255], [255, 0, 255], [255, 0, 170], [255, 0, 85]]
		pass
	
	#Done - untested
	def init_custom_pose_model(self, dir, name):
		try:
			model = load_model(os.path.join(dir, name))
			return model
		except:
			raise Exception("Model not found")

	def get_pose(self, frame):
		keypoints = pose.get_pose(frame)
		num_people = len(keypoints)
		if num_people == 0:
			print("No people found")
			return None
		
		#Only run on the first skeleton found, not multiple people
		frame = self.draw_person(frame, keypoints[0])
		label = self.predict(frame)
		label = self.label_to_name("D:/RobotMaster/res/legend.txt", label
		
		return label
	
	def label_to_name(self, legend, label_val):
		with open(legend, 'r') as f:
			lines = f.readlines()
			return lines[int(label_num].split(",")[1]
	
	def predict(self, img, img_target_size=299):
		img = cv2.resize(img, (img_target_size, img_target_size))
		x = np.expand_dims(img, axis=0)
		x = preprocess_input(x.astype(float))
		pred = self.model.predict(x)
		pred = pred.tolist()
		pred = pred[0]
		return pred.index(max(pred))
	
	def draw_person(self, img, person):
		partial_joints = [0, 1, 2, 3, 4, 5, 6, 7, 15, 16, 17, 18]
		upper_skeleton_img = np.zeros(img.shape[0], img.shape[1], 3), np.unit8)
		counter = 0
		
		for data in person:
			x, y, score = data
			if x > 0 and y > 0:
				if counter in partial_joints:
					cv2.cricle(upper_skeleton_img, (round(x), round(y)), 7, (0, 0, 255), -1)
					
			counter += 1
		
		upper_skeleton_img = visualise_person(upper_skeleton_img, person)
		cropped_img = crop_person(upper_skeleton_img, person, partial_joints)
		return cropped_img
	
	def crop_person(self, img, person, upper_indexs):
		minx = 99999
		miny = 99999
		maxx = 0
		maxy = 0
		counter = 0
		
		for joint in person:
			x, y, score = joint
			if counter in partial_joints:
				if x > 0
					if round(x) > maxx:
						maxx = x + 10
					if round(x) < minx:
						minx = x - 10
					if round(y) > maxy:
						maxy = y + 10
					if round(y) < miny:
						miny = y - 10
			counter += 1
		cropped = img[int(round(miny)):int(round(maxy)), int(round(minx)):int(round(maxx))]
		return cropped
	
	def visualise_person(self, img, person):
		pairs = self.part_pairs
		stickwidth = 4
		cur_img = img.copy()
		counter = 0;
		for i in range(0, len(pairs),2):   
			if person[pairs[i],0] > 0 and person[pairs[i+1],0] > 0:
				Y = [person[pairs[i],0], person[pairs[i+1],0]]
				X = [person[pairs[i],1], person[pairs[i+1],1]]
				mX = np.mean(X)
				mY = np.mean(Y)
				length = ((X[0] - X[1]) ** 2 + (Y[0] - Y[1]) ** 2) ** 0.5
				angle = math.degrees(math.atan2(X[0] - X[1], Y[0] - Y[1]))
				polygon = cv2.ellipse2Poly((int(mY),int(mX)), (int(length/2), stickwidth), int(angle), 0, 360, 1)
				cv2.fillConvexPoly(cur_img, polygon, colours[counter])
				counter = counter + 1
		
		img = cv2.addWeighted(img, 0.4, cur_img, 0.6, 0)
		return img
	
class Age_Gender():
	def __init__(self, dir="D:/NAORobot/RossProject/"):
		self.bat_file_dir = dir
		pass
	
	def start(self):
		Popen(os.path.join(self.dir, "CompleteBuild.bat"))

class Secret_Project():
	def __init__(self):
		self.lower_blue = np.array([0, 102, 255])
		self.upper_blue = np.array([0, 0, 204])
		pass
	
	def apply_mask(self, img):
		hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
		res = cv2.bitwise_and(img,img, mask=mask)
		
		kernel = np.ones((15,15),np.float32)/225
		smoothed = cv2.filter2D(res,-1,kernel)
		
		_, secret = cv2.threshold(smoothed, 30, 255, cv2.THRESH_BINARY)
		
		cv2.imshow("average", smoothed)
		cv2.imshow("secret", secret)
		
		if cv2.waitKey(1) & 0xFF == 27:
			cv2.destroyAllWindows()
			return False
	
	def reposition(self):
		
		pass