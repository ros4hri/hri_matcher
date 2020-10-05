#! /usr/bin/python

import rospy
import sqlite3
import datetime
import time
import tf2_ros
import StringIO
import uuid
import numpy as np 
import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math


import rospy

import math
import geometry_msgs.msg


from functools import partial

#published messages 
from hri_msgs.msg import AgeAndGender
from hri_msgs.msg import Expression
from hri_msgs.msg import FacialActionUnits
from hri_msgs.msg import FacialLandmarks
from hri_msgs.msg import GazeSenderReceiver
from hri_msgs.msg import GazesStamped
from hri_msgs.msg import Person
from hri_msgs.msg import BodyAttitude
from std_msgs.msg import String

#subscribed messages
from hri_msgs.msg import PointOfInterest2D
from hri_msgs.msg import Skeleton2D
from social_nets.msg import BodyLang
from social_nets.msg import Face
from social_nets.msg import Faces
from vino_people_msgs.msg  import ReidentificationStamped
from vino_people_msgs.msg  import Reidentification
from vino_people_msgs.msg  import AgeGenderStamped
from vino_people_msgs.msg  import AgeGender
from vino_people_msgs.msg  import EmotionsStamped
from tf2_msgs.msg import TFMessage

max_heads = 7
window_width = 640 
window_height = 480 
markerArray = MarkerArray()
POI = PointOfInterest2D()
face_IDs = []
bod_IDs = []
person_IDs = []
faces_count = 0
feature_dict = {}

for head_id in range(max_heads):
    face_IDs.append(str(uuid.uuid4())[:5])
    bod_IDs.append(str(uuid.uuid4())[:5])
    person_IDs.append(str(uuid.uuid4())[:5])


# The class has several callback functions which are related to each model used, a call back must me created for each model used
# then the condition is added bellow to relate the data extracted from the callback to the reidentification table.  
class ListenCompare:
    def __init__(self):
        self.diff_face_ids = []
        self.diff_body_ids = []
        self.old_faces_ids = []
        self.old_bodies_ids = []
        self.faces_landmarks = []
        self.BODYDATA = []
        self.BODYLANG = None 
        self.agegender_data = []
        self.emotion_data = None
        self.PERSONDATA = None
        self.trans = []
        self.landmarks2d_x = None 
        self.face_pose = None
        self.NOSE_X = 0
        self.NOSE_Y = 0
        self.NECK_X = 0
        self.NECK_Y = 0
        self.RIGHT_SHOULDER_X = 0
        self.RIGHT_SHOULDER_Y = 0
        self.RIGHT_ELBOW_X = 0
        self.RIGHT_ELBOW_Y = 0
        self.RIGHT_WRIST_X = 0
        self.RIGHT_WRIST_Y = 0
        self.LEFT_SHOULDER_X = 0
        self.LEFT_SHOULDER_Y = 0
        self.LEFT_ELBOW_X = 0
        self.LEFT_ELBOW_Y = 0
        self.LEFT_WRIST_X  = 0
        self.LEFT_WRIST_Y = 0
        self.RIGHT_HIP_X = 0
        self.RIGHT_HIP_Y = 0
        self.RIGHT_KNEE_X = 0
        self.RIGHT_KNEE_Y = 0
        self.RIGHT_ANKLE_X  = 0
        self.RIGHT_ANKLE_Y = 0
        self.LEFT_HIP_X  = 0
        self.LEFT_HIP_Y = 0
        self.LEFT_KNEE_X  = 0
        self.LEFT_KNEE_Y = 0
        self.LEFT_ANKLE_X  = 0
        self.LEFT_ANKLE_Y = 0
        self.LEFT_EYE_X  = 0
        self.LEFT_EYE_Y = 0
        self.RIGHT_EYE_X = 0 
        self.RIGHT_EYE_Y = 0
        self.LEFT_EAR_X  = 0
        self.LEFT_EAR_Y = 0
        self.RIGHT_EAR_X = 0
        self.RIGHT_EAR_Y = 0 
        self.RIGHT_EAR=0
        self.count_faces = 0
        




 # This can be used to extract landmarks, action units, face count, left and right eye gaze, headpose. 
    def faces_callback(self,data):
     #iteration through the faces 
        self.count_faces = data.count
        for faces_2D in data.faces:
            self.landmarksArray_X = []
            self.landmarksArray_Y = []
            self.FAUs_Numbs = [] 
            self.FAUs_presence = [] 
            self.FAUS_intensity = [] 
            self.landmarks2d_x = faces_2D.landmarks_2d[30].x
            self.face_pose = [faces_2D.head_pose.position.x/1000 , faces_2D.head_pose.position.y/1000 , faces_2D.head_pose.position.z/1000]
            for all_landmarks in faces_2D.landmarks_2d:
                # print(all_landmarks.x)
                self.landmarksArray_X.append(all_landmarks.x)
                self.landmarksArray_Y.append(all_landmarks.y)

            for FAUs in faces_2D.action_units:
                self.FAUs_Numbs.append(FAUs.name[2]+ FAUs.name[3])
                self.FAUs_presence.append(FAUs.presence)
                self.FAUS_intensity.append(FAUs.intensity)
                # print(self.FAUs_Numbs)

            # print(self.landmarksArray_X + self.landmarksArray_Y)
            # print('-------------------------')
            self.process()


# age and gender data for each person. 
    def agegender_callback(self,data):

        self.agegender_data = data.objects #array of people 

        self.process()

    def tf_callback(self,data):

        self.tf_head_ID = data.transforms[0].child_frame_id
        print(self.tf_head_ID)
        self.process()
#emotion data for each person 
    def emotion_callback(self,data):
        for emotion_data in data.emotions:
            self.emotion = emotion_data.emotion
            self.emotion_roi_x = emotion_data.roi.x_offset
            self.emotion_roi_y = emotion_data.roi.y_offset
            self.emotion_roi_width = emotion_data.roi.width
            self.process()



    # Function for all the skeletal points
    def body_callback(self,data):
        self.BODYDATA = data.skeleton
        self.LEFT_EYE_X  = data.skeleton[data.LEFT_EYE].x
        self.LEFT_EYE_Y  = data.skeleton[data.LEFT_EYE].y
        self.NOSE_X = data.skeleton[data.NOSE].x
        self.NOSE_Y = data.skeleton[data.NOSE].y
        self.NECK_X = data.skeleton[data.NECK].x
        self.NECK_Y = data.skeleton[data.NECK].y
        self.RIGHT_SHOULDER_X = data.skeleton[data.RIGHT_SHOULDER].x
        self.RIGHT_SHOULDER_Y = data.skeleton[data.RIGHT_SHOULDER].y
        self.RIGHT_ELBOW_X = data.skeleton[data.RIGHT_ELBOW].x
        self.RIGHT_ELBOW_Y = data.skeleton[data.RIGHT_ELBOW].y
        self.RIGHT_WRIST_X = data.skeleton[data.RIGHT_WRIST].x
        self.RIGHT_WRIST_Y = data.skeleton[data.RIGHT_WRIST].y
        self.LEFT_SHOULDER_X = data.skeleton[data.LEFT_SHOULDER].x
        self.LEFT_SHOULDER_Y = data.skeleton[data.LEFT_SHOULDER].y
        self.LEFT_ELBOW_X = data.skeleton[data.LEFT_ELBOW].x
        self.LEFT_ELBOW_Y = data.skeleton[data.LEFT_ELBOW].y
        self.LEFT_WRIST_X  = data.skeleton[data.LEFT_WRIST].x
        self.LEFT_WRIST_Y = data.skeleton[data.LEFT_WRIST].y
        self.RIGHT_HIP_X = data.skeleton[data.RIGHT_HIP].x
        self.RIGHT_HIP_Y = data.skeleton[data.RIGHT_HIP].y
        self.RIGHT_KNEE_X = data.skeleton[data.RIGHT_KNEE].x
        self.RIGHT_KNEE_Y = data.skeleton[data.RIGHT_KNEE].y
        self.RIGHT_ANKLE_X  = data.skeleton[data.RIGHT_ANKLE].x
        self.RIGHT_ANKLE_Y = data.skeleton[data.RIGHT_ANKLE].y
        self.LEFT_HIP_X  = data.skeleton[data.LEFT_HIP].x
        self.LEFT_HIP_Y = data.skeleton[data.LEFT_HIP].y
        self.LEFT_KNEE_X  = data.skeleton[data.LEFT_KNEE].x
        self.LEFT_KNEE_Y = data.skeleton[data.LEFT_KNEE].y
        self.LEFT_ANKLE_X  = data.skeleton[data.LEFT_ANKLE].x
        self.LEFT_ANKLE_Y = data.skeleton[data.LEFT_ANKLE].y
        self.RIGHT_EYE_X = data.skeleton[data.RIGHT_EYE].x 
        self.RIGHT_EYE_Y = data.skeleton[data.RIGHT_EYE].y
        self.LEFT_EAR_X  = data.skeleton[data.LEFT_EAR].x
        self.LEFT_EAR_Y = data.skeleton[data.LEFT_EAR].y
        self.RIGHT_EAR_X = data.skeleton[data.RIGHT_EAR].x
        self.RIGHT_EAR_Y = data.skeleton[data.RIGHT_EAR].y 
        self.process()

    # function for the upper body pose 
    def BodyPrediction_callback(self,data):
        self.BODYLANG = data.lang
        self.body_ID = data.person
        self.process()

    # reidentification model 
    def person_callback(self,data):
        self.PERSONDATA = data.reidentified_vector[0]
        self.process()

    # TODO: store the ids for all the bodies and faces for them to be added in the IDs topic
    # TODO: store the features themselves so the latest position can be published. 

    def process(self):



        faces_ids_update = []
        bodies_ids_update = []
        for faces_count in range(self.count_faces): 
            faces_ids_update.append(face_IDs[faces_count])
            bodies_ids_update.append(bod_IDs[faces_count])
        
        self.person_ID = person_IDs[int (self.PERSONDATA.identity[3])]

        self.person_ID_topic = "humans/persons/%s" % (self.person_ID)
        # print( self.person_ID)



        Face_ID_unique = face_IDs[int (self.tf_head_ID[4])]

        Face_ID= "humans/faces/"+Face_ID_unique

        Body_ID_unique = bod_IDs[self.body_ID]
        Body_ID = "humans/bodies/"+ Body_ID_unique # for a 5 char long ID

        x_offset = self.PERSONDATA.roi.x_offset
        width = self.PERSONDATA.roi.width
        x_b = self.NOSE_X

        if self.emotion_roi_x < self.landmarks2d_x < self.emotion_roi_x + self.emotion_roi_width:

            FL = FacialLandmarks()
            FL.landmarks = []

            

            for LA in range(len(self.landmarksArray_X)):
                # print(LA)
                POI.x = self.landmarksArray_X[LA]
                POI.y = self.landmarksArray_Y[LA]
                FL.landmarks.append(POI)
            # print(FL.landmarks)
            # print("-------------")

            # print(FL)
            self.landmarks_publisher = rospy.Publisher(Face_ID  + "/landmarks"  , FacialLandmarks ,queue_size=1)
            self.landmarks_publisher.publish(FL)



            Exp = Expression()
            OpenVino_emo = str (self.emotion.upper())
            print(OpenVino_emo)
            Exp.expression = getattr(Exp,OpenVino_emo)

            self.Expression_publisher = rospy.Publisher(Face_ID  + "/expression"  ,Expression ,queue_size=1)
            self.Expression_publisher.publish(Exp)

            self.FAU_publisher = rospy.Publisher(Face_ID + "/facs"  ,FacialActionUnits,queue_size=1)
            FAUs_data = FacialActionUnits()
            
            FAUs_data.FAU = [int(i) for i in self.FAUs_Numbs]
            # FAUs_data.presence = self.FAUs_presence
            FAUs_data.intensity = self.FAUS_intensity
            # print(FAUs_data)
            self.FAU_publisher.publish(FAUs_data)

            # holding the old face data 
            feature_dict[Face_ID_unique] = [FAUs_data,FL,Exp]


        self.Skele2D_publisher = rospy.Publisher(Body_ID  + "/skeleton2D"  ,Skeleton2D,queue_size=10)
        skele2d = Skeleton2D()
        skele2d.skeleton = [POI]*18

        skele2d.skeleton[skele2d.NOSE].x=  self.NOSE_X
        skele2d.skeleton[skele2d.NOSE].y= self.NOSE_Y 
        skele2d.skeleton[skele2d.NECK].x= self.NECK_X
        skele2d.skeleton[skele2d.NECK].y= self.NECK_Y 
        skele2d.skeleton[skele2d.RIGHT_SHOULDER].x=  self.RIGHT_SHOULDER_X
        skele2d.skeleton[skele2d.RIGHT_SHOULDER].y= self.RIGHT_SHOULDER_Y
        skele2d.skeleton[skele2d.RIGHT_ELBOW].x= self.RIGHT_ELBOW_X
        skele2d.skeleton[skele2d.RIGHT_ELBOW].y= self.RIGHT_ELBOW_Y
        skele2d.skeleton[skele2d.RIGHT_WRIST].x= self.RIGHT_WRIST_X
        skele2d.skeleton[skele2d.RIGHT_WRIST].y= self.RIGHT_WRIST_Y
        skele2d.skeleton[skele2d.LEFT_SHOULDER].x= self.LEFT_SHOULDER_X 
        skele2d.skeleton[skele2d.LEFT_SHOULDER].y= self.LEFT_SHOULDER_Y 
        skele2d.skeleton[skele2d.LEFT_ELBOW].x= self.LEFT_ELBOW_X
        skele2d.skeleton[skele2d.LEFT_ELBOW].y= self.LEFT_ELBOW_Y
        skele2d.skeleton[skele2d.LEFT_WRIST].x= self.LEFT_WRIST_X 
        skele2d.skeleton[skele2d.LEFT_WRIST].y= self.LEFT_WRIST_Y 
        skele2d.skeleton[skele2d.RIGHT_HIP].x= self.RIGHT_HIP_X
        skele2d.skeleton[skele2d.RIGHT_HIP].y= self.RIGHT_HIP_Y
        skele2d.skeleton[skele2d.RIGHT_KNEE].x= self.RIGHT_KNEE_X
        skele2d.skeleton[skele2d.RIGHT_KNEE].y= self.RIGHT_KNEE_Y
        skele2d.skeleton[skele2d.RIGHT_ANKLE].x= self.RIGHT_ANKLE_X
        skele2d.skeleton[skele2d.RIGHT_ANKLE].y= self.RIGHT_ANKLE_Y 
        skele2d.skeleton[skele2d.LEFT_HIP].x= self.LEFT_HIP_X 
        skele2d.skeleton[skele2d.LEFT_HIP].y= self.LEFT_HIP_Y
        skele2d.skeleton[skele2d.LEFT_KNEE].x= self.LEFT_KNEE_X
        skele2d.skeleton[skele2d.LEFT_KNEE].y= self.LEFT_KNEE_Y
        skele2d.skeleton[skele2d.LEFT_ANKLE].x= self.LEFT_ANKLE_X
        skele2d.skeleton[skele2d.LEFT_ANKLE].y= self.LEFT_ANKLE_Y
        skele2d.skeleton[skele2d.LEFT_EYE].x = self.LEFT_EYE_X 
        skele2d.skeleton[skele2d.LEFT_EYE].y  = self.LEFT_EYE_Y
        skele2d.skeleton[skele2d.RIGHT_EYE].x = self.RIGHT_EYE_X 
        skele2d.skeleton[skele2d.RIGHT_EYE].y= self.RIGHT_EYE_Y 
        skele2d.skeleton[skele2d.LEFT_EAR].x= self.LEFT_EAR_X 
        skele2d.skeleton[skele2d.LEFT_EAR].y= self.LEFT_EAR_Y
        skele2d.skeleton[skele2d.RIGHT_EAR].x= self.RIGHT_EAR_X
        skele2d.skeleton[skele2d.RIGHT_EAR].y =  self.RIGHT_EAR_Y

        self.Skele2D_publisher.publish(skele2d)

        self.BodPred_publisher = rospy.Publisher(Body_ID  + "/attitude"  ,BodyAttitude,queue_size=1)

        bodlang = BodyPose()
        OpenVino_lang = str (self.BODYLANG.upper())

    
        bodlang.lang = getattr(bodlang,OpenVino_lang)
        


        self.BodPred_publisher.publish(bodlang)


        # updating a dict to hold the old values of a feature. 
        feature_dict[Body_ID_unique] = [skele2d,bodlang]
        if self.old_faces_ids: 
            self.diff_face_ids = set(self.old_faces_ids) - set(faces_ids_update)
        if self.old_bodies_ids: 
            self.diff_body_ids = set(self.old_bodies_ids) - set(bodies_ids_update)
        self.old_faces_ids = faces_ids_update
        self.old_bodies_ids = bodies_ids_update

        for id in self.diff_face_ids:
            self.landmarks_publisher.publish(feature_dict.get(id)[1])
            self.FAU_publisher.publish(feature_dict.get(id)[0])
            self.Expression_publisher.publish(feature_dict.get(id)[2])

        for id in self.diff_body_ids:   
            self.BodPred_publisher.publish(feature_dict.get(id)[1])  
            self.Skele2D_publisher.publish(feature_dict.get(id)[0])       

        # a series of conditions are added to relate all the features to the reidentfication model, any new model added must have a condition
        for agegender in self.agegender_data:
            # condition for age and gender 
            if x_offset < agegender.roi.x_offset <  x_offset + width  or  x_offset < agegender.roi.x_offset +  agegender.roi.width <  x_offset + width: 
               #condition for emotions
                if x_offset < self.emotion_roi_x < x_offset + width:

                    #condition for 2D facial landmarks
                    if x_offset < self.landmarks2d_x < x_offset + width:
                        
                        #condition for skeletal points
                        if x_offset < window_width - x_b < x_offset + width:

                        

                            self.face_ID_publisher = rospy.Publisher(self.person_ID_topic + "/face_id"  ,String,queue_size=1, latch=True)
                            self.face_ID_publisher.publish(str (faces_ids_update))

                            self.body_ID_publisher = rospy.Publisher(self.person_ID_topic + "/body_id"  ,String,queue_size=1, latch=True)
                            self.body_ID_publisher.publish(str (bodies_ids_update))

                            AG = AgeAndGender()
                            AG.age = agegender.age
                            OpenVino_gender = agegender.gender


                            AG.gender = getattr(AG,OpenVino_gender)
                            # print(AG)

                            self.AgeGender_publisher = rospy.Publisher(self.person_ID_topic  + "/demographics"  ,AgeAndGender,queue_size=1)
                            self.AgeGender_publisher.publish(AG)

                        
                            
                        

                        
if __name__ == "__main__":


    rospy.init_node('humans_matcher')

    # Subscribers for all the models that publish to ROS
    ls = ListenCompare()
    rospy.Subscriber("/body_pose", Skeleton2D, ls.body_callback, queue_size=1)
    rospy.Subscriber("/body_prediction", BodyLang,
                     ls.BodyPrediction_callback, queue_size=1)
    rospy.Subscriber("/openface2/faces", Faces,
                     ls.faces_callback, queue_size=1)
    rospy.Subscriber("/ros_openvino_toolkit/reidentified_persons",
                     ReidentificationStamped, ls.person_callback, queue_size=1)
    rospy.Subscriber("/ros_openvino_toolkit/age_genders_Recognition",
                     AgeGenderStamped, ls.agegender_callback, queue_size=1)
    rospy.Subscriber("/ros_openvino_toolkit/emotions_recognition",
                     EmotionsStamped, ls.emotion_callback, queue_size=1)

    rospy.Subscriber("/tf",TFMessage, ls.tf_callback, queue_size=1)

    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
    # #     # group = select_humans(conn)
    # #     # humans_pub.publish(group)
    # #     # conn.commit()
        rate.sleep()

    # # conn.close()
