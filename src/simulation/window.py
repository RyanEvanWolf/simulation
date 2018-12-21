import rospy
import tf.transformations
from geometry_msgs.msg import TransformStamped,Transform,Quaternion,Vector3,Pose,Point32

from sensor_msgs.msg import PointCloud,ChannelFloat32
from simulation.msg import simLandmark,simStereo
from visualization_msgs.msg import Marker,MarkerArray
import time
import numpy as np 

from bumblebee.stereo import *


import cv2

def rigid_transform_3D(previousLandmarks, currentLandmarks):
    N=previousLandmarks.shape[1]
    centroid_A = np.mean(previousLandmarks.T, axis=0)
    centroid_B = np.mean(currentLandmarks.T, axis=0)

    AA = copy.deepcopy(previousLandmarks.T - np.tile(centroid_A, (N, 1)))
    BB = copy.deepcopy(currentLandmarks.T - np.tile(centroid_B, (N, 1)))
    H = np.transpose(AA).dot(BB)

    U, S, Vt = np.linalg.svd(H)
    R = (Vt.T).dot( U.T)
    # special reflection case
    if(np.linalg.det(R) < 0):
        Vt[2,:] *= -1
        R = (Vt.T).dot(U.T)
    t = -R.dot(centroid_A.T) + centroid_B.T

    return R,t

class svdWindow:
    def __init__(self):
        ####
        self.kSettings=getCameraSettingsFromServer(cameraType="subROI")
        self.Landmarks={}
        self.current=None
        self.previous=None
    def extractMotion(self,nMaxIterations=150,terminateRMS=0.2,)



def svdExtraction(previousX,currentX,Pl,Pr):
    maxIt=150
    minParam=3
    go


class simpleWindow:
    def __init__(self,stereoTopicName="/simulatedStereo"):

        self.deltaPub=rospy.Publisher("deltaPose",Pose,queue_size=10)
        self.overallPub=rospy.Publisher("currentPose",Pose,queue_size=10)
        self.stereoPub=rospy.Publisher("currentPoints",PointCloud,queue_size=10,latch=True)
        self.stereoSub=rospy.Subscriber(stereoTopicName,simStereo,self.newFrame)
        self.active=PointCloud()
        self.previous=None
        self.current=None
        self.oldMarkers=MarkerArray()
    def newFrame(self,data):
        self.active.header.frame_id=data.frame
        self.active.points=[]
        c=ChannelFloat32()
        c.name="rgb"
        c.values.append(255)
        c.values.append(0)
        c.values.append(0)

        tracksCount=0
        newTracks=0

        for landmark in data.Active:
            inPoint=Point32()
            inPoint.x=landmark.x
            inPoint.y=landmark.y
            inPoint.z=landmark.z
            self.active.points.append(inPoint)
            # c=ChannelFloat32()
            # if(landmark.ID in self.seen):
            #     tracksCount+=1
            # else:
            #     newTracks+=1
            #     newM=Marker()

            #     newM.header.frame_id=data.frame
            #     newM.type=2
            #     newM.id=int(landmark.ID)
            #     newM.action=0
            #     newM.pose.position.x=landmark.x
            #     newM.pose.position.y=landmark.y
            #     newM.pose.position.z=landmark.z
            #     newM.pose.orientation.x=0
            #     newM.pose.orientation.y=0
            #     newM.pose.orientation.z=0
            #     newM.pose.orientation.w=1
            #     newM.scale.x=0.02
            #     newM.scale.y=0.02
            #     newM.scale.z=0.02
            #     newM.color.a=0.3
            #     newM.color.b=1
            #     self.oldMarkers.markers.append(newM)
            #     self.seen.append(landmark.ID)
        self.stereoPub.publish(self.active)
        # self.mapPub.publish(self.oldMarkers)
        #################

            ####first initialization
