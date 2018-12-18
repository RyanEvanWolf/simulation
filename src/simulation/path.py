from simulation.settings import *
import numpy as np
from tf.transformations import quaternion_from_euler,quaternion_matrix,euler_from_matrix

from tf import TransformBroadcaster
from geometry_msgs.msg import Pose,TransformStamped
from math import pi,radians,degrees
import os
import rospy
from visualization_msgs.msg import Marker,MarkerArray

import pickle

def MotionCategorySettings():
    Settings={}
    Settings["Fast"]={}
    Settings["Medium"]={}
    Settings["Slow"]={}
    Settings["Fast"]["TranslationMean"]=0.066
    Settings["Fast"]["RotationMean"]=30
    Settings["Fast"]["TranslationNoise"]=0.1*Settings["Fast"]["TranslationMean"] ##meters
    Settings["Fast"]["RotationNoise"]=8    ##degrees

    Settings["Medium"]["TranslationMean"]=0.044
    Settings["Medium"]["RotationMean"]=20
    Settings["Medium"]["TranslationNoise"]=0.1*Settings["Medium"]["TranslationMean"] ##meters
    Settings["Medium"]["RotationNoise"]=1        ##degrees

    Settings["Slow"]["TranslationMean"]=0.022
    Settings["Slow"]["RotationMean"]=10
    Settings["Slow"]["TranslationNoise"]=0.1*Settings["Slow"]["TranslationMean"] ##meters
    Settings["Slow"]["RotationNoise"]=1        ##degrees
    return Settings


def noisyRotations(noise=5):
    out=np.zeros((3,1))
    out[0,0]=np.random.normal(0,noise,1)
    out[1,0]=np.random.normal(0,noise*0.1,1)
    out[2,0]=np.random.normal(0,noise,1)
    return out



def forwardTranslation(zBase=0.2,noise=0.1):
    out=np.zeros((3,1))
    out[0,0]=np.random.normal(0,noise,1)
    out[1,0]=np.random.normal(0,noise,1)
    out[2,0]=abs(np.random.normal(zBase,noise,1))
    return out

def dominantRotation(yawBase=15,noise=5):
    out=np.zeros((3,1))
    out[0,0]=np.random.normal(0,noise,1)
    out[1,0]=np.random.normal(0,noise,1) 
    out[2,0]=np.clip(abs(np.random.normal(yawBase,noise,1)),0,40)
    return out

def genRandomCoordinate(xAvg,yAvg,zAvg):
    Point=np.ones((4,1),dtype=np.float64)
    Point[0,0]=np.random.normal(0,xAvg,1)
    Point[1,0]=np.random.normal(0,yAvg,1)
    Point[2,0]=np.random.normal(0,zAvg,1)
    return Point



def genStraightTransform(mSettings,nFrames=2):
    setTransforms=[]
    for i in range(0,nFrames-1):
        C=forwardTranslation(mSettings["TranslationMean"],mSettings["TranslationNoise"])
        Rtheta=noisyRotations(mSettings["RotationNoise"])
        setTransforms.append((Rtheta,C))
    return setTransforms 

def genTurningTransform(mSettings,nFrames=2):
    setTransforms=[]
    for i in range(0,nFrames-1):
        C=forwardTranslation(mSettings["TranslationMean"],mSettings["TranslationNoise"])
        Rtheta=dominantRotation(mSettings["RotationMean"],mSettings["RotationNoise"])
        setTransforms.append((Rtheta,C))
    return setTransforms  


class pathViewer:
    def __init__(self,pubName="path"):
        self.topicName=pubName
        self.interFrameMotions=[]
        self.br =TransformBroadcaster()
    def loadFromFile(self,Dir):
        fileNames=os.listdir(Dir)
        

        ###add original Transform

        origin=TransformStamped()
        origin.header.frame_id="world"
        origin.child_frame_id=self.topicName
        origin.transform.rotation.w=1
        self.interFrameMotions.append(origin)
        count=0
        for f in fileNames:
            with open(Dir+"/"+f,"r") as current:
                Rtheta,C=pickle.load(current)
                q=quaternion_from_euler(radians(Rtheta[0]),
                                radians(Rtheta[1]),
                                radians(Rtheta[2]),
                                'szxy')
                count+=1
                print(Rtheta)
                print(C)
                latestPose=TransformStamped()
                latestPose.header.frame_id=self.interFrameMotions[-1].child_frame_id
                latestPose.child_frame_id=f[:f.rfind(".")]

                latestPose.transform.translation.x=C[0,0]
                latestPose.transform.translation.y=C[1,0]
                latestPose.transform.translation.z=C[2,0]
                latestPose.transform.rotation.x= q[0]
                latestPose.transform.rotation.y=q[1]
                latestPose.transform.rotation.z=q[2]
                latestPose.transform.rotation.w=q[3]
                self.interFrameMotions.append(latestPose)
    def publish(self):
        for i in self.interFrameMotions:
            i.header.stamp=rospy.Time.now()
            self.br.sendTransformMessage(i)

      # while(True):
#     for i in transforms:
#         i.header.stamp=rospy.Time.now()
#         br.sendTransformMessage(i)
#     time.sleep(0.01)  
# ###########
# ###display Path

# transforms=[]

# br = tf.TransformBroadcaster()


# origin=TransformStamped()
# origin.header.frame_id="world"
# origin.child_frame_id="origin"
# origin.transform.rotation.w=1
# transforms.append(origin)


# for i in range(0,len(motions)):
#     cvTransformMsg=TransformStamped()
#     cvTransformMsg.header.frame_id=transforms[-1].child_frame_id
#     cvTransformMsg.child_frame_id=str(i).zfill(4)
#     cvTransformMsg.transform.translation.x=motions[i][1][0,0]
#     cvTransformMsg.transform.translation.y=motions[i][1][1,0]
#     cvTransformMsg.transform.translation.z=motions[i][1][2,0]


#     # q=quaternion_from_euler(radians(0),
#     #                                 radians(0),
#     #                                 radians(0))#Rtheta[2]),'szxy')

#     cvTransformMsg.transform.rotation.x=motions[i][0][0]
#     cvTransformMsg.transform.rotation.y=motions[i][0][1]
#     cvTransformMsg.transform.rotation.z=motions[i][0][2]
#     cvTransformMsg.transform.rotation.w=motions[i][0][3]
#     transforms.append(cvTransformMsg)

# while(True):
#     for i in transforms:
#         i.header.stamp=rospy.Time.now()
#         br.sendTransformMessage(i)
#     time.sleep(0.01)

# def genRelativePose(mSettings):
#     '''
#     pose delta= [R(quaternion rTheta) | -RC]=[R|T]
#     straight motion
#     '''
#     C=forwardTranslation(mSettings["TranslationMean"],mSettings["TranslationNoise"])
#     Rtheta=dominantRotation(mSettings["RotationMean"],mSettings["RotationNoise"])

#     q=quaternion_from_euler(radians(Rtheta[0]),
#                                 radians(Rtheta[1]),
#                                 radians(Rtheta[2]),
#                                 'szxy')
    
#     R=quaternion_matrix(q)[0:3,0:3]  
#     T=-R.dot(C)
#     return q,C

