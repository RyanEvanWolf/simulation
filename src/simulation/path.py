from simulation.settings import *
import numpy as np
from tf.transformations import quaternion_from_euler,quaternion_matrix,euler_from_matrix

from tf import TransformBroadcaster
import tf2_ros
import time


from geometry_msgs.msg import Pose,TransformStamped
from math import pi,radians,degrees
import os
import rospy
from visualization_msgs.msg import Marker,MarkerArray
import copy

import threading
import pickle
import networkx as nx
from bumblebee.baseTypes import slidingGraph


def MotionCategorySettings():
    Settings={}
    Settings["Fast"]={}
    Settings["Medium"]={}
    Settings["Slow"]={}
    Settings["Fast"]["TranslationMean"]=0.066
    Settings["Fast"]["RotationMean"]=8
    Settings["Fast"]["TranslationNoise"]=0.1*Settings["Fast"]["TranslationMean"] ##meters
    Settings["Fast"]["RotationNoise"]=5    ##degrees

    Settings["Medium"]["TranslationMean"]=0.044
    Settings["Medium"]["RotationMean"]=5
    Settings["Medium"]["TranslationNoise"]=0.1*Settings["Medium"]["TranslationMean"] ##meters
    Settings["Medium"]["RotationNoise"]=0.2        ##degrees

    Settings["Slow"]["TranslationMean"]=0.022
    Settings["Slow"]["RotationMean"]=2
    Settings["Slow"]["TranslationNoise"]=0.1*Settings["Slow"]["TranslationMean"] ##meters
    Settings["Slow"]["RotationNoise"]=1        ##degrees
    return Settings


def noisyRotations(noise=5):
    out=np.zeros((3,1))
    out[0,0]=np.clip(np.random.normal(0,noise,1),-2,2)
    out[1,0]=np.clip(np.random.normal(0,noise,1),-2,2)
    out[2,0]=np.clip(np.random.normal(0,noise,1),-2,2)
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
        C=forwardTranslation(mSettings["TranslationMean"]+0.3,mSettings["TranslationNoise"])
        Rtheta=dominantRotation(mSettings["RotationMean"],mSettings["RotationNoise"])
        setTransforms.append((Rtheta,C))
    return setTransforms  




class pureOdometry(slidingGraph):
    def __init__(self,dispName="ideal"):
        super(pureOdometry,self).__init__(displayName=dispName)
        self.mset=MotionCategorySettings()
        totalSeconds=1

        fps=1/15.0

        nFrames=int(totalSeconds/fps)

        self.newPoseVertex()
        motions=genStraightTransform(self.mset["Medium"],nFrames)

        for f in motions:
            poseID=self.newPoseVertex()
            
            Rtheta=f[0]
            C=f[1]
            print(poseID,C)
            q=quaternion_from_euler(radians(Rtheta[0]),
                    radians(Rtheta[1]),
                    radians(Rtheta[2]),
                    'szxy')  
            latestPose=TransformStamped()
            latestPose.transform.translation.x=C[0,0]
            latestPose.transform.translation.y=C[1,0]
            latestPose.transform.translation.z=C[2,0]
            latestPose.transform.rotation.x=q[0]
            latestPose.transform.rotation.y=q[1]
            latestPose.transform.rotation.z=q[2]
            latestPose.transform.rotation.w=q[3]

            self.nodes[poseID]["msg"].transform=latestPose.transform
            # self.nodes[poseID][]
            #             Rtheta,C=self.input[m][0],self.input[m][1]
            # q=quaternion_from_euler(radians(Rtheta[0]),
            #                     radians(Rtheta[1]),
            #                     radians(Rtheta[2]),
            #                     'szxy')  
            # latestPose=TransformStamped()
            # latestPose.header.frame_id=self.frameNames[m]
            # latestPose.child_frame_id=self.frameNames[m+1]

            # latestPose.transform.translation.x=C[0,0]
            # latestPose.transform.translation.y=C[1,0]
            # latestPose.transform.translation.z=C[2,0]
            # latestPose.transform.rotation.x=q[0]
            # latestPose.transform.rotation.y=q[1]
            # latestPose.transform.rotation.z=q[2]
            # latestPose.transform.rotation.w=q[3]
        print(len(self.nodes()))



class simTFpub(threading.Thread):
    def __init__(self,motionSet,pubName="simulation"):
        threading.Thread.__init__(self)
        self.pubName=pubName
        self.input=copy.deepcopy(motionSet)
        self.tfMessages=[]
        self.frameNames=[self.pubName]
        self.isDaemon=True
        for i in range(0,len(motionSet)):
            self.frameNames.append(str(i).zfill(8))
        
        origin=TransformStamped()
        origin.header.frame_id="world"
        origin.child_frame_id=self.pubName
        origin.transform.rotation.w=1
        self.tfMessages.append(origin)


        for m in range(0,len(self.input)):
            Rtheta,C=self.input[m][0],self.input[m][1]
            q=quaternion_from_euler(radians(Rtheta[0]),
                                radians(Rtheta[1]),
                                radians(Rtheta[2]),
                                'szxy')  
            latestPose=TransformStamped()
            latestPose.header.frame_id=self.frameNames[m]
            latestPose.child_frame_id=self.frameNames[m+1]

            latestPose.transform.translation.x=C[0,0]
            latestPose.transform.translation.y=C[1,0]
            latestPose.transform.translation.z=C[2,0]
            latestPose.transform.rotation.x=q[0]
            latestPose.transform.rotation.y=q[1]
            latestPose.transform.rotation.z=q[2]
            latestPose.transform.rotation.w=q[3]


            self.tfMessages.append(latestPose)
    def run(self):
        br = tf2_ros.StaticTransformBroadcaster()
        for i in self.tfMessages:
            i.header.stamp=rospy.Time.now()
        br.sendTransform(self.tfMessages)
        # while(True):
        #     # for i in self.tfMessages:
        #     #     i.header.stamp=rospy.Time.now()
        #     #     br.sendTransformMessage(i)
        #     time.sleep(0.1)




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
                print(f)
                latestPose=TransformStamped()
                latestPose.header.frame_id=self.interFrameMotions[-1].child_frame_id
                latestPose.child_frame_id=self.topicName+"/"+f[:f.rfind(".")]

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

