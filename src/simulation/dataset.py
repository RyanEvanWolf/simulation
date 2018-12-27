import math
import time
import random
import numpy as np
from sensor_msgs.msg import PointCloud,ChannelFloat32
from math import pi,radians,degrees
from geometry_msgs.msg import Point32,PointStamped
from visualization_msgs.msg import Marker,MarkerArray


from tf import TransformListener
import rospy

from tf.transformations import *
from geometry_msgs.msg import Pose,TransformStamped
import msgpack

from bumblebee.utils import createDir
from bumblebee.baseTypes import slidingGraph
import networkx as nx


from simulation.settings import *

import pickle


from simulation.srv import idealSimulation,idealSimulationResponse,idealSimulationRequest
from simulation.msg import simLandmark,simStereo
from simulation.path import *


from bumblebee.stereo import ROIfrmMsg
import cv2
import os

import matplotlib.pyplot as plt

def ROIcheck(pt,roi):
    ###
    ##check width
    if((pt[0,0]>=roi[0])and(pt[0,0]<=(roi[0]+roi[2]))):
        if((pt[1,0]>=roi[1])and(pt[1,0]<=(roi[1]+roi[3]))):   
            return True
        else:
            return False
    else:
        return False


def genRandomCoordinate(xAvg,yAvg,zAvg):
    Point=np.ones((4,1),dtype=np.float64)
    Point[0,0]=np.random.normal(0,xAvg,1)
    Point[1,0]=np.random.normal(0,yAvg,1)
    Point[2,0]=abs(np.random.normal(0,zAvg,1))
    return Point



# class simulator:
#     def __init__(self,lSettings,kSettings):
#         self.lSettings=lSettings
#         self.kSettings=kSettings   
#     def simulate(pathDir,nPoints=10):
#         ############
#         ###create first frame
        





class Landmark:
    def __init__(self,BaseId,X,l,r):
        self.frameTracks=[BaseId]
        self.X=X
        self.Ml=[l]
        self.Mr=[r]
    def addEdge(self,frame,l,r):
        self.frameTracks.append(frame)
        #self.X.append(x)
        self.Ml.append(l)
        self.Mr.append(r)

class simPlayBack:
    def __init__(self,rootDir,displayName="playback"):
        self.root=rootDir
        self.Landmarks={}
        self.tf=[]
        self.currentPose=displayName
        self.outName=displayName

        fileNames=os.listdir(rootDir)
        

        ###add original Transform

        origin=TransformStamped()
        origin.header.frame_id="world"
        origin.child_frame_id=self.outName
        origin.transform.rotation.w=1
        self.tf.append((origin,np.zeros((3,1)),np.zeros((3,1))))
        for f in fileNames:
            with open(self.root+"/Motion/"+f,"r") as current:
                Rtheta,C=pickle.load(current)
                q=quaternion_from_euler(radians(Rtheta[0]),
                                radians(Rtheta[1]),
                                radians(Rtheta[2]),
                                'szxy')
                latestPose=TransformStamped()
                latestPose.header.frame_id=self.tf[-1][0].child_frame_id
                latestPose.child_frame_id=f[:f.rfind(".")]

                latestPose.transform.translation.x=C[0,0]
                latestPose.transform.translation.y=C[1,0]
                latestPose.transform.translation.z=C[2,0]
                latestPose.transform.rotation.x= q[0]
                latestPose.transform.rotation.y=q[1]
                latestPose.transform.rotation.z=q[2]
                latestPose.transform.rotation.w=q[3]
    #             self.interFrameMotions.append(latestPose)
    # def publish(self):
    #     for i in self.interFrameMotions:
    #         i.header.stamp=rospy.Time.now()
    #         self.br.sendTransformMessage(i)





class stereo_simulator_node(slidingGraph):
    def __init__(self):
        super(stereo_simulator_node,self).__init__(displayName="simulation")
       # self.listener = tf2_ros.Buffer()

        self.mset=MotionCategorySettings()
        totalSeconds=5

        fps=1/15.0

        nFrames=int(totalSeconds/fps)

        self.newPoseVertex()
        motions=genStraightTransform(self.mset["Medium"],nFrames)

        for f in motions:
            poseID=self.newPoseVertex()
            
            Rtheta=f[0]
            C=f[1]
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
         #tf2_ros.TransformListener(tfBuffer)
        self.lSettings=getSimulatedLandmarkSettings()


    def simulate(self):
        frames=self.getPoseVertices()
        nTracks=400
        print("generating new simulation")
        for i in range(1,len(frames)):
            self.publishPoses()
            time.sleep(0.1)
            frameTracks=0
            currentPoseName=frames[i]
            previousPoseName=frames[i-1]
            added=0
            evaluated=0
            ##############
            ##how many tracsk from current to Future?
            for k in self.getLandmarkVertices():
                PoseVertices=self.getLandmarkConnections(k)
                if(previousPoseName in PoseVertices):
                    #######
                    ##get the edge, check it tracks from previous frame into current frame
                    #####
                    ###if it does, add it as an edge, increment number of tracks
                    e=self.edges[previousPoseName,k]
                    e["X"]

                    originalPoint=PointStamped()
                    originalPoint.header.stamp=rospy.Time(0)
                    originalPoint.point.x=e["X"][0,0]
                    originalPoint.point.y=e["X"][1,0]
                    originalPoint.point.z=e["X"][2,0]
                    originalPoint.header.frame_id=self.displayName+"/"+previousPoseName
                    PotentialPoint=self.listener.transformPoint(self.displayName+"/"+currentPoseName,originalPoint)
                    Xb=np.ones((4,1))
                    Xb[0,0]=PotentialPoint.point.x
                    Xb[1,0]=PotentialPoint.point.y
                    Xb[2,0]=PotentialPoint.point.z

                    Lb,Rb=self.kSettings["Pl"].dot(Xb),self.kSettings["Pr"].dot(Xb)
                    Lb/=Lb[2,0]
                    Rb/=Rb[2,0]

                    tracked=(self.checkWithinROI(Lb)and self.checkWithinROI(Rb,False)
                            and (Xb[1,0]<self.lSettings["HeightMaximum"])
                            and (Xb[2,0]>0))
                    if(tracked):
                        frameTracks+=1
                        Mb=np.zeros((4,1))
                        Mb[0,0]=Lb[0,0]
                        Mb[1,0]=Lb[1,0]
                        Mb[2,0]=Rb[0,0]
                        Mb[3,0]=Rb[1,0]

                        self.add_edge(currentPoseName,k,X=Xb,M=Mb)

            #######
            ##for each landmark lost, generate a new one from previous to current
            for Extra in range(nTracks-frameTracks):
                self.genLandmark(previousPoseName,currentPoseName)
                added+=1
            print(previousPoseName,currentPoseName,added,frameTracks)
            self.publishCurrentPose()
            #self.svdRANSAC(previousPoseName,currentPoseName)
            #self.publishGlobalPoints()
            #self.publishLocalPoints(currentPoseName)
            # img=np.zeros((768,1024,3))

            # for k in self.getLandmarkVertices():
            #     self.plotLandmark(img,k,[previousPoseName,currentPoseName])
            # cv2.imshow("a",img)
            # cv2.waitKey(1)


        # feed=[]  
        print(len(self.getLandmarkVertices())  )
        print(len(self.getPoseVertices())  )
        print(len(self.nodes()))#
        print(len(self.edges()))

        # for k in self.getLandmarkVertices():
        #     #print(k)
        #     feed.append(len(self.getLandmarkConnections(k)))
        # plt.hist(feed,14)
        # plt.show()   
    def genLandmark(self,current,future,sourceName="ideal"):
        Ans=None
        while(Ans is None):
            Xa=genRandomCoordinate(self.lSettings["Xdepth"],
                                    self.lSettings["Ydepth"],
                                    self.lSettings["Zdepth"])    
            La,Ra=self.kSettings["Pl"].dot(Xa),self.kSettings["Pr"].dot(Xa)
            La/=La[2,0]
            Ra/=Ra[2,0]
            originalPoint=PointStamped()
            originalPoint.header.stamp=rospy.Time(0)
            originalPoint.point.x=Xa[0,0]
            originalPoint.point.y=Xa[1,0]
            originalPoint.point.z=Xa[2,0]
            originalPoint.header.frame_id=self.displayName+"/"+current
            framesTracked=0
            if(self.checkWithinROI(La)
                and self.checkWithinROI(Ra,False)
                and (Xa[1,0]<self.lSettings["HeightMaximum"])
                and (Xa[2,0]>0)):
                PotentialPoint=self.listener.transformPoint(self.displayName+"/"+future,originalPoint)
                Xb=np.ones((4,1))
                Xb[0,0]=PotentialPoint.point.x
                Xb[1,0]=PotentialPoint.point.y
                Xb[2,0]=PotentialPoint.point.z

                Lb,Rb=self.kSettings["Pl"].dot(Xb),self.kSettings["Pr"].dot(Xb)
                Lb/=Lb[2,0]
                Rb/=Rb[2,0]

                tracked=(self.checkWithinROI(Lb)and self.checkWithinROI(Rb,False)
                        and (Xb[1,0]<self.lSettings["HeightMaximum"])
                        and (Xb[2,0]>0))
                if(tracked):
                    #################
                    #################
                    newID=self.newLandmarkVertex()
                    M=np.zeros((4,1))
                    M[0,0]=La[0,0]
                    M[1,0]=La[1,0]
                    M[2,0]=Ra[0,0]
                    M[3,0]=Ra[1,0]
                    self.add_edge(current,newID,X=Xa,M=M)

                    Mb=np.zeros((4,1))
                    Mb[0,0]=Lb[0,0]
                    Mb[1,0]=Lb[1,0]
                    Mb[2,0]=Rb[0,0]
                    Mb[3,0]=Rb[1,0]
                    self.add_edge(future,newID,X=Xb,M=Mb)
                    Ans=True 
    def checkWithinROI(self,pt,left=True):
        if(left):
            return ROIcheck(pt,ROIfrmMsg(self.kSettings["lInfo"].roi))
        else:
            return ROIcheck(pt,ROIfrmMsg(self.kSettings["rInfo"].roi))
    def createSlidingGraph(self,dispName="graph"):
        ans=slidingGraph(displayName=dispName,graph=self)

        for a in ans.getPoseVertices():
            previous=ans.nodes[a]["msg"].header.frame_id
            if(previous!="world"):
                ans.nodes[a]["msg"].header.frame_id=ans.displayName+previous[previous.rfind('/'):]
            current=ans.nodes[a]["msg"].child_frame_id
            ans.nodes[a]["msg"].child_frame_id=ans.displayName+current[current.rfind('/'):]
        return ans



# class stereo_simulator_node:
#     def __init__(self):
#         self.lSettings=getSimulatedLandmarkSettings()
#         self.kSettings=getCameraSettingsFromServer(cameraType="subROI")
#         self.s=rospy.Service("/idealSimulation",idealSimulation,self.genSimulation)
#         self.pub=rospy.Publisher("/simulatedStereo",simStereo,queue_size=10)
#         self.mapPub=rospy.Publisher("/dataMap",MarkerArray,queue_size=10,latch=True)
#         self.listener=TransformListener()
#         self.Landmarks={}
#     def genSimulation(self,req):
#         Landmarks={}
#         count=0     ###total landmarks created in simulation
#         print("generating new simulation")
#         for i in range(0,len(req.interMotions)-1):
#             frameTracks=0
#             currentPoseName=req.interMotions[i]
#             futurePoseName=req.interMotions[i+1]
#             added=0
#             print(currentPoseName,futurePoseName)
#             while(frameTracks<req.nTracks):
#                 frameTracks=0
#                 ##############
#                 ##how many tracsk from current to Future?
#                 for k in Landmarks.keys():
#                     if((currentPoseName in Landmarks[k].frameTracks)
#                         and (futurePoseName in Landmarks[k].frameTracks)):
#                             frameTracks+=1
#                 for Extra in range(req.nTracks-frameTracks):
#                     newVertex=self.genLandmark(req.interMotions[i:])    
#                     Landmarks[str(count).zfill(7)]=newVertex
#                     count+=1 
#                     added+=1
#             print(added,count,frameTracks)


#         print("Ready to begin publishing")

#         active=MarkerArray()
#         for k in Landmarks.keys():
#             newM=Marker()
#             newM.header.frame_id=Landmarks[k].frameTracks[0]
#             c=ChannelFloat32()
#             c.name="rgb"
#             c.values.append(255)
#             c.values.append(255)
#             c.values.append(0)

#             newM.type=2
#             newM.id=int(k)
#             newM.action=0
#             newM.pose.position.x=Landmarks[k].X[0,0]
#             newM.pose.position.y=Landmarks[k].X[1,0]
#             newM.pose.position.z=Landmarks[k].X[2,0]
#             newM.pose.orientation.x=0
#             newM.pose.orientation.y=0
#             newM.pose.orientation.z=0
#             newM.pose.orientation.w=1
#             newM.scale.x=0.1
#             newM.scale.y=0.1
#             newM.scale.z=0.1
#             newM.color.a=0.6
#             newM.color.b=1
#             active.markers.append(newM)
#         self.mapPub.publish(active)

#         print("map published")

#         for f in req.interMotions:


#             a=simStereo()
#             a.frame=f
#             for k in Landmarks.keys():
#                 if(f in Landmarks[k].frameTracks):

#                     originalPoint=PointStamped()
#                     originalPoint.point.x=Landmarks[k].X[0,0]
#                     originalPoint.point.y=Landmarks[k].X[1,0]
#                     originalPoint.point.z=Landmarks[k].X[2,0]
#                     originalPoint.header.frame_id=Landmarks[k].frameTracks[0]
#                     currentPoint=self.listener.transformPoint(f,originalPoint)
#                     activeLandmark=simLandmark()
#                     activeLandmark.x=currentPoint.point.x
#                     activeLandmark.y=currentPoint.point.y
#                     activeLandmark.z=currentPoint.point.z
#                     activeLandmark.ID=k
#                     a.Active.append(activeLandmark)
#             print("totalLandmarks",len(a.Active),f)
#             self.pub.publish(a)
#             time.sleep(1)
#         return idealSimulationResponse()








class idealSimulator:
    def __init__(self,lSettings,kSettings,nTracks=10):
        self.lSettings=lSettings
        self.kSettings=kSettings   
        self.Landmarks={}
        self.count=0
        self.tracks=nTracks
        for i in range(nTracks):
            data=self.genLandmark()     
            newVertex=simLandmark("path",data[0],data[1],data[2])    
            self.Landmarks[str(self.count).zfill(7)]=newVertex
            self.count+=1
        self.p= rospy.Publisher('ptss',MarkerArray,queue_size=10,latch=True)
    def dumpTo(self,fileDir):
        createDir(fileDir)
        for k in self.Landmarks.keys():
            with open(fileDir+"/"+k+".landmark","wb") as inFile:

                pickle.dump(self.Landmarks[k],inFile)
    def run(self,motionDir):
        ###assumes it has been initialized
        #assumes ros node exists
        listener = TransformListener()
        time.sleep(2)
        PoseFiles=os.listdir(motionDir)
        prevPoseName="path"
        currentPoseName=None
        for f in PoseFiles:
            frameTracks=0
            frameLosses=0
            currentPoseName=f[:f.rfind(".")]
            print(prevPoseName,currentPoseName)
                   #####
                ##check each landmark if it was present in the previous frame
            
            for k in self.Landmarks.keys():
                if(prevPoseName in self.Landmarks[k].frameTracks):
                    ########
                    ##check if it is still visible
                    
                    ###
                    ##transform from first sighting into current coordinate frame
                    originalPoint=PointStamped()
                    originalPoint.point.x=self.Landmarks[k].X[0,0]
                    originalPoint.point.y=self.Landmarks[k].X[1,0]
                    originalPoint.point.z=self.Landmarks[k].X[2,0]
                    originalPoint.header.frame_id=self.Landmarks[k].frameTracks[0]

                    PotentialPoint=listener.transformPoint(currentPoseName,originalPoint)
                    ########################
                    ##project it onto the cameras
                    hX=np.ones((4,1))
                    hX[0,0]=PotentialPoint.point.x
                    hX[1,0]=PotentialPoint.point.y
                    hX[2,0]=PotentialPoint.point.z

                    L=self.kSettings["Pl"].dot(hX)
                    L/=L[2,0]
                    R=self.kSettings["Pr"].dot(hX)
                    R/=R[2,0]

                    if(self.checkWithinROI(L) and self.checkWithinROI(R,False)):
                        self.Landmarks[k].Ml.append(L)
                        self.Landmarks[k].Mr.append(R)
                        self.Landmarks[k].frameTracks.append(currentPoseName)
                        frameTracks+=1
                    else:
                        data=self.genLandmark()     
                        newVertex=simLandmark(currentPoseName,data[0],data[1],data[2])    
                        self.Landmarks[str(self.count).zfill(7)]=newVertex
                        self.count+=1
                        frameLosses+=1
            print(frameTracks,frameLosses,self.count)
            prevPoseName=currentPoseName
    def publishLandmarks(self):
        
        out = MarkerArray()
        for k in self.Landmarks.keys():
            newM=Marker()
            newM.header.frame_id=self.Landmarks[k].frameTracks[0]
            newM.type=2
            newM.id=int(k)
            newM.action=0
            newM.pose.position.x=self.Landmarks[k].X[0,0]
            newM.pose.position.y=self.Landmarks[k].X[1,0]
            newM.pose.position.z=self.Landmarks[k].X[2,0]
            newM.pose.orientation.x=0
            newM.pose.orientation.y=0
            newM.pose.orientation.z=0
            newM.pose.orientation.w=1
            newM.scale.x=0.02
            newM.scale.y=0.02
            newM.scale.z=0.02
            newM.color.a=0.3
            newM.color.r=1
           # newM.duration=rospy.Duration(0)

            out.markers.append(newM)
        self.p.publish(out)
    def genLandmark(self):
        Ans=None
        while(Ans is None):
            Xa=genRandomCoordinate(self.lSettings["Xdepth"],
                                    self.lSettings["Ydepth"],
                                    self.lSettings["Zdepth"])    
            La,Ra=self.kSettings["Pl"].dot(Xa),self.kSettings["Pr"].dot(Xa)
            La/=La[2,0]
            Ra/=Ra[2,0]

            if(self.checkWithinROI(La)
                and self.checkWithinROI(Ra,False)
                and (Xa[1,0]<self.lSettings["HeightMaximum"])
                and (Xa[2,0]>0)):
                    Ans=Xa,La,Ra
        return Ans    
    def checkWithinROI(self,pt,left=True):
        if(left):
            return ROIcheck(pt,ROIfrmMsg(self.kSettings["lInfo"].roi))
        else:
            return ROIcheck(pt,ROIfrmMsg(self.kSettings["rInfo"].roi))




class simStereoFrame:
    def __init__(self,lSettings,kSettings):
        self.lSettings=lSettings
        self.kSettings=kSettings
        self.Xl=[]
        self.Ml=[]
        self.Mr=[]
        self.ptCloud=None
        self.lImage=None
        self.rImage=None
        self.epiImage=None
        self.pub= rospy.Publisher('ptss',PointCloud,queue_size=10,latch=True)
        self.pub2= rospy.Publisher('ptss2',PointCloud,queue_size=10,latch=True)
    def genNewFrames(self,nPoints=10):
        for i in range(nPoints):
            validPoint=False
            while(not validPoint):
                Xa=genRandomCoordinate(self.lSettings["Xdepth"],
                                        self.lSettings["Ydepth"],
                                        self.lSettings["Zdepth"])    
                La,Ra=self.kSettings["Pl"].dot(Xa),self.kSettings["Pr"].dot(Xa)
                La/=La[2,0]
                Ra/=Ra[2,0]

                if(self.checkWithinROI(La)
                    and self.checkWithinROI(Ra,False)
                    and (Xa[1,0]<self.lSettings["HeightMaximum"])):
                        print("valid")
                        validPoint=True
                        self.Xl.append(Xa)
                        self.Ml.append(La)
                        self.Mr.append(Ra)
        #####
        ##publish pointCloud

        ster=PointCloud()
        ster.header.frame_id="path"
        for i in self.Xl:
            latest=Point32()
            ch=ChannelFloat32()
            ch.name="intensity"
            ch.values.append(1)
            latest.x=i[0,0]
            latest.y=i[1,0]
            latest.z=i[2,0]
            print(latest)
            ster.points.append(latest)
            ster.channels.append(ch)
        self.pub.publish(ster)   
    def checkWithinROI(self,pt,left=True):
        if(left):
            return ROIcheck(pt,ROIfrmMsg(self.kSettings["lInfo"].roi))
        else:
            return ROIcheck(pt,ROIfrmMsg(self.kSettings["rInfo"].roi))
