import math
import time
import random
import numpy as np
from sensor_msgs.msg import PointCloud,ChannelFloat32

from geometry_msgs.msg import Point32,PointStamped
from visualization_msgs.msg import Marker,MarkerArray

from bumblebee.stereo import *
from tf import TransformListener
import rospy

from tf.transformations import *

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
        



class simLandmark:
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


                # try:

                #     (trans,rot) = listener.lookupTransform(currentPoseName,prevPoseName,rospy.Time(0.0))
                #     print(trans,rot)

                #     R=quaternion_matrix(rot)

                #     R[0,3]=trans[0]
                #     R[1,3]=trans[1]
                #     R[2,3]=trans[2]
                #     print(R)
                #     print(R.dot(testPoint))
                #     print(listener.transformPoint(currentPoseName,test).point)
                #     valid=True

                    
                #     #print(test.point,listener.transformPoint(currentPoseName,test).point)
                # except Exception as e:
                #     print(e)
                #     time.sleep(0.2)
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
                and (Xa[1,0]<self.lSettings["HeightMaximum"])):
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
