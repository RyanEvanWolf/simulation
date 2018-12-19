import rospy
import tf.transformations
from geometry_msgs.msg import TransformStamped,Transform,Quaternion,Vector3,Pose,Point32

from sensor_msgs.msg import PointCloud,ChannelFloat32
from simulation.msg import simLandmark,simStereo
from visualization_msgs.msg import Marker,MarkerArray
import time

class simpleWindow:
    def __init__(self,stereoTopicName="/simulatedStereo"):

        self.deltaPub=rospy.Publisher("deltaPose",Pose,queue_size=10)
        self.overallPub=rospy.Publisher("currentPose",Pose,queue_size=10)
        self.stereoPub=rospy.Publisher("currentPoints",PointCloud,queue_size=10,latch=True)
        self.mapPub=rospy.Publisher("currentMap",MarkerArray,queue_size=10,latch=True)
        self.stereoSub=rospy.Subscriber(stereoTopicName,simStereo,self.newFrame)
        self.active=PointCloud()
        self.seen=[]
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
        ##calculate tracks


        print(time.time(),data.frame)