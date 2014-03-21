#!/usr/bin/python

import roslib
roslib.load_manifest('cob_script_server')
import rospy
import time
import tf
import yaml

from tf import TransformListener
from tf import transformations

from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

from simple_script_server import *
sss = simple_script_server()

slamPose = PoseStamped()
LocPose = PoseWithCovarianceStamped()
listener = []
trans = 0
rot = 0
tasks_file = ""
result_file = ""

def xyz_to_mat44(pos):
  return transformations.translation_matrix((pos.x, pos.y, pos.z))  

def xyzw_to_mat44(ori):
   return transformations.quaternion_matrix((ori.x, ori.y, ori.z, ori.w))

def cbSlamPose(data):
  global slamPose
  slamPose = data

def cbLocPose(data):
  global LocPose
  LocPose = data

def save_mapbase():
  global trans, rot
  try:
    (trans,rot) = listener.lookupTransform('/map', '/odom_combined', rospy.Time(0))
  except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    print "Error receiving transform"
    return

def getError(pose_name = ""):
  global trans, rot
  mat44 = numpy.dot(transformations.translation_matrix(trans), transformations.quaternion_matrix(rot))

  pose44 = numpy.dot(xyz_to_mat44(slamPose.pose.position), xyzw_to_mat44(slamPose.pose.orientation))
  # txpose is the new pose in target_frame as a 4x4
  txpose = numpy.dot(mat44, pose44)

  # xyz and quat are txpose's position and orientation
  xyz = tuple(transformations.translation_from_matrix(txpose))[:3]
  euler = tuple(transformations.euler_from_matrix(txpose))
  
  error_x = xyz[0] - LocPose.pose.pose.position.x
  error_y = xyz[1] - LocPose.pose.pose.position.y
  error_rot = euler[2] - euler_from_quaternion([LocPose.pose.pose.orientation.x, LocPose.pose.pose.orientation.y, LocPose.pose.pose.orientation.z, LocPose.pose.pose.orientation.w])[2]

  error = {pose_name: {"time": time.strftime("%d.%m.%Y um %H:%M:%S Uhr"), "error_x": str(error_x), "error_y": str(error_y), "error_rot": str(error_rot)}}
  with open(result_file, 'a') as outfile:
     outfile.write(yaml.dump(error, default_flow_style=False))

if __name__ == "__main__":
  global tasks_file, result_file
  global listener
  rospy.init_node("asd")
  startSLAM = rospy.ServiceProxy('/startSLAM', Empty)
  rospy.Subscriber("/karto_pose", PoseStamped, cbSlamPose)
  rospy.Subscriber("/state_ekf", PoseWithCovarianceStamped, cbLocPose)
  listener = tf.TransformListener()

  tasks_file = rospy.get_param('~tasks_file')  
  result_file = rospy.get_param('~result_file')  
  open(result_file, 'w')

  tasks = yaml.load(file(tasks_file, 'r'))
  for goal in tasks["goals"]:
    print goal
    handle = sss.move("base",[goal['x'], goal['y'], goal['rot']], False)
    time.sleep(tasks["slam_reset_delay"])
    if(goal["reset_slam"] == True):
      print "starting SLAM"
      startSLAM()
      save_mapbase()
    handle.wait()
    print "first pose done"
    getError(goal["name"])
