import rospy
import rosnode
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import GetModelState
import numpy as np

import random
import time

def mover():
  rospy.init_node('set_pose')
  model_name = rospy.get_param('~model_name')
  direction = rospy.get_param('~direction')
  distance = rospy.get_param('~distance')
  step = 0.015
  frequency = 0.025
  
  rospy.wait_for_service('/gazebo/set_model_state')
  set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
  get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
  state_msg = ModelState()
  # state_msg.model_name = model_name
  model = get_state(model_name, "")
  y_position = model.pose.position.y
  x_position = model.pose.position.x
  state_msg.model_name = model_name
  state_msg.pose.position.x = x_position
  state_msg.pose.position.y = y_position
  state_msg.pose.position.z = model.pose.position.z
  state_msg.pose.orientation.x = model.pose.orientation.x
  state_msg.pose.orientation.y = model.pose.orientation.y
  state_msg.pose.orientation.z = model.pose.orientation.z
  state_msg.pose.orientation.w = model.pose.orientation.w

  if direction == "left/right":
    while True:
      for y in np.arange(y_position, y_position + distance, step):
        y_position = y
        state_msg.pose.position.y = y
        resp = set_state( state_msg )
        time.sleep(frequency)
      for y in np.arange(y_position, y_position - distance, -step):
        y_position = y
        state_msg.pose.position.y = y
        resp = set_state( state_msg )
        time.sleep(frequency)
  elif direction == "right/left":
    while True:
      for y in np.arange(y_position, y_position - distance, -step):
        y_position = y
        state_msg.pose.position.y = y
        resp = set_state( state_msg )
        time.sleep(frequency)
      for y in np.arange(y_position, y_position + distance, step):
        y_position = y
        state_msg.pose.position.y = y
        resp = set_state( state_msg )
        time.sleep(frequency)
  elif direction == "down/up":
    while True:
      for x in np.arange(x_position, x_position - distance, -step):
          x_position = x
          state_msg.pose.position.x = x
          resp = set_state( state_msg )
          time.sleep(frequency)
      for x in np.arange(x_position, x_position + distance, step):
        x_position = x
        state_msg.pose.position.x = x
        resp = set_state( state_msg )
        time.sleep(frequency)
  elif direction == "up/down":
    while True:
      for x in np.arange(x_position, x_position + distance, step):
          x_position = x
          state_msg.pose.position.x = x
          resp = set_state( state_msg )
          time.sleep(frequency)
      for x in np.arange(x_position, x_position - distance, -step):
        x_position = x
        state_msg.pose.position.x = x
        resp = set_state( state_msg )
        time.sleep(frequency)
  else :
    print("no direction was given")

  
mover()
