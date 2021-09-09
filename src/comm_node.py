#!/usr/bin/env python
#! Terminal 1 - minicom
#  and type anything
# Terminal 2
#  python communicator_node /dev/ttyUSB0 57600

import serial
from serial import Serial, SerialException, SerialTimeoutException
import rospy
import sys
import threading

from autolanding.srv import *
from autolanding.msg import camstatus
from mavros_commons import MavrosCommons
from std_msgs.msg import *
from flightaction import ControlState
from six.moves import xrange

class FakeSerialModel_Server:
  def __init__(self):
    self._msgs = bytearray()
    self._fake_serial_server = rospy.Service("fake_serial/topics",fakestr_srv,self.fake_serial_cb)
    
  def read(self,bytes_remaining):
    temp = bytearray()
    if self._msgs.__len__ > bytes_remaining:
      temp = self._msgs
      self._msgs = bytearray()
    else:
      temp = self._msgs[:bytes_remaining]
      self._msgs[bytes_remaining:]
    return temp

  def write(self,str2send):
    rospy.logwarn("Fake Serial Send : "+str2send)

  def fake_serial_cb(self, req):
    fake_serial_read = bytearray(req.msgs)
    self._msgs += fake_serial_read
    return True


class CommunicatorModel:
  def __init__(self,node_name="comm_node",
                    hz=10.0,
                    port_name="/dev/ttyUSB0",
                    baudrate=57600,
                    serial_timeout=1.0):
    rospy.init_node(node_name)
    self._readpx4 = MavrosCommons()
    self._rate = rospy.Rate(hz)
    self._port_name = port_name
    self._baudrate = baudrate
    self._serial_timeout = serial_timeout
    self._port = FakeSerialModel_Server()
    self._control_cmd = "FAIL" # from wireless comm.
    self._command_srv = 0
    self._camrec_srv = False
    self._control_state = "FAIL" # now control state
    
    self._cam_state = False
    self._msgs = bytearray()


  def setup(self):
    self._readpx4.setup()
    service_timeout = 1
    self.PRINT("waiting for ROS services")
    
    # Scan nodes booted
    timeout = 5
    loop_freq = 1  # Hz
    rate = rospy.Rate(loop_freq)
    reached_control = False
    reached_cam = False
    reached_command = False
    reached_camrec = False
    for i in xrange(timeout * loop_freq):
      # Control Node
      #if reached_control == False :
        #try:
          #rospy.wait_for_service('control_node/control_cmd', service_timeout)
          #self._control_cmd_call = rospy.ServiceProxy("control_node/control_cmd",ctrl_srv)
          #reached_control = True
        #except rospy.ROSException:
          #self.PRINT_ERROR("failed to connect to control_node services")

      # Command_srv Node
      if reached_command == False :
        try:
          rospy.wait_for_service('Command_srv', service_timeout)
          self._command_srv_call = rospy.ServiceProxy("Command_srv",Command_srv)
          self.PRINT("Command_srv are up")
          reached_command = True
        except rospy.ROSException:
          self.PRINT_ERROR("failed to connect to Command_srv services")

      # Camrec_srv Node
      if reached_camrec == False :
        try:
          rospy.wait_for_service('Camrec_srv', service_timeout)
          self._camrec_srv_call = rospy.ServiceProxy("Camrec_srv",Camrec_srv)
          self.PRINT("Camrec_srv are up")
          reached_camrec = True
        except rospy.ROSException:
          self.PRINT_ERROR("failed to connect to Camrec_srv services")

      # done?
      if reached_command & reached_camrec:
        break
      # Wait loop
      rate.sleep()

    # Cannot Open Serial Port
    if self._port_name[:4] != "fake":
      try:
        self._port = serial.Serial(self._port_name,self._baudrate,timeout=self._serial_timeout)
        self.PRINT("Open serial port {0}, {1} bps\n".format(self._port_name, self._baudrate))
      except SerialException as e:
        self.PRINT_ERROR("Error opening serial: %s"+str(e))
        pass

  def PRINT(self, msgs):
    rospy.loginfo(msgs)
    self._port.write(msgs+"\r\n")

  def PRINT_ERROR(self, msgs):
    rospy.logerr(msgs)
    self._port.write("[ERROR] "+msgs+"\r\n")

  def PRINT_WARN(self, msgs):
    rospy.logwarn(msgs)
    self._port.write("[WARN] "+msgs+"\r\n")
      
  def run(self):
    start_time = rospy.Time.now()
    Timeidx = 1.00
    while not rospy.is_shutdown():
      #
      # Show States
      #
      if rospy.Time.now() - start_time > rospy.Duration(Timeidx):
        msg2send = "T{0:.0f} /".format(Timeidx)
        msg2send += self._readpx4.state.mode + "/"
        msg2send += str(self._readpx4.state.armed) + "/"
        # msg2send += "vNED {0:>4.1f} {1:>4.1f} {2:>4.1f} ".format(self.local_velocity.twist.linear.x,
        #                                                     self.local_velocity.twist.linear.y,
        #                                                     self.local_velocity.twist.linear.z)
        msg2send += "pNED {0:>4.1f} {1:>4.1f} {2:>4.1f} ".format(self._readpx4.local_position.pose.position.x,
                                                            self._readpx4.local_position.pose.position.y,
                                                            self._readpx4.local_position.pose.position.z)
        self.PRINT(msg2send)
      
        Timeidx += 1.0
      #
      # Read from serial port
      #
      bytes_remaining = 100
      received = self._port.read(bytes_remaining)
      if len(received) is not 0:
        rospy.loginfo("remaining {0}, received {1}".format(bytes_remaining,received))
        self.message_parser(received)
        if received[:2] == "BB":
          self._command_srv_call(self._command_srv)
        elif received[:2] == "CC":
          self._camrec_srv_call(self._camrec_srv)

      
      # Read from mavros_node
      # sleep
      self._rate.sleep()

  # Moving command
  # 1, 2        Takeoff, Landing
  #   ^         upward
  # < x         leftside forward
  #     o >     backward rightside
  #     v       downward
  # !, N, F     stop, cam on/off
  def message_parser(self, received_msg):
    if received_msg[:2] == "AA":
      if received_msg[2:7] == "11111":
        msg2send = "Receive TAKEOFF!"
        self._control_cmd = "TAKEOFF"
      elif received_msg[2:7] == "22222":
        msg2send = "Receive LANDING!"
        self._control_cmd = "LANDING"

      elif received_msg[2:7] == "xxxxx":
        msg2send = "Receive FORWARD!"
        self._control_cmd = "FORWARD"
      elif received_msg[2:7] == "ooooo":
        msg2send = "Receive BACKWARD!"
        self._control_cmd = "BACKWARD"
      elif received_msg[2:7] == ">>>>>":
        msg2send = "Receive RIGHTSIDE!"
        self._control_cmd = "RIGHTSIDE"
      elif received_msg[2:7] == "<<<<<":
        msg2send = "Receive LEFTSIDE!"
        self._control_cmd = "LEFTSIDE"
      elif received_msg[2:7] == "^^^^^":
        msg2send = "Receive UPWARD!"
        self._control_cmd = "UPWARD"
      elif received_msg[2:7] == "vvvvv":
        msg2send = "Receive DOWNWARD!"
        self._control_cmd = "DOWNWARD"
      elif received_msg[2:7] == "RRRRR":
        msg2send = "Receive Right Turn!"
        self._control_cmd = "RIGHTTURN"
      elif received_msg[2:7] == "LLLLL":
        msg2send = "Receive Left Turn!"
        self._control_cmd = "LEFTTURN"

      elif received_msg[2:7] == "HHHHH":
        msg2send = "Receive RTH!"
        self._control_cmd = "RTH"
      elif received_msg[2:7] == "!!!!!":
        msg2send = "Receive STOP!"
        self._control_cmd = "STOP"
        
      else:
        msg2send = "Receive WRONG PROTOCAL!"
      self.PRINT(msg2send)

    elif received_msg[:2] == "BB":
      if received_msg[2:7] == "11111":
        msg2send = "Receive set Offboard mode!"
        self._command_srv = 1

      elif received_msg[2:7] == "22222":
        msg2send = "Receive Take Off!"
        self._command_srv = 2

      elif received_msg[2:7] == "33333":
        msg2send = "Receive Set position!"
        self._command_srv = 3

      elif received_msg[2:7] == "44444":
        msg2send = "Receive Return!"
        self._command_srv = 4

      elif received_msg[2:7] == "55555":
        msg2send = "Receive landing!"
        self._command_srv = 5

      elif received_msg[2:7] == "66666":
        msg2send = "Receive Auto landing engage!"
        self._command_srv = 6

      else:
        msg2send = "Receive WRONG PROTOCAL!"
      self.PRINT(msg2send)

    elif received_msg[:2] == "CC":
      if received_msg[2:7] == "RECON":
        msg2send = "Receive Camera record on!"
        self._camrec_srv = 1

      elif received_msg[2:7] == "RECOF":
        msg2send = "Receive Camera record off!"
        self._camrec_srv = 0

      else:
        msg2send = "Receive WRONG PROTOCAL!"
      self.PRINT(msg2send)

    else:
        msg2send = "Receive WRONG PROTOCAL!"
    self.PRINT(msg2send)


if __name__ == "__main__":
  node_name = "comm_node"
  hz = 10.0
  baudrate = 57600
  port_name = "/dev/ttyUSB0"
  serial_timeout = 1.0

  # rospy.sleep(10)
  print("Wait for messages")

  #################################################
  # Argumentations
  #################################################
  # if len(sys.argv) == 1:
  #   rospy.loginfo("Not use serial port")
  #   port_name = rospy.get_param("serial_port")
  # elif len(sys.argv) == 2:  # Port
  #   port_name = sys.argv[1]
  #   rospy.loginfo("Port Name {0}".format(port_name))
  # elif len(sys.argv) == 3: # Port and baudrate
  #   port_name = sys.argv[1]
  #   rospy.loginfo("Port Name {0}".format(port_name))
  #   baudrate = sys.argv[2]
  #   rospy.loginfo("Baudrate {0}".format(baudrate))
  # else:
  #   rospy.loginfo("Not enough arguments!")
  #   exit()
  # port_name = rospy.get_param("fcu_url","fake")
  # baudrate = rospy.get_param("baudrate",57600)

  # port_name = "fake"
  rospy.loginfo("Port name : " + port_name + ", baudrate : {:.0f}".format(baudrate))
  
  comm_model = CommunicatorModel(node_name,hz,port_name,baudrate,serial_timeout)
  comm_model.setup()
  comm_model.run()
