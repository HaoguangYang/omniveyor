#!/usr/bin/env python2

import rospy
from std_msgs.msg import UInt16MultiArray, MultiArrayDimension
#from twilio.rest import Client
import xml.etree.ElementTree as ET

class genericPayload():
    def __init__(self):
        """
        sms_cred = ET.parse('/home/cartman/Dev/smsCredentials.xml')
        account_sid = sms_cred.findall('account_sid')[0].get('value')
        auth_token = sms_cred.findall('auth_token')[0].get('value')
        self.sms_client = Client(account_sid, auth_token)
        self.sms_from = sms_cred.findall('from')[0].get('value')
        self.sms_to = sms_cred.findall('to')[0].get('value')
        """
        self.msgToSend = UInt16MultiArray()
        dimInfo = MultiArrayDimension()
        dimInfo.size = 2
        dimInfo.stride = 1
        self.msgToSend.layout.dim.append(dimInfo)
        self.msgToSend.layout.data_offset = 0
        self.msgToSend.data = [0, 0]
        self.statusFlag = 0
        self.digitalPorts = 0
        self.analogPorts = [0, 0, 0, 0, 0, 0]
        initialState = rospy.wait_for_message("gpio_get", UInt16MultiArray)
        self.decode(initialState)
        self.gpioSet = rospy.Publisher("gpio_set", UInt16MultiArray, queue_size = 1)
        self.gpioGet = rospy.Subscriber("gpio_get", UInt16MultiArray, self.decode)
        #try:
        #    thread.start_new_thread( self.readSerial )
        #except:
        #    print "Error: unable to start Serial Listener thread"

    def decode(self, msg):
        self.statusFlag = msg.data[0]
        self.digitalPorts = msg.data[1]
        self.analogPorts = msg.data[2:8]

    def initialize(self):
        self.msgToSend.data[0] = 0
        self.msgToSend.data[1] = self.digitalPorts
        self.gpioSet.publish(self.msgToSend)
        
    def pause(self):
        self.msgToSend.data[0] = (self.statusFlag & 0x7FF8)+5
        self.msgToSend.data[1] = self.digitalPorts
        self.gpioSet.publish(self.msgToSend)
    
    def resume(self):
        self.msgToSend.data[0] = (self.statusFlag & 0x7FF8)+3
        self.msgToSend.data[1] = self.digitalPorts
        self.gpioSet.publish(self.msgToSend)
        
    def setDoneStatus(self):
        self.msgToSend.data[0] = self.statusFlag & 0x7FF8
        self.msgToSend.data[1] = self.digitalPorts
        self.gpioSet.publish(self.msgToSend)
        
    def startCharging(self):
        self.msgToSend.data[0] = (self.statusFlag & 0xFFE7)+8
        self.msgToSend.data[1] = self.digitalPorts
        self.gpioSet.publish(self.msgToSend)
    
    def stopCharging(self):
        self.msgToSend.data[0] = self.statusFlag & 0xFFE7
        self.msgToSend.data[1] = self.digitalPorts
        self.gpioSet.publish(self.msgToSend)
    
    def isReady(self):
        return (self.statusFlag & 0x8007 == 0)
        
    def isRunning(self):
        return (self.statusFlag & 0x8007 == 3)
        
    def isPaused(self):
        return (self.statusFlag & 0x8007 == 5)
    
    def isCharging(self):
        return (self.statusFlag & 0x08)
        
    def isChargingFault(self):
        return (self.statusFlag & 0x10)
        
    def getMeas(self, arg):
        return self.analogPorts[arg]

    def sendSMS(self, text):
        """
        message = self.sms_client.messages \
                      .create(
                            body=text,
                            from_=self.sms_from, # this is my twilio number
                            to=self.sms_to # this is my number
                      )
        """
        print('message sent!')

if __name__ == '__main__':
    rospy.init_node('genericPayload', anonymous=True)
    payload = genericPayload()
    payload.initialize()
