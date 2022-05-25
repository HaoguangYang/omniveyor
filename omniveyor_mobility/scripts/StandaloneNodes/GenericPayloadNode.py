#!/usr/bin/env python2

import rospy
from omniveyor_common.srv import functionCall, functionCallResponse
from std_msgs.msg import UInt16MultiArray, MultiArrayDimension, String
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
        initialState = rospy.wait_for_message("gpio/get", UInt16MultiArray)
        self.decode(initialState)
        self.gpioSet = rospy.Publisher("gpio/set", UInt16MultiArray, queue_size = 1)
        self.gpioGet = rospy.Subscriber("gpio/get", UInt16MultiArray, self.decode)
        self.managementSrv = rospy.Service("gpio/management", functionCall, self.srvCb)
        #try:
        #    thread.start_new_thread( self.readSerial )
        #except:
        #    print "Error: unable to start Serial Listener thread"
        self.funcOptions = {
                'pause' : self.pause,
                'resume' : self.resume,
                'setDoneStatus' : self.setDoneStatus,
                'startCharging' : self.startCharging,
                'stopCharging' : self.stopCharging,
                'isReady' : self.isReady,
                'isRunning' : self.isRunning,
                'isPaused' : self.isPaused,
                'isCharging' : self.isCharging,
                'isChargingFault' : self.isChargingFault,
                'getMeas' : self.getMeas,
                'sendSMS' : self.sendSMS
            }

    def decode(self, msg):
        self.statusFlag = msg.data[0]
        self.digitalPorts = msg.data[1]
        self.analogPorts = msg.data[2:8]

    def initialize(self):
        self.msgToSend.data[0] = 0
        self.msgToSend.data[1] = self.digitalPorts
        self.gpioSet.publish(self.msgToSend)

    def srvCb(self, req):
        if req.func not in self.funcOptions:
            return functionCallResponse("ERROR: Command Not Found")
        return functionCallResponse(self.funcOptions[req.func](req.param))
        
    def pause(self):
        self.msgToSend.data[0] = (self.statusFlag & 0x7FF8)+5
        self.msgToSend.data[1] = self.digitalPorts
        self.gpioSet.publish(self.msgToSend)
        return 'INFO: OK'
    
    def resume(self):
        self.msgToSend.data[0] = (self.statusFlag & 0x7FF8)+3
        self.msgToSend.data[1] = self.digitalPorts
        self.gpioSet.publish(self.msgToSend)
        return 'INFO: OK'
        
    def setDoneStatus(self):
        self.msgToSend.data[0] = self.statusFlag & 0x7FF8
        self.msgToSend.data[1] = self.digitalPorts
        self.gpioSet.publish(self.msgToSend)
        return 'INFO: OK'
        
    def startCharging(self):
        self.msgToSend.data[0] = (self.statusFlag & 0xFFE7)+8
        self.msgToSend.data[1] = self.digitalPorts
        self.gpioSet.publish(self.msgToSend)
        return 'INFO: OK'
    
    def stopCharging(self):
        self.msgToSend.data[0] = self.statusFlag & 0xFFE7
        self.msgToSend.data[1] = self.digitalPorts
        self.gpioSet.publish(self.msgToSend)
        return 'INFO: OK'
    
    def isReady(self):
        return 'INFO: ' + str(self.statusFlag & 0x8007 == 0)
        
    def isRunning(self):
        return 'INFO ' + str(self.statusFlag & 0x8007 == 3)
        
    def isPaused(self):
        return 'INFO ' + str(self.statusFlag & 0x8007 == 5)
    
    def isCharging(self):
        return 'INFO ' + str(self.statusFlag & 0x08)
        
    def isChargingFault(self):
        return 'INFO ' + str(self.statusFlag & 0x10)
        
    def getMeas(self, *args, **kwargs):
        ans = 'INFO: '
        for arg in args:
            if arg < len(self.analogPorts):
                ans += str(self.analogPorts[arg]) + ','
            else:
                ans += '-1,'
        return ans[0:-2]

    def sendSMS(self, text):
        """
        message = self.sms_client.messages \
                      .create(
                            body=text,
                            from_=self.sms_from, # this is my twilio number
                            to=self.sms_to # this is my number
                      )
        """
        return 'INFO: Message Sent!'

if __name__ == '__main__':
    rospy.init_node('genericPayload', anonymous=True)
    payload = genericPayload()
    payload.initialize()
    rospy.spin()
