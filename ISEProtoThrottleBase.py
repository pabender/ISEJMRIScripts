# Act as a base for ISE ProtoThrottles
#
# This code is a reimplementation of Iowa Scaled Engineering's ESU-Bridge
# that can be run as a script directly in JMRI.
#
# Dependencies:  This code uses an XBee connection configured in JMRI. See 
# JMRI's documentation for configuring that connection.
#
# References:
# This code is based on code in the following ISE repositories
#    esu-bridge: https://github.com/IowaScaledEngineering/esu-bridge.git
#    mrbw-cabbus: https://github.com/IowaScaledEngineering/mrbw-cabbus.git
# And the author's own contributions to JMRI.
#
# Author: Paul Bender, Copyright 2021
#
#    This program is free software; you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation; either version 3 of the License, or
#    any later version.
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.

import java
import java.util
import jarray
import jmri
import com.digi.xbee

# The packet class handles translation of data from the received packet data.
class packet:
  def __init__(self, dest, src, cmd, data):
    self.dest=dest&0xFF
    self.src=src&0xFF
    self.cmd=cmd&0xFF
    self.data=data

  def __hash__(self):
    return hash(self.__repr__())

  def __eq__(self, other):
    return self.__repr__() == other.__repr__()

  def __repr__(self):
    return "mrbus.packet(0x%02x, 0x%02x, 0x%02x, %s)"%(self.dest, self.src, self.cmd, repr(self.data))

  def __str__(self):
    c='(%02xh'%self.cmd
    if self.cmd >= 32 and self.cmd <= 127:
      c+=" '%c')"%self.cmd
    else:
      c+="    )"
    return "packet(%02xh->%02xh) %s %2d:%s"%(self.src, self.dest, c, len(self.data), ["0x%02X"%d for d in self.data])
  
  def address(self):
      if self.longAddress():
         return (0xFF & self.data[0]) * 256 + (0xFF & self.data[1])
      return self.data[1] & 0x7F  
  
  def longAddress(self):
      if 0 != ( self.data[0] & 0x80):
         return False
      else:
         return True
  
  def speed(self):
      return self.data[2] & 0x7F
  
  def direction(self):
      return (0 != (self.data[2] & 0x80))
  
  def functions(self):
      functions = [ 0,0,0,0,0,0,0,0,0,0,
                       0,0,0,0,0,0,0,0,0,0,
                       0,0,0,0,0,0,0,0,0 ]
      
      for i in range(29):
         if i >= 0 and i < 8:
            if (self.data[6] & 0xFF) & (1<<i):
               functions[i] = 1 
         elif i >= 8 and i < 16:
            if (self.data[5] & 0xFF) & (1<<(i-8)):
               functions[i] = 1
         elif i >= 16 and i < 24:
            if (self.data[4] & 0xFF) & (1<<(i-16)):
               functions[i] = 1
         elif i >= 24 and i < 29:
            if (self.data[3] & 0xFF) & (1<<(i-24)):
               functions[i] = 1
      return functions

# The MRBusThrottle class handles the translation between recieved packet 
# data and the JMRI throttles.
class MRBusThrottle(jmri.jmrit.automat.AbstractAutomaton):
   
   def __init__(self, addr):
      print "Creating MRBusThrottle for 0x%02X" % (addr)
      self.locAddr = -1
      self.locAddrLong = True
      self.locSpeed = 0
      self.throttle = 0
      self.locEStop = 0
      self.throttleAddr = addr
      return
   
   def init(self):
      return
   
   def handle(self):
      return 0 # don't repeat.
      
   def update(self, pkt):
      print "MRBusThrottle (0x%02X): UPDATE loco %d" % (self.throttleAddr, pkt.address())
      
      if (pkt.address() != self.locAddr):
         self.locAddr = pkt.address()
         self.locAddrLong = pkt.longAddress()
         self.throttle = self.getThrottle(self.locAddr,self.locAddrLong)
         print "MRBusThrottle (0x%02X): Acquiring new locomotive %d" % (self.throttleAddr, self.locAddr)
      
      self.handleSpeedAndDirection(pkt)
      self.handleFunctions(pkt)
      
      return
   
   def handleFunctions(self,pkt):
      functions = pkt.functions()
      for i in range(29):
         if (functions[i]==1 and not self.throttle.getFunction(i)) or (not functions[i]==1 and self.throttle.getFunction(i)):
            self.setFunction(i,functions[i]==1)
      return
   
   def setFunction(self,functionNo,value):
      print "MRBusThrottle (0x%02X): Set loco [%d] function [%d] to [%d]" % (self.throttleAddr, self.locAddr, functionNo, value)
      self.throttle.setFunction(functionNo,value)
      return


   def handleSpeedAndDirection(self,pkt):
      speed = pkt.speed()
      if 1 == speed:
         speed = 0
         estop = 1
      elif speed > 1:
         estop = 0
         speed = speed - 1
      elif speed == 0:
         estop = 0
      
      direction = pkt.direction();
      
      # Only send ESTOP if we just moved into that state
      if estop is 1:
         if not (estop is self.locEStop):
            self.setEStop()
      else:
         if (direction and not self.throttle.getIsForward()) or (not direction and self.throttle.getIsForward()):
            self.setDirection(direction)
         if speed != self.locSpeed:
             self.setSpeed(speed)
      
      self.locEStop = estop
      self.locSpeed = speed
      return
   
   def setEStop(self):
      print "MRBusThrottle (0x%02X): Set ESTOP loco %d" % (self.throttleAddr, self.locAddr)
      self.throttle.setSpeedSetting(-1.0)
      return
   
   def setSpeed(self,speed):
      print "MRBusThrottle (0x%02X): Set loco [%d] speed %d" % (self.throttleAddr, self.locAddr, speed)
      self.throttle.setSpeedSetting(speed/126.0)
      return

   def setDirection(self,direction):
      print "MRBusThrottle (0x%02X): Set loco [%d] direction %s" % (self.throttleAddr, self.locAddr, ["REV","FWD"][direction])
      self.throttle.setIsForward(direction)
      return

# The ProtoThrottleListener class receives data from the XBee network and 
# passes that data on to instances of the MRBusThrottle class.
class ProtoThrottleListener(com.digi.xbee.api.listeners.IDataReceiveListener):
      
   def __init__(self,mrbus_dev_addr):
       self.mrbus_dev_addr = mrbus_dev_addr
       self.throttles = java.util.HashMap()
   
   def dataReceived(self,message):
      print "data: ",message.getData()
      data = message.getData()
      pkt = packet(data[0]&0xFF,data[1]&0xFF,data[5]&0xFF,data[6:])
      print "packet received ",pkt
      
      if pkt is None:
         print "pkt none"
         return
      
      # Bypass anything that doesn't look like a throttle packet
      if pkt.cmd != 0x53 or len(pkt.data) != 9:
         print "packet invalid ",pkt
         return
      
      print "valid packet"
     
      throttle = self.throttles.get(pkt.src)
      if throttle is None:
         throttle = MRBusThrottle(pkt.src)
         self.throttles.put(pkt.src,throttle)
      throttle.update(pkt)

# The ProtoThrottleHeartBeat class provides a heartbeat to the ProtoThrottles.  
# This tells the ProtoThrottle that something is out there to listen to their 
# requests.
class ProtoThrottleHeartBeat(jmri.jmrit.automat.AbstractAutomaton):

   def __init__(self,xbee,mrbus_dev_addr) :
      self.mrbus_dev_addr = mrbus_dev_addr
      self.Xbee = xbee 
   
   def init(self) :
      return
   
   def handle(self) :
      self.sendStatus()
      return 1
   
   def sendStatus(self):
      txBuffer = []
      txBuffer.append(java.lang.Byte(0xff).byteValue()) #destination
      txBuffer.append(java.lang.Byte(0xFF & self.mrbus_dev_addr).byteValue()) #source
      txBuffer.append(java.lang.Byte(0xFF & 16).byteValue()) #length
      txBuffer.append(java.lang.Byte(0).byteValue()) #checksum L
      txBuffer.append(java.lang.Byte(0).byteValue()) #checksum H
      txBuffer.append(java.lang.Byte(0x76).byteValue())  #ascii for v 
      txBuffer.append(java.lang.Byte(0x80).byteValue()) 
      txBuffer.append(java.lang.Byte(0x01).byteValue()) 
      txBuffer.append(java.lang.Byte(0x00).byteValue()) 
      txBuffer.append(java.lang.Byte(0x00).byteValue())
      txBuffer.append(java.lang.Byte(0x01).byteValue())
      txBuffer.append(java.lang.Byte(0x00).byteValue())
      txBuffer.append(java.lang.Byte(0x4A).byteValue()) # J
      txBuffer.append(java.lang.Byte(0x4D).byteValue()) # M
      txBuffer.append(java.lang.Byte(0x52).byteValue()) # R
      txBuffer.append(java.lang.Byte(0x49).byteValue()) # I
      crc = self.mrbusCRC16Calculate(txBuffer)
      txBuffer[3] = java.lang.Byte(0xFF & crc).byteValue()
      txBuffer[4] = java.lang.Byte(0xFF & (crc >> 8)).byteValue()
      try:
         self.Xbee.setReceiveTimeout(2000)
         self.Xbee.sendBroadcastData(txBuffer)
         print 'status message sent'
      except:
         print 'failed to send heartbeat'
         pass
      return
   
   def mrbusCRC16Calculate(self,data):
      mrbusPktLen = data[2]
      crc = 0
      
      for i in range(0, mrbusPktLen):
         if i == 3 or i == 4:
            continue
         else:
            a = data[i]
         crc = self.mrbusCRC16Update(crc, a)
      
      return crc
   
   
   def mrbusCRC16Update(self,crc, a):
      MRBus_CRC16_HighTable = [ 0x00, 0xA0, 0xE0, 0x40, 0x60, 0xC0, 0x80, 0x20, 0xC0, 0x60, 0x20, 0x80, 0xA0, 0x00, 0x40, 0xE0 ]
      MRBus_CRC16_LowTable =  [ 0x00, 0x01, 0x03, 0x02, 0x07, 0x06, 0x04, 0x05, 0x0E, 0x0F, 0x0D, 0x0C, 0x09, 0x08, 0x0A, 0x0B ]
      crc16_h = (crc>>8) & 0xFF
      crc16_l = crc & 0xFF
      
      i = 0
      
      while i < 2:
         if i != 0:
            w = ((crc16_h << 4) & 0xF0) | ((crc16_h >> 4) & 0x0F)
            t = (w ^ a) & 0x0F
         else:
            t = (crc16_h ^ a) & 0xF0
            t = ((t << 4) & 0xF0) | ((t >> 4) & 0x0F)
         
         crc16_h = (crc16_h << 4) & 0xFF
         crc16_h = crc16_h | (crc16_l >> 4)
         crc16_l = (crc16_l << 4) & 0xFF
         
         crc16_h = crc16_h ^ MRBus_CRC16_HighTable[t]
         crc16_l = crc16_l ^ MRBus_CRC16_LowTable[t]
         
         i = i + 1
      
      return (crc16_h<<8) | crc16_l

#The ProtoThrottleDriver is an orchestrator class that brings everything else 
# together.  It starts the XBee Listener and the Heartbeat classes, which do 
# the real work.
class ProtoThrottleDriver:
   # ctor starts up the XBee Listener and Heartbeat
   def __init__(self,mrbus_dev_addr) :
      self.mrbus_dev_addr = mrbus_dev_addr
      # find the XBee Module
      self.cm = jmri.InstanceManager.getDefault(jmri.jmrix.ieee802154.xbee.XBeeConnectionMemo)
      self.tc = self.cm.getTrafficController()
      self.Xbee = self.tc.getXBee()
      self.Xbee.addDataListener(ProtoThrottleListener(self.mrbus_dev_addr))

      print "Listener Started"
      
      #Start the heartbeat
      protoThrottleHeartbeat = ProtoThrottleHeartBeat(self.Xbee,self.mrbus_dev_addr)
      protoThrottleHeartbeat.start()
      
      print "Heartbeat Started"
      
      return
   
# end of class definition

#The code that follows sets up the ProtoThrottleDriver class
mrbus_dev_addr = 0xD0 # base address.  0xD0 coresponds to a base 
                      # address of "0" on the protothrottle.
a = ProtoThrottleDriver(mrbus_dev_addr)
print "End of Script"
