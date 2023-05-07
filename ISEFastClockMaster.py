# Send Fastclock data to ISE Clocks and ProtoThrottles
#
# This code is a reimplementation of Iowa Scaled Engineering's Fast Clock
# master code that uses JMRI's fast clock (which may be controlled by 
# clocks from other systems) to drive the fast clock on a ProtoThrottle or
# ISE's Fast Clock Slaves.
#
# Dependencies:  This code uses an XBee connection configured in JMRI. See 
# JMRI's documentation for configuring that connection.
#
# References:
# This code is based on code in the following ISE repositories
#    esu-bridge: https://github.com/IowaScaledEngineering/esu-bridge.git
#    mrb-fcm: https://github.com/IowaScaledEngineering/mrb-fcm.git
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
import java.lang
import java.beans
import jmri
import jarray

#First, define a listner for the timebase.
class ClockListener(java.beans.PropertyChangeListener):
   
   def __init__(self,mrbus_dev_addr):
      self.mrbus_dev_addr = mrbus_dev_addr
      # find the XBee Module
      self.cm = jmri.InstanceManager.getDefault(jmri.jmrix.ieee802154.xbee.XBeeConnectionMemo)
      self.xbee = self.cm.getTrafficController().getXBee() #This is the XBee connected to the computer.
      self.timebase=jmri.InstanceManager.getDefault(jmri.Timebase)
   
   def propertyChange(self,event):
      print "property", event.propertyName,"new value",event.newValue
      self.sendCurrentTime()
      return

   def sendCurrentTime(self):
      flags = 0
      scaleFactor = int(round(self.timebase.getRate()*10))
      txBuffer = []
      dateTime=self.timebase.getTime()
      date = dateTime
      
      if (self.timebase.getRun()):
         flags |= 0x01  #DISP_FAST
      txBuffer.append(java.lang.Byte(0xff).byteValue()) #destination
      txBuffer.append(java.lang.Byte(0xFF & self.mrbus_dev_addr).byteValue()) #source
      txBuffer.append(java.lang.Byte(0xFF & 18).byteValue()) #length
      txBuffer.append(java.lang.Byte(0).byteValue()) #checksum L
      txBuffer.append(java.lang.Byte(0).byteValue()) #checksum H
      txBuffer.append(java.lang.Byte(0x54).byteValue())  #ascii for T
      txBuffer.append(java.lang.Byte(0).byteValue()) #real hours
      txBuffer.append(java.lang.Byte(0).byteValue()) #real minutes
      txBuffer.append(java.lang.Byte(0).byteValue()) #real seconds
      txBuffer.append(java.lang.Byte(0xFF & flags).byteValue())
      txBuffer.append(java.lang.Byte(0xFF & dateTime.getHours()).byteValue()) #fast
      txBuffer.append(java.lang.Byte(0xFF & dateTime.getMinutes()).byteValue())
      txBuffer.append(java.lang.Byte(0xFF & dateTime.getSeconds()).byteValue())
      txBuffer.append(java.lang.Byte(0xFF & (scaleFactor>>8)).byteValue())
      txBuffer.append(java.lang.Byte(0xFF & scaleFactor).byteValue())
      txBuffer.append(java.lang.Byte(0).byteValue()) # part of real year.
      txBuffer.append(java.lang.Byte(0).byteValue()) # rest of real year and month
      txBuffer.append(java.lang.Byte(0).byteValue()) #day of real month
      crc = self.mrbusCRC16Calculate(txBuffer)
      txBuffer[3] = java.lang.Byte(0xFF & crc).byteValue()
      txBuffer[4] = java.lang.Byte(0xFF & (crc >> 8)).byteValue()
      try:
         self.xbee.sendBroadcastData(txBuffer)
         # print 'status message sent'
      except:
         print 'failed to send time update'
         pass
      return
   
   #CRC Calculations from esu-bridge
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


#setup the required instance variables
mrbus_dev_addr = 0
timebase=jmri.InstanceManager.getDefault(jmri.Timebase)
timebase.addMinuteChangeListener(ClockListener(mrbus_dev_addr));
