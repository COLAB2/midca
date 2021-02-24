#! /usr/bin/python

import sys, struct
import xml.dom.minidom
from midca.domains.tsp.lmcp.py.lmcp import LMCPObject

## ===============================================================================
## Authors: AFRL/RQQA
## Organization: Air Force Research Laboratory, Aerospace Systems Directorate, Power and Control Division
## 
## Copyright (c) 2017 Government of the United State of America, as represented by
## the Secretary of the Air Force.  No copyright is claimed in the United States under
## Title 17, U.S. Code.  All Other Rights Reserved.
## ===============================================================================

## This file was auto-created by LmcpGen. Modifications will be overwritten.



class TaskAssignment(LMCPObject.LMCPObject):

    def __init__(self):

        self.LMCP_TYPE = 18
        self.SERIES_NAME = "UXTASK"
        self.FULL_LMCP_TYPE_NAME = "uxas.messages.task.TaskAssignment"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 6149757930721443840
        self.SERIES_VERSION = 8

        #Define message fields
        self.TaskID = 0   #int64
        self.OptionID = 0   #int64
        self.AssignedVehicle = 0   #int64
        self.TimeThreshold = 0   #int64
        self.TimeTaskCompleted = 0   #int64


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(LMCPObject.LMCPObject.pack(self))
        buffer.extend(struct.pack(">q", self.TaskID))
        buffer.extend(struct.pack(">q", self.OptionID))
        buffer.extend(struct.pack(">q", self.AssignedVehicle))
        buffer.extend(struct.pack(">q", self.TimeThreshold))
        buffer.extend(struct.pack(">q", self.TimeTaskCompleted))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = LMCPObject.LMCPObject.unpack(self, buffer, _pos)
        self.TaskID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.OptionID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.AssignedVehicle = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.TimeThreshold = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.TimeTaskCompleted = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        LMCPObject.LMCPObject.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "TaskID" and len(e.childNodes) > 0 :
                    self.TaskID = int(e.childNodes[0].nodeValue)
                elif e.localName == "OptionID" and len(e.childNodes) > 0 :
                    self.OptionID = int(e.childNodes[0].nodeValue)
                elif e.localName == "AssignedVehicle" and len(e.childNodes) > 0 :
                    self.AssignedVehicle = int(e.childNodes[0].nodeValue)
                elif e.localName == "TimeThreshold" and len(e.childNodes) > 0 :
                    self.TimeThreshold = int(e.childNodes[0].nodeValue)
                elif e.localName == "TimeTaskCompleted" and len(e.childNodes) > 0 :
                    self.TimeTaskCompleted = int(e.childNodes[0].nodeValue)

        return

    def unpackFromDict(self, d, seriesFactory):
        LMCPObject.LMCPObject.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "TaskID":
                self.TaskID = d[key]
            elif key == "OptionID":
                self.OptionID = d[key]
            elif key == "AssignedVehicle":
                self.AssignedVehicle = d[key]
            elif key == "TimeThreshold":
                self.TimeThreshold = d[key]
            elif key == "TimeTaskCompleted":
                self.TimeTaskCompleted = d[key]

        return

    def get_TaskID(self):
        return self.TaskID

    def set_TaskID(self, value):
        self.TaskID = int( value )

    def get_OptionID(self):
        return self.OptionID

    def set_OptionID(self, value):
        self.OptionID = int( value )

    def get_AssignedVehicle(self):
        return self.AssignedVehicle

    def set_AssignedVehicle(self, value):
        self.AssignedVehicle = int( value )

    def get_TimeThreshold(self):
        return self.TimeThreshold

    def set_TimeThreshold(self, value):
        self.TimeThreshold = int( value )

    def get_TimeTaskCompleted(self):
        return self.TimeTaskCompleted

    def set_TimeTaskCompleted(self, value):
        self.TimeTaskCompleted = int( value )



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = LMCPObject.LMCPObject.toString(self)
        buf += "From TaskAssignment:\n"
        buf +=    "TaskID = " + str( self.TaskID ) + "\n" 
        buf +=    "OptionID = " + str( self.OptionID ) + "\n" 
        buf +=    "AssignedVehicle = " + str( self.AssignedVehicle ) + "\n" 
        buf +=    "TimeThreshold = " + str( self.TimeThreshold ) + "\n" 
        buf +=    "TimeTaskCompleted = " + str( self.TimeTaskCompleted ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "UXTASK") or (len("UXTASK") == 0): # this should never happen
        	# Checks for "UXTASK" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/TaskAssignment")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("UXTASK" + "/TaskAssignment")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        LMCPObject.LMCPObject.toDictMembers(self, d)
        d['TaskID'] = self.TaskID
        d['OptionID'] = self.OptionID
        d['AssignedVehicle'] = self.AssignedVehicle
        d['TimeThreshold'] = self.TimeThreshold
        d['TimeTaskCompleted'] = self.TimeTaskCompleted

        return

    def getLMCPType(self):
        return self.LMCP_TYPE

    def getSeriesName(self):
        return self.SERIES_NAME

    def getSeriesNameID(self):
        return self.SERIES_NAME_ID

    def getSeriesVersion(self):
        return self.SERIES_VERSION

    def toXMLStr(self, ws):
        str = ws + '<TaskAssignment Series="UXTASK" >\n';
        #str +=LMCPObject.LMCPObject.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</TaskAssignment>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += LMCPObject.LMCPObject.toXMLMembersStr(self, ws)
        buf += ws + "<TaskID>" + str(self.TaskID) + "</TaskID>\n"
        buf += ws + "<OptionID>" + str(self.OptionID) + "</OptionID>\n"
        buf += ws + "<AssignedVehicle>" + str(self.AssignedVehicle) + "</AssignedVehicle>\n"
        buf += ws + "<TimeThreshold>" + str(self.TimeThreshold) + "</TimeThreshold>\n"
        buf += ws + "<TimeTaskCompleted>" + str(self.TimeTaskCompleted) + "</TimeTaskCompleted>\n"

        return buf
        
