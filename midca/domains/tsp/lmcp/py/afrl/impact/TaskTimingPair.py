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



class TaskTimingPair(LMCPObject.LMCPObject):

    def __init__(self):

        self.LMCP_TYPE = 11
        self.SERIES_NAME = "IMPACT"
        self.FULL_LMCP_TYPE_NAME = "afrl.impact.TaskTimingPair"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 5281966179208134656
        self.SERIES_VERSION = 14

        #Define message fields
        self.VehicleID = 0   #int64
        self.InitialTaskID = 0   #int64
        self.InitialTaskPercentage = 0   #real32
        self.DestinationTaskID = 0   #int64
        self.TimeToGo = 0   #int64


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(LMCPObject.LMCPObject.pack(self))
        buffer.extend(struct.pack(">q", self.VehicleID))
        buffer.extend(struct.pack(">q", self.InitialTaskID))
        buffer.extend(struct.pack(">f", self.InitialTaskPercentage))
        buffer.extend(struct.pack(">q", self.DestinationTaskID))
        buffer.extend(struct.pack(">q", self.TimeToGo))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = LMCPObject.LMCPObject.unpack(self, buffer, _pos)
        self.VehicleID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.InitialTaskID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.InitialTaskPercentage = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.DestinationTaskID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.TimeToGo = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        LMCPObject.LMCPObject.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "VehicleID" and len(e.childNodes) > 0 :
                    self.VehicleID = int(e.childNodes[0].nodeValue)
                elif e.localName == "InitialTaskID" and len(e.childNodes) > 0 :
                    self.InitialTaskID = int(e.childNodes[0].nodeValue)
                elif e.localName == "InitialTaskPercentage" and len(e.childNodes) > 0 :
                    self.InitialTaskPercentage = float(e.childNodes[0].nodeValue)
                elif e.localName == "DestinationTaskID" and len(e.childNodes) > 0 :
                    self.DestinationTaskID = int(e.childNodes[0].nodeValue)
                elif e.localName == "TimeToGo" and len(e.childNodes) > 0 :
                    self.TimeToGo = int(e.childNodes[0].nodeValue)

        return

    def unpackFromDict(self, d, seriesFactory):
        LMCPObject.LMCPObject.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "VehicleID":
                self.VehicleID = d[key]
            elif key == "InitialTaskID":
                self.InitialTaskID = d[key]
            elif key == "InitialTaskPercentage":
                self.InitialTaskPercentage = d[key]
            elif key == "DestinationTaskID":
                self.DestinationTaskID = d[key]
            elif key == "TimeToGo":
                self.TimeToGo = d[key]

        return

    def get_VehicleID(self):
        return self.VehicleID

    def set_VehicleID(self, value):
        self.VehicleID = int( value )

    def get_InitialTaskID(self):
        return self.InitialTaskID

    def set_InitialTaskID(self, value):
        self.InitialTaskID = int( value )

    def get_InitialTaskPercentage(self):
        return self.InitialTaskPercentage

    def set_InitialTaskPercentage(self, value):
        self.InitialTaskPercentage = float( value )

    def get_DestinationTaskID(self):
        return self.DestinationTaskID

    def set_DestinationTaskID(self, value):
        self.DestinationTaskID = int( value )

    def get_TimeToGo(self):
        return self.TimeToGo

    def set_TimeToGo(self, value):
        self.TimeToGo = int( value )



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = LMCPObject.LMCPObject.toString(self)
        buf += "From TaskTimingPair:\n"
        buf +=    "VehicleID = " + str( self.VehicleID ) + "\n" 
        buf +=    "InitialTaskID = " + str( self.InitialTaskID ) + "\n" 
        buf +=    "InitialTaskPercentage = " + str( self.InitialTaskPercentage ) + "\n" 
        buf +=    "DestinationTaskID = " + str( self.DestinationTaskID ) + "\n" 
        buf +=    "TimeToGo = " + str( self.TimeToGo ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "IMPACT") or (len("IMPACT") == 0): # this should never happen
        	# Checks for "IMPACT" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/TaskTimingPair")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("IMPACT" + "/TaskTimingPair")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        LMCPObject.LMCPObject.toDictMembers(self, d)
        d['VehicleID'] = self.VehicleID
        d['InitialTaskID'] = self.InitialTaskID
        d['InitialTaskPercentage'] = self.InitialTaskPercentage
        d['DestinationTaskID'] = self.DestinationTaskID
        d['TimeToGo'] = self.TimeToGo

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
        str = ws + '<TaskTimingPair Series="IMPACT" >\n';
        #str +=LMCPObject.LMCPObject.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</TaskTimingPair>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += LMCPObject.LMCPObject.toXMLMembersStr(self, ws)
        buf += ws + "<VehicleID>" + str(self.VehicleID) + "</VehicleID>\n"
        buf += ws + "<InitialTaskID>" + str(self.InitialTaskID) + "</InitialTaskID>\n"
        buf += ws + "<InitialTaskPercentage>" + str(self.InitialTaskPercentage) + "</InitialTaskPercentage>\n"
        buf += ws + "<DestinationTaskID>" + str(self.DestinationTaskID) + "</DestinationTaskID>\n"
        buf += ws + "<TimeToGo>" + str(self.TimeToGo) + "</TimeToGo>\n"

        return buf
        
