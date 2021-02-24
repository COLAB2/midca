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



class TaskOptionCost(LMCPObject.LMCPObject):

    def __init__(self):

        self.LMCP_TYPE = 17
        self.SERIES_NAME = "UXTASK"
        self.FULL_LMCP_TYPE_NAME = "uxas.messages.task.TaskOptionCost"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 6149757930721443840
        self.SERIES_VERSION = 8

        #Define message fields
        self.VehicleID = 0   #int64
        self.IntialTaskID = 0   #int64
        self.IntialTaskOption = 0   #int64
        self.DestinationTaskID = 0   #int64
        self.DestinationTaskOption = 0   #int64
        self.TimeToGo = 0   #int64


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(LMCPObject.LMCPObject.pack(self))
        buffer.extend(struct.pack(">q", self.VehicleID))
        buffer.extend(struct.pack(">q", self.IntialTaskID))
        buffer.extend(struct.pack(">q", self.IntialTaskOption))
        buffer.extend(struct.pack(">q", self.DestinationTaskID))
        buffer.extend(struct.pack(">q", self.DestinationTaskOption))
        buffer.extend(struct.pack(">q", self.TimeToGo))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = LMCPObject.LMCPObject.unpack(self, buffer, _pos)
        self.VehicleID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.IntialTaskID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.IntialTaskOption = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.DestinationTaskID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.DestinationTaskOption = struct.unpack_from(">q", buffer, _pos)[0]
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
                elif e.localName == "IntialTaskID" and len(e.childNodes) > 0 :
                    self.IntialTaskID = int(e.childNodes[0].nodeValue)
                elif e.localName == "IntialTaskOption" and len(e.childNodes) > 0 :
                    self.IntialTaskOption = int(e.childNodes[0].nodeValue)
                elif e.localName == "DestinationTaskID" and len(e.childNodes) > 0 :
                    self.DestinationTaskID = int(e.childNodes[0].nodeValue)
                elif e.localName == "DestinationTaskOption" and len(e.childNodes) > 0 :
                    self.DestinationTaskOption = int(e.childNodes[0].nodeValue)
                elif e.localName == "TimeToGo" and len(e.childNodes) > 0 :
                    self.TimeToGo = int(e.childNodes[0].nodeValue)

        return

    def unpackFromDict(self, d, seriesFactory):
        LMCPObject.LMCPObject.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "VehicleID":
                self.VehicleID = d[key]
            elif key == "IntialTaskID":
                self.IntialTaskID = d[key]
            elif key == "IntialTaskOption":
                self.IntialTaskOption = d[key]
            elif key == "DestinationTaskID":
                self.DestinationTaskID = d[key]
            elif key == "DestinationTaskOption":
                self.DestinationTaskOption = d[key]
            elif key == "TimeToGo":
                self.TimeToGo = d[key]

        return

    def get_VehicleID(self):
        return self.VehicleID

    def set_VehicleID(self, value):
        self.VehicleID = int( value )

    def get_IntialTaskID(self):
        return self.IntialTaskID

    def set_IntialTaskID(self, value):
        self.IntialTaskID = int( value )

    def get_IntialTaskOption(self):
        return self.IntialTaskOption

    def set_IntialTaskOption(self, value):
        self.IntialTaskOption = int( value )

    def get_DestinationTaskID(self):
        return self.DestinationTaskID

    def set_DestinationTaskID(self, value):
        self.DestinationTaskID = int( value )

    def get_DestinationTaskOption(self):
        return self.DestinationTaskOption

    def set_DestinationTaskOption(self, value):
        self.DestinationTaskOption = int( value )

    def get_TimeToGo(self):
        return self.TimeToGo

    def set_TimeToGo(self, value):
        self.TimeToGo = int( value )



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = LMCPObject.LMCPObject.toString(self)
        buf += "From TaskOptionCost:\n"
        buf +=    "VehicleID = " + str( self.VehicleID ) + "\n" 
        buf +=    "IntialTaskID = " + str( self.IntialTaskID ) + "\n" 
        buf +=    "IntialTaskOption = " + str( self.IntialTaskOption ) + "\n" 
        buf +=    "DestinationTaskID = " + str( self.DestinationTaskID ) + "\n" 
        buf +=    "DestinationTaskOption = " + str( self.DestinationTaskOption ) + "\n" 
        buf +=    "TimeToGo = " + str( self.TimeToGo ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "UXTASK") or (len("UXTASK") == 0): # this should never happen
        	# Checks for "UXTASK" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/TaskOptionCost")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("UXTASK" + "/TaskOptionCost")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        LMCPObject.LMCPObject.toDictMembers(self, d)
        d['VehicleID'] = self.VehicleID
        d['IntialTaskID'] = self.IntialTaskID
        d['IntialTaskOption'] = self.IntialTaskOption
        d['DestinationTaskID'] = self.DestinationTaskID
        d['DestinationTaskOption'] = self.DestinationTaskOption
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
        str = ws + '<TaskOptionCost Series="UXTASK" >\n';
        #str +=LMCPObject.LMCPObject.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</TaskOptionCost>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += LMCPObject.LMCPObject.toXMLMembersStr(self, ws)
        buf += ws + "<VehicleID>" + str(self.VehicleID) + "</VehicleID>\n"
        buf += ws + "<IntialTaskID>" + str(self.IntialTaskID) + "</IntialTaskID>\n"
        buf += ws + "<IntialTaskOption>" + str(self.IntialTaskOption) + "</IntialTaskOption>\n"
        buf += ws + "<DestinationTaskID>" + str(self.DestinationTaskID) + "</DestinationTaskID>\n"
        buf += ws + "<DestinationTaskOption>" + str(self.DestinationTaskOption) + "</DestinationTaskOption>\n"
        buf += ws + "<TimeToGo>" + str(self.TimeToGo) + "</TimeToGo>\n"

        return buf
        
