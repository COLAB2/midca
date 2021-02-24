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

from midca.domains.tsp.lmcp.py.afrl.cmasi import VehicleAction


class VideoStreamAction(VehicleAction.VehicleAction):

    def __init__(self):
        VehicleAction.VehicleAction.__init__(self)
        self.LMCP_TYPE = 48
        self.SERIES_NAME = "CMASI"
        self.FULL_LMCP_TYPE_NAME = "afrl.cmasi.VideoStreamAction"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 4849604199710720000
        self.SERIES_VERSION = 3

        #Define message fields
        self.VideoStreamID = 0   #int32
        self.ActiveSensor = 0   #int32


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(VehicleAction.VehicleAction.pack(self))
        buffer.extend(struct.pack(">i", self.VideoStreamID))
        buffer.extend(struct.pack(">i", self.ActiveSensor))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = VehicleAction.VehicleAction.unpack(self, buffer, _pos)
        self.VideoStreamID = struct.unpack_from(">i", buffer, _pos)[0]
        _pos += 4
        self.ActiveSensor = struct.unpack_from(">i", buffer, _pos)[0]
        _pos += 4
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        VehicleAction.VehicleAction.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "VideoStreamID" and len(e.childNodes) > 0 :
                    self.VideoStreamID = int(e.childNodes[0].nodeValue)
                elif e.localName == "ActiveSensor" and len(e.childNodes) > 0 :
                    self.ActiveSensor = int(e.childNodes[0].nodeValue)

        return

    def unpackFromDict(self, d, seriesFactory):
        VehicleAction.VehicleAction.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "VideoStreamID":
                self.VideoStreamID = d[key]
            elif key == "ActiveSensor":
                self.ActiveSensor = d[key]

        return

    def get_VideoStreamID(self):
        return self.VideoStreamID

    def set_VideoStreamID(self, value):
        self.VideoStreamID = int( value )

    def get_ActiveSensor(self):
        return self.ActiveSensor

    def set_ActiveSensor(self, value):
        self.ActiveSensor = int( value )



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = VehicleAction.VehicleAction.toString(self)
        buf += "From VideoStreamAction:\n"
        buf +=    "VideoStreamID = " + str( self.VideoStreamID ) + "\n" 
        buf +=    "ActiveSensor = " + str( self.ActiveSensor ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "CMASI") or (len("CMASI") == 0): # this should never happen
        	# Checks for "CMASI" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/VideoStreamAction")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("CMASI" + "/VideoStreamAction")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        VehicleAction.VehicleAction.toDictMembers(self, d)
        d['VideoStreamID'] = self.VideoStreamID
        d['ActiveSensor'] = self.ActiveSensor

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
        str = ws + '<VideoStreamAction Series="CMASI" >\n';
        #str +=VehicleAction.VehicleAction.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</VideoStreamAction>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += VehicleAction.VehicleAction.toXMLMembersStr(self, ws)
        buf += ws + "<VideoStreamID>" + str(self.VideoStreamID) + "</VideoStreamID>\n"
        buf += ws + "<ActiveSensor>" + str(self.ActiveSensor) + "</ActiveSensor>\n"

        return buf
        
