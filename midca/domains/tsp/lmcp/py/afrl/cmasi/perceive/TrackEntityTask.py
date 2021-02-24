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

from midca.domains.tsp.lmcp.py.afrl.cmasi import Task
from midca.domains.tsp.lmcp.py.afrl.cmasi import WavelengthBand


class TrackEntityTask(Task.Task):

    def __init__(self):
        Task.Task.__init__(self)
        self.LMCP_TYPE = 3
        self.SERIES_NAME = "PERCEIVE"
        self.FULL_LMCP_TYPE_NAME = "afrl.cmasi.perceive.TrackEntityTask"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 5784119745305990725
        self.SERIES_VERSION = 1

        #Define message fields
        self.EntityID = 0   #uint32
        self.SensorModality = WavelengthBand.WavelengthBand.AllAny   #WavelengthBand
        self.GroundSampleDistance = 0   #real32


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(Task.Task.pack(self))
        buffer.extend(struct.pack(">I", self.EntityID))
        buffer.extend(struct.pack(">i", self.SensorModality))
        buffer.extend(struct.pack(">f", self.GroundSampleDistance))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = Task.Task.unpack(self, buffer, _pos)
        self.EntityID = struct.unpack_from(">I", buffer, _pos)[0]
        _pos += 4
        self.SensorModality = struct.unpack_from(">i", buffer, _pos)[0]
        _pos += 4
        self.GroundSampleDistance = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        Task.Task.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "EntityID" and len(e.childNodes) > 0 :
                    self.EntityID = int(e.childNodes[0].nodeValue)
                elif e.localName == "SensorModality" and len(e.childNodes) > 0 :
                    self.SensorModality = WavelengthBand.get_WavelengthBand_str(e.childNodes[0].nodeValue)
                elif e.localName == "GroundSampleDistance" and len(e.childNodes) > 0 :
                    self.GroundSampleDistance = float(e.childNodes[0].nodeValue)

        return

    def unpackFromDict(self, d, seriesFactory):
        Task.Task.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "EntityID":
                self.EntityID = d[key]
            elif key == "SensorModality":
                self.SensorModality = d[key]
            elif key == "GroundSampleDistance":
                self.GroundSampleDistance = d[key]

        return

    def get_EntityID(self):
        return self.EntityID

    def set_EntityID(self, value):
        self.EntityID = int( value )

    def get_SensorModality(self):
        return self.SensorModality

    def set_SensorModality(self, value):
        self.SensorModality = value 

    def get_GroundSampleDistance(self):
        return self.GroundSampleDistance

    def set_GroundSampleDistance(self, value):
        self.GroundSampleDistance = float( value )



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = Task.Task.toString(self)
        buf += "From TrackEntityTask:\n"
        buf +=    "EntityID = " + str( self.EntityID ) + "\n" 
        buf +=    "SensorModality = " + str( self.SensorModality ) + "\n" 
        buf +=    "GroundSampleDistance = " + str( self.GroundSampleDistance ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "PERCEIVE") or (len("PERCEIVE") == 0): # this should never happen
        	# Checks for "PERCEIVE" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/TrackEntityTask")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("PERCEIVE" + "/TrackEntityTask")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        Task.Task.toDictMembers(self, d)
        d['EntityID'] = self.EntityID
        d['SensorModality'] = self.SensorModality
        d['GroundSampleDistance'] = self.GroundSampleDistance

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
        str = ws + '<TrackEntityTask Series="PERCEIVE" >\n';
        #str +=Task.Task.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</TrackEntityTask>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += Task.Task.toXMLMembersStr(self, ws)
        buf += ws + "<EntityID>" + str(self.EntityID) + "</EntityID>\n"
        buf += ws + "<SensorModality>" + WavelengthBand.get_WavelengthBand_int(self.SensorModality) + "</SensorModality>\n"
        buf += ws + "<GroundSampleDistance>" + str(self.GroundSampleDistance) + "</GroundSampleDistance>\n"

        return buf
        
