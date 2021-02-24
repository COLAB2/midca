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

from midca.domains.tsp.lmcp.py.afrl.cmasi import VehicleActionCommand
from midca.domains.tsp.lmcp.py.afrl.cmasi import PathWaypoint
from midca.domains.tsp.lmcp.py.afrl.cmasi import TravelMode


class FollowPathCommand(VehicleActionCommand.VehicleActionCommand):

    def __init__(self):
        VehicleActionCommand.VehicleActionCommand.__init__(self)
        self.LMCP_TYPE = 56
        self.SERIES_NAME = "CMASI"
        self.FULL_LMCP_TYPE_NAME = "afrl.cmasi.FollowPathCommand"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 4849604199710720000
        self.SERIES_VERSION = 3

        #Define message fields
        self.FirstWaypoint = 0   #int64
        self.WaypointList = []   #PathWaypoint
        self.StartTime = 0   #int64
        self.StopTime = 0   #int64
        self.RepeatMode = TravelMode.TravelMode.SinglePass   #TravelMode


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(VehicleActionCommand.VehicleActionCommand.pack(self))
        buffer.extend(struct.pack(">q", self.FirstWaypoint))
        buffer.extend(struct.pack(">H", len(self.WaypointList) ))
        for x in self.WaypointList:
           buffer.extend(struct.pack("B", x != None ))
           if x != None:
               buffer.extend(struct.pack(">q", x.SERIES_NAME_ID))
               buffer.extend(struct.pack(">I", x.LMCP_TYPE))
               buffer.extend(struct.pack(">H", x.SERIES_VERSION))
               buffer.extend(x.pack())
        buffer.extend(struct.pack(">q", self.StartTime))
        buffer.extend(struct.pack(">q", self.StopTime))
        buffer.extend(struct.pack(">i", self.RepeatMode))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = VehicleActionCommand.VehicleActionCommand.unpack(self, buffer, _pos)
        self.FirstWaypoint = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.WaypointList = [None] * _arraylen
        for x in range(_arraylen):
            _valid = struct.unpack_from("B", buffer, _pos )[0]
            _pos += 1
            if _valid:
                _series = struct.unpack_from(">q", buffer, _pos)[0]
                _pos += 8
                _type = struct.unpack_from(">I", buffer, _pos)[0]
                _pos += 4
                _version = struct.unpack_from(">H", buffer, _pos)[0]
                _pos += 2
                from midca.domains.tsp.lmcp.py.lmcp import LMCPFactory
                self.WaypointList[x] = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
                _pos = self.WaypointList[x].unpack(buffer, _pos)
            else:
                self.WaypointList[x] = None
        self.StartTime = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.StopTime = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.RepeatMode = struct.unpack_from(">i", buffer, _pos)[0]
        _pos += 4
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        VehicleActionCommand.VehicleActionCommand.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "FirstWaypoint" and len(e.childNodes) > 0 :
                    self.FirstWaypoint = int(e.childNodes[0].nodeValue)
                elif e.localName == "WaypointList" and len(e.childNodes) > 0 :
                    self.WaypointList = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            obj = seriesFactory.createObjectByName(c.getAttribute('Series'), c.localName)
                            if obj != None:
                                obj.unpackFromXMLNode(c, seriesFactory)
                                self.WaypointList.append(obj)
                elif e.localName == "StartTime" and len(e.childNodes) > 0 :
                    self.StartTime = int(e.childNodes[0].nodeValue)
                elif e.localName == "StopTime" and len(e.childNodes) > 0 :
                    self.StopTime = int(e.childNodes[0].nodeValue)
                elif e.localName == "RepeatMode" and len(e.childNodes) > 0 :
                    self.RepeatMode = TravelMode.get_TravelMode_str(e.childNodes[0].nodeValue)

        return

    def unpackFromDict(self, d, seriesFactory):
        VehicleActionCommand.VehicleActionCommand.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "FirstWaypoint":
                self.FirstWaypoint = d[key]
            elif key == "WaypointList":
                self.WaypointList = []
                for c in d[key]:
                    obj = seriesFactory.unpackFromDict(c)
                    if obj != None:
                        self.WaypointList.append(obj)
            elif key == "StartTime":
                self.StartTime = d[key]
            elif key == "StopTime":
                self.StopTime = d[key]
            elif key == "RepeatMode":
                self.RepeatMode = d[key]

        return

    def get_FirstWaypoint(self):
        return self.FirstWaypoint

    def set_FirstWaypoint(self, value):
        self.FirstWaypoint = int( value )

    def get_WaypointList(self):
        return self.WaypointList

    def get_StartTime(self):
        return self.StartTime

    def set_StartTime(self, value):
        self.StartTime = int( value )

    def get_StopTime(self):
        return self.StopTime

    def set_StopTime(self, value):
        self.StopTime = int( value )

    def get_RepeatMode(self):
        return self.RepeatMode

    def set_RepeatMode(self, value):
        self.RepeatMode = value 



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = VehicleActionCommand.VehicleActionCommand.toString(self)
        buf += "From FollowPathCommand:\n"
        buf +=    "FirstWaypoint = " + str( self.FirstWaypoint ) + "\n" 
        buf +=    "WaypointList = " + str( self.WaypointList ) + "\n" 
        buf +=    "StartTime = " + str( self.StartTime ) + "\n" 
        buf +=    "StopTime = " + str( self.StopTime ) + "\n" 
        buf +=    "RepeatMode = " + str( self.RepeatMode ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "CMASI") or (len("CMASI") == 0): # this should never happen
        	# Checks for "CMASI" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/FollowPathCommand")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("CMASI" + "/FollowPathCommand")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        VehicleActionCommand.VehicleActionCommand.toDictMembers(self, d)
        d['FirstWaypoint'] = self.FirstWaypoint
        d['WaypointList'] = []
        for x in self.WaypointList:
            if x == None:
                d['WaypointList'].append(None)
            else:
                d['WaypointList'].append(x.toDict())
        d['StartTime'] = self.StartTime
        d['StopTime'] = self.StopTime
        d['RepeatMode'] = self.RepeatMode

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
        str = ws + '<FollowPathCommand Series="CMASI" >\n';
        #str +=VehicleActionCommand.VehicleActionCommand.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</FollowPathCommand>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += VehicleActionCommand.VehicleActionCommand.toXMLMembersStr(self, ws)
        buf += ws + "<FirstWaypoint>" + str(self.FirstWaypoint) + "</FirstWaypoint>\n"
        buf += ws + "<WaypointList>\n"
        for x in self.WaypointList:
            if x == None:
                buf += ws + "    <null/>\n"
            else:
                buf += x.toXMLStr(ws + "    ") 
        buf += ws + "</WaypointList>\n"
        buf += ws + "<StartTime>" + str(self.StartTime) + "</StartTime>\n"
        buf += ws + "<StopTime>" + str(self.StopTime) + "</StopTime>\n"
        buf += ws + "<RepeatMode>" + TravelMode.get_TravelMode_int(self.RepeatMode) + "</RepeatMode>\n"

        return buf
        
