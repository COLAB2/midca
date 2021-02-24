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

from midca.domains.tsp.lmcp.py.afrl.cmasi import Location3D
from midca.domains.tsp.lmcp.py.afrl.cmasi import SpeedType
from midca.domains.tsp.lmcp.py.afrl.cmasi import TurnType
from midca.domains.tsp.lmcp.py.afrl.cmasi import VehicleAction


class Waypoint(Location3D.Location3D):

    def __init__(self):
        Location3D.Location3D.__init__(self)
        self.LMCP_TYPE = 35
        self.SERIES_NAME = "CMASI"
        self.FULL_LMCP_TYPE_NAME = "afrl.cmasi.Waypoint"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 4849604199710720000
        self.SERIES_VERSION = 3

        #Define message fields
        self.Number = 0   #int64
        self.NextWaypoint = 0   #int64
        self.Speed = 0   #real32
        self.SpeedType = SpeedType.SpeedType.Airspeed   #SpeedType
        self.ClimbRate = 0   #real32
        self.TurnType = TurnType.TurnType.TurnShort   #TurnType
        self.VehicleActionList = []   #VehicleAction
        self.ContingencyWaypointA = 0   #int64
        self.ContingencyWaypointB = 0   #int64
        self.AssociatedTasks = []   #int64


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(Location3D.Location3D.pack(self))
        buffer.extend(struct.pack(">q", self.Number))
        buffer.extend(struct.pack(">q", self.NextWaypoint))
        buffer.extend(struct.pack(">f", self.Speed))
        buffer.extend(struct.pack(">i", self.SpeedType))
        buffer.extend(struct.pack(">f", self.ClimbRate))
        buffer.extend(struct.pack(">i", self.TurnType))
        buffer.extend(struct.pack(">H", len(self.VehicleActionList) ))
        for x in self.VehicleActionList:
           buffer.extend(struct.pack("B", x != None ))
           if x != None:
               buffer.extend(struct.pack(">q", x.SERIES_NAME_ID))
               buffer.extend(struct.pack(">I", x.LMCP_TYPE))
               buffer.extend(struct.pack(">H", x.SERIES_VERSION))
               buffer.extend(x.pack())
        buffer.extend(struct.pack(">q", self.ContingencyWaypointA))
        buffer.extend(struct.pack(">q", self.ContingencyWaypointB))
        buffer.extend(struct.pack(">H", len(self.AssociatedTasks) ))
        for x in self.AssociatedTasks:
            buffer.extend(struct.pack(">q", x ))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = Location3D.Location3D.unpack(self, buffer, _pos)
        self.Number = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.NextWaypoint = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.Speed = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.SpeedType = struct.unpack_from(">i", buffer, _pos)[0]
        _pos += 4
        self.ClimbRate = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.TurnType = struct.unpack_from(">i", buffer, _pos)[0]
        _pos += 4
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.VehicleActionList = [None] * _arraylen
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
                self.VehicleActionList[x] = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
                _pos = self.VehicleActionList[x].unpack(buffer, _pos)
            else:
                self.VehicleActionList[x] = None
        self.ContingencyWaypointA = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.ContingencyWaypointB = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.AssociatedTasks = [None] * _arraylen
        if _arraylen > 0:
            self.AssociatedTasks = struct.unpack_from(">" + repr(_arraylen) + "q", buffer, _pos )
            _pos += 8 * _arraylen
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        Location3D.Location3D.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "Number" and len(e.childNodes) > 0 :
                    self.Number = int(e.childNodes[0].nodeValue)
                elif e.localName == "NextWaypoint" and len(e.childNodes) > 0 :
                    self.NextWaypoint = int(e.childNodes[0].nodeValue)
                elif e.localName == "Speed" and len(e.childNodes) > 0 :
                    self.Speed = float(e.childNodes[0].nodeValue)
                elif e.localName == "SpeedType" and len(e.childNodes) > 0 :
                    self.SpeedType = SpeedType.get_SpeedType_str(e.childNodes[0].nodeValue)
                elif e.localName == "ClimbRate" and len(e.childNodes) > 0 :
                    self.ClimbRate = float(e.childNodes[0].nodeValue)
                elif e.localName == "TurnType" and len(e.childNodes) > 0 :
                    self.TurnType = TurnType.get_TurnType_str(e.childNodes[0].nodeValue)
                elif e.localName == "VehicleActionList" and len(e.childNodes) > 0 :
                    self.VehicleActionList = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            obj = seriesFactory.createObjectByName(c.getAttribute('Series'), c.localName)
                            if obj != None:
                                obj.unpackFromXMLNode(c, seriesFactory)
                                self.VehicleActionList.append(obj)
                elif e.localName == "ContingencyWaypointA" and len(e.childNodes) > 0 :
                    self.ContingencyWaypointA = int(e.childNodes[0].nodeValue)
                elif e.localName == "ContingencyWaypointB" and len(e.childNodes) > 0 :
                    self.ContingencyWaypointB = int(e.childNodes[0].nodeValue)
                elif e.localName == "AssociatedTasks" and len(e.childNodes) > 0 :
                    self.AssociatedTasks = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.AssociatedTasks.append( int(c.childNodes[0].nodeValue) )

        return

    def unpackFromDict(self, d, seriesFactory):
        Location3D.Location3D.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "Number":
                self.Number = d[key]
            elif key == "NextWaypoint":
                self.NextWaypoint = d[key]
            elif key == "Speed":
                self.Speed = d[key]
            elif key == "SpeedType":
                self.SpeedType = d[key]
            elif key == "ClimbRate":
                self.ClimbRate = d[key]
            elif key == "TurnType":
                self.TurnType = d[key]
            elif key == "VehicleActionList":
                self.VehicleActionList = []
                for c in d[key]:
                    obj = seriesFactory.unpackFromDict(c)
                    if obj != None:
                        self.VehicleActionList.append(obj)
            elif key == "ContingencyWaypointA":
                self.ContingencyWaypointA = d[key]
            elif key == "ContingencyWaypointB":
                self.ContingencyWaypointB = d[key]
            elif key == "AssociatedTasks":
                self.AssociatedTasks = []
                for c in d[key]:
                    self.AssociatedTasks.append( c )

        return

    def get_Number(self):
        return self.Number

    def set_Number(self, value):
        self.Number = int( value )

    def get_NextWaypoint(self):
        return self.NextWaypoint

    def set_NextWaypoint(self, value):
        self.NextWaypoint = int( value )

    def get_Speed(self):
        return self.Speed

    def set_Speed(self, value):
        self.Speed = float( value )

    def get_SpeedType(self):
        return self.SpeedType

    def set_SpeedType(self, value):
        self.SpeedType = value 

    def get_ClimbRate(self):
        return self.ClimbRate

    def set_ClimbRate(self, value):
        self.ClimbRate = float( value )

    def get_TurnType(self):
        return self.TurnType

    def set_TurnType(self, value):
        self.TurnType = value 

    def get_VehicleActionList(self):
        return self.VehicleActionList

    def get_ContingencyWaypointA(self):
        return self.ContingencyWaypointA

    def set_ContingencyWaypointA(self, value):
        self.ContingencyWaypointA = int( value )

    def get_ContingencyWaypointB(self):
        return self.ContingencyWaypointB

    def set_ContingencyWaypointB(self, value):
        self.ContingencyWaypointB = int( value )

    def get_AssociatedTasks(self):
        return self.AssociatedTasks



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = Location3D.Location3D.toString(self)
        buf += "From Waypoint:\n"
        buf +=    "Number = " + str( self.Number ) + "\n" 
        buf +=    "NextWaypoint = " + str( self.NextWaypoint ) + "\n" 
        buf +=    "Speed = " + str( self.Speed ) + "\n" 
        buf +=    "SpeedType = " + str( self.SpeedType ) + "\n" 
        buf +=    "ClimbRate = " + str( self.ClimbRate ) + "\n" 
        buf +=    "TurnType = " + str( self.TurnType ) + "\n" 
        buf +=    "VehicleActionList = " + str( self.VehicleActionList ) + "\n" 
        buf +=    "ContingencyWaypointA = " + str( self.ContingencyWaypointA ) + "\n" 
        buf +=    "ContingencyWaypointB = " + str( self.ContingencyWaypointB ) + "\n" 
        buf +=    "AssociatedTasks = " + str( self.AssociatedTasks ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "CMASI") or (len("CMASI") == 0): # this should never happen
        	# Checks for "CMASI" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/Waypoint")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("CMASI" + "/Waypoint")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        Location3D.Location3D.toDictMembers(self, d)
        d['Number'] = self.Number
        d['NextWaypoint'] = self.NextWaypoint
        d['Speed'] = self.Speed
        d['SpeedType'] = self.SpeedType
        d['ClimbRate'] = self.ClimbRate
        d['TurnType'] = self.TurnType
        d['VehicleActionList'] = []
        for x in self.VehicleActionList:
            if x == None:
                d['VehicleActionList'].append(None)
            else:
                d['VehicleActionList'].append(x.toDict())
        d['ContingencyWaypointA'] = self.ContingencyWaypointA
        d['ContingencyWaypointB'] = self.ContingencyWaypointB
        d['AssociatedTasks'] = []
        for x in self.AssociatedTasks:
            d['AssociatedTasks'].append(x)

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
        str = ws + '<Waypoint Series="CMASI" >\n';
        #str +=Location3D.Location3D.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</Waypoint>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += Location3D.Location3D.toXMLMembersStr(self, ws)
        buf += ws + "<Number>" + str(self.Number) + "</Number>\n"
        buf += ws + "<NextWaypoint>" + str(self.NextWaypoint) + "</NextWaypoint>\n"
        buf += ws + "<Speed>" + str(self.Speed) + "</Speed>\n"
        buf += ws + "<SpeedType>" + SpeedType.get_SpeedType_int(self.SpeedType) + "</SpeedType>\n"
        buf += ws + "<ClimbRate>" + str(self.ClimbRate) + "</ClimbRate>\n"
        buf += ws + "<TurnType>" + TurnType.get_TurnType_int(self.TurnType) + "</TurnType>\n"
        buf += ws + "<VehicleActionList>\n"
        for x in self.VehicleActionList:
            if x == None:
                buf += ws + "    <null/>\n"
            else:
                buf += x.toXMLStr(ws + "    ") 
        buf += ws + "</VehicleActionList>\n"
        buf += ws + "<ContingencyWaypointA>" + str(self.ContingencyWaypointA) + "</ContingencyWaypointA>\n"
        buf += ws + "<ContingencyWaypointB>" + str(self.ContingencyWaypointB) + "</ContingencyWaypointB>\n"
        buf += ws + "<AssociatedTasks>\n"
        for x in self.AssociatedTasks:
            buf += ws + "<int64>" + str(x) + "</int64>\n"
        buf += ws + "</AssociatedTasks>\n"

        return buf
        
