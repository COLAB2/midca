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
from midca.domains.tsp.lmcp.py.uxas.messages.task import PlanningState


class TaskImplementationRequest(LMCPObject.LMCPObject):

    def __init__(self):

        self.LMCP_TYPE = 14
        self.SERIES_NAME = "UXTASK"
        self.FULL_LMCP_TYPE_NAME = "uxas.messages.task.TaskImplementationRequest"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 6149757930721443840
        self.SERIES_VERSION = 8

        #Define message fields
        self.RequestID = 0   #int64
        self.CorrespondingAutomationRequestID = 0   #int64
        self.StartingWaypointID = 0   #int64
        self.VehicleID = 0   #int64
        self.StartPosition = Location3D.Location3D()   #Location3D
        self.StartHeading = 0   #real32
        self.StartTime = 0   #int64
        self.RegionID = 0   #int64
        self.TaskID = 0   #int64
        self.OptionID = 0   #int64
        self.TimeThreshold = 0   #int64
        self.NeighborLocations = []   #PlanningState


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(LMCPObject.LMCPObject.pack(self))
        buffer.extend(struct.pack(">q", self.RequestID))
        buffer.extend(struct.pack(">q", self.CorrespondingAutomationRequestID))
        buffer.extend(struct.pack(">q", self.StartingWaypointID))
        buffer.extend(struct.pack(">q", self.VehicleID))
        buffer.extend(struct.pack("B", self.StartPosition != None ))
        if self.StartPosition != None:
            buffer.extend(struct.pack(">q", self.StartPosition.SERIES_NAME_ID))
            buffer.extend(struct.pack(">I", self.StartPosition.LMCP_TYPE))
            buffer.extend(struct.pack(">H", self.StartPosition.SERIES_VERSION))
            buffer.extend(self.StartPosition.pack())
        buffer.extend(struct.pack(">f", self.StartHeading))
        buffer.extend(struct.pack(">q", self.StartTime))
        buffer.extend(struct.pack(">q", self.RegionID))
        buffer.extend(struct.pack(">q", self.TaskID))
        buffer.extend(struct.pack(">q", self.OptionID))
        buffer.extend(struct.pack(">q", self.TimeThreshold))
        buffer.extend(struct.pack(">H", len(self.NeighborLocations) ))
        for x in self.NeighborLocations:
           buffer.extend(struct.pack("B", x != None ))
           if x != None:
               buffer.extend(struct.pack(">q", x.SERIES_NAME_ID))
               buffer.extend(struct.pack(">I", x.LMCP_TYPE))
               buffer.extend(struct.pack(">H", x.SERIES_VERSION))
               buffer.extend(x.pack())

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = LMCPObject.LMCPObject.unpack(self, buffer, _pos)
        self.RequestID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.CorrespondingAutomationRequestID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.StartingWaypointID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.VehicleID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
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
            self.StartPosition = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
            _pos = self.StartPosition.unpack(buffer, _pos)
        else:
            self.StartPosition = None
        self.StartHeading = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.StartTime = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.RegionID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.TaskID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.OptionID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.TimeThreshold = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.NeighborLocations = [None] * _arraylen
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
                self.NeighborLocations[x] = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
                _pos = self.NeighborLocations[x].unpack(buffer, _pos)
            else:
                self.NeighborLocations[x] = None
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        LMCPObject.LMCPObject.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "RequestID" and len(e.childNodes) > 0 :
                    self.RequestID = int(e.childNodes[0].nodeValue)
                elif e.localName == "CorrespondingAutomationRequestID" and len(e.childNodes) > 0 :
                    self.CorrespondingAutomationRequestID = int(e.childNodes[0].nodeValue)
                elif e.localName == "StartingWaypointID" and len(e.childNodes) > 0 :
                    self.StartingWaypointID = int(e.childNodes[0].nodeValue)
                elif e.localName == "VehicleID" and len(e.childNodes) > 0 :
                    self.VehicleID = int(e.childNodes[0].nodeValue)
                elif e.localName == "StartPosition" and len(e.childNodes) > 0 :
                    for n in e.childNodes:
                        if n.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.StartPosition = seriesFactory.createObjectByName(n.getAttribute('Series'), n.localName)
                            if self.StartPosition != None:
                                self.StartPosition.unpackFromXMLNode(n, seriesFactory)
                elif e.localName == "StartHeading" and len(e.childNodes) > 0 :
                    self.StartHeading = float(e.childNodes[0].nodeValue)
                elif e.localName == "StartTime" and len(e.childNodes) > 0 :
                    self.StartTime = int(e.childNodes[0].nodeValue)
                elif e.localName == "RegionID" and len(e.childNodes) > 0 :
                    self.RegionID = int(e.childNodes[0].nodeValue)
                elif e.localName == "TaskID" and len(e.childNodes) > 0 :
                    self.TaskID = int(e.childNodes[0].nodeValue)
                elif e.localName == "OptionID" and len(e.childNodes) > 0 :
                    self.OptionID = int(e.childNodes[0].nodeValue)
                elif e.localName == "TimeThreshold" and len(e.childNodes) > 0 :
                    self.TimeThreshold = int(e.childNodes[0].nodeValue)
                elif e.localName == "NeighborLocations" and len(e.childNodes) > 0 :
                    self.NeighborLocations = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            obj = seriesFactory.createObjectByName(c.getAttribute('Series'), c.localName)
                            if obj != None:
                                obj.unpackFromXMLNode(c, seriesFactory)
                                self.NeighborLocations.append(obj)

        return

    def unpackFromDict(self, d, seriesFactory):
        LMCPObject.LMCPObject.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "RequestID":
                self.RequestID = d[key]
            elif key == "CorrespondingAutomationRequestID":
                self.CorrespondingAutomationRequestID = d[key]
            elif key == "StartingWaypointID":
                self.StartingWaypointID = d[key]
            elif key == "VehicleID":
                self.VehicleID = d[key]
            elif key == "StartPosition":
                self.StartPosition = seriesFactory.unpackFromDict(d[key])
            elif key == "StartHeading":
                self.StartHeading = d[key]
            elif key == "StartTime":
                self.StartTime = d[key]
            elif key == "RegionID":
                self.RegionID = d[key]
            elif key == "TaskID":
                self.TaskID = d[key]
            elif key == "OptionID":
                self.OptionID = d[key]
            elif key == "TimeThreshold":
                self.TimeThreshold = d[key]
            elif key == "NeighborLocations":
                self.NeighborLocations = []
                for c in d[key]:
                    obj = seriesFactory.unpackFromDict(c)
                    if obj != None:
                        self.NeighborLocations.append(obj)

        return

    def get_RequestID(self):
        return self.RequestID

    def set_RequestID(self, value):
        self.RequestID = int( value )

    def get_CorrespondingAutomationRequestID(self):
        return self.CorrespondingAutomationRequestID

    def set_CorrespondingAutomationRequestID(self, value):
        self.CorrespondingAutomationRequestID = int( value )

    def get_StartingWaypointID(self):
        return self.StartingWaypointID

    def set_StartingWaypointID(self, value):
        self.StartingWaypointID = int( value )

    def get_VehicleID(self):
        return self.VehicleID

    def set_VehicleID(self, value):
        self.VehicleID = int( value )

    def get_StartPosition(self):
        return self.StartPosition

    def set_StartPosition(self, value):
        self.StartPosition = value 

    def get_StartHeading(self):
        return self.StartHeading

    def set_StartHeading(self, value):
        self.StartHeading = float( value )

    def get_StartTime(self):
        return self.StartTime

    def set_StartTime(self, value):
        self.StartTime = int( value )

    def get_RegionID(self):
        return self.RegionID

    def set_RegionID(self, value):
        self.RegionID = int( value )

    def get_TaskID(self):
        return self.TaskID

    def set_TaskID(self, value):
        self.TaskID = int( value )

    def get_OptionID(self):
        return self.OptionID

    def set_OptionID(self, value):
        self.OptionID = int( value )

    def get_TimeThreshold(self):
        return self.TimeThreshold

    def set_TimeThreshold(self, value):
        self.TimeThreshold = int( value )

    def get_NeighborLocations(self):
        return self.NeighborLocations



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = LMCPObject.LMCPObject.toString(self)
        buf += "From TaskImplementationRequest:\n"
        buf +=    "RequestID = " + str( self.RequestID ) + "\n" 
        buf +=    "CorrespondingAutomationRequestID = " + str( self.CorrespondingAutomationRequestID ) + "\n" 
        buf +=    "StartingWaypointID = " + str( self.StartingWaypointID ) + "\n" 
        buf +=    "VehicleID = " + str( self.VehicleID ) + "\n" 
        buf +=    "StartPosition = " + str( self.StartPosition ) + "\n" 
        buf +=    "StartHeading = " + str( self.StartHeading ) + "\n" 
        buf +=    "StartTime = " + str( self.StartTime ) + "\n" 
        buf +=    "RegionID = " + str( self.RegionID ) + "\n" 
        buf +=    "TaskID = " + str( self.TaskID ) + "\n" 
        buf +=    "OptionID = " + str( self.OptionID ) + "\n" 
        buf +=    "TimeThreshold = " + str( self.TimeThreshold ) + "\n" 
        buf +=    "NeighborLocations = " + str( self.NeighborLocations ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "UXTASK") or (len("UXTASK") == 0): # this should never happen
        	# Checks for "UXTASK" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/TaskImplementationRequest")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("UXTASK" + "/TaskImplementationRequest")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        LMCPObject.LMCPObject.toDictMembers(self, d)
        d['RequestID'] = self.RequestID
        d['CorrespondingAutomationRequestID'] = self.CorrespondingAutomationRequestID
        d['StartingWaypointID'] = self.StartingWaypointID
        d['VehicleID'] = self.VehicleID
        if self.StartPosition == None:
            d['StartPosition'] = None
        else:
            d['StartPosition'] = self.StartPosition.toDict()
        d['StartHeading'] = self.StartHeading
        d['StartTime'] = self.StartTime
        d['RegionID'] = self.RegionID
        d['TaskID'] = self.TaskID
        d['OptionID'] = self.OptionID
        d['TimeThreshold'] = self.TimeThreshold
        d['NeighborLocations'] = []
        for x in self.NeighborLocations:
            if x == None:
                d['NeighborLocations'].append(None)
            else:
                d['NeighborLocations'].append(x.toDict())

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
        str = ws + '<TaskImplementationRequest Series="UXTASK" >\n';
        #str +=LMCPObject.LMCPObject.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</TaskImplementationRequest>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += LMCPObject.LMCPObject.toXMLMembersStr(self, ws)
        buf += ws + "<RequestID>" + str(self.RequestID) + "</RequestID>\n"
        buf += ws + "<CorrespondingAutomationRequestID>" + str(self.CorrespondingAutomationRequestID) + "</CorrespondingAutomationRequestID>\n"
        buf += ws + "<StartingWaypointID>" + str(self.StartingWaypointID) + "</StartingWaypointID>\n"
        buf += ws + "<VehicleID>" + str(self.VehicleID) + "</VehicleID>\n"
        if self.StartPosition != None:
            buf += ws + "<StartPosition>\n"
            buf += ws + self.StartPosition.toXMLStr(ws + "    ") 
            buf += ws + "</StartPosition>\n"
        buf += ws + "<StartHeading>" + str(self.StartHeading) + "</StartHeading>\n"
        buf += ws + "<StartTime>" + str(self.StartTime) + "</StartTime>\n"
        buf += ws + "<RegionID>" + str(self.RegionID) + "</RegionID>\n"
        buf += ws + "<TaskID>" + str(self.TaskID) + "</TaskID>\n"
        buf += ws + "<OptionID>" + str(self.OptionID) + "</OptionID>\n"
        buf += ws + "<TimeThreshold>" + str(self.TimeThreshold) + "</TimeThreshold>\n"
        buf += ws + "<NeighborLocations>\n"
        for x in self.NeighborLocations:
            if x == None:
                buf += ws + "    <null/>\n"
            else:
                buf += x.toXMLStr(ws + "    ") 
        buf += ws + "</NeighborLocations>\n"

        return buf
        
