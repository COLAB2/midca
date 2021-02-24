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

from midca.domains.tsp.lmcp.py.afrl.cmasi import Waypoint
from midca.domains.tsp.lmcp.py.afrl.cmasi import Location3D


class TaskImplementationResponse(LMCPObject.LMCPObject):

    def __init__(self):

        self.LMCP_TYPE = 15
        self.SERIES_NAME = "UXTASK"
        self.FULL_LMCP_TYPE_NAME = "uxas.messages.task.TaskImplementationResponse"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 6149757930721443840
        self.SERIES_VERSION = 8

        #Define message fields
        self.ResponseID = 0   #int64
        self.CorrespondingAutomationRequestID = 0   #int64
        self.TaskID = 0   #int64
        self.OptionID = 0   #int64
        self.VehicleID = 0   #int64
        self.TaskWaypoints = []   #Waypoint
        self.FinalLocation = Location3D.Location3D()   #Location3D
        self.FinalHeading = 0   #real32
        self.FinalTime = 0   #int64


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(LMCPObject.LMCPObject.pack(self))
        buffer.extend(struct.pack(">q", self.ResponseID))
        buffer.extend(struct.pack(">q", self.CorrespondingAutomationRequestID))
        buffer.extend(struct.pack(">q", self.TaskID))
        buffer.extend(struct.pack(">q", self.OptionID))
        buffer.extend(struct.pack(">q", self.VehicleID))
        buffer.extend(struct.pack(">H", len(self.TaskWaypoints) ))
        for x in self.TaskWaypoints:
           buffer.extend(struct.pack("B", x != None ))
           if x != None:
               buffer.extend(struct.pack(">q", x.SERIES_NAME_ID))
               buffer.extend(struct.pack(">I", x.LMCP_TYPE))
               buffer.extend(struct.pack(">H", x.SERIES_VERSION))
               buffer.extend(x.pack())
        buffer.extend(struct.pack("B", self.FinalLocation != None ))
        if self.FinalLocation != None:
            buffer.extend(struct.pack(">q", self.FinalLocation.SERIES_NAME_ID))
            buffer.extend(struct.pack(">I", self.FinalLocation.LMCP_TYPE))
            buffer.extend(struct.pack(">H", self.FinalLocation.SERIES_VERSION))
            buffer.extend(self.FinalLocation.pack())
        buffer.extend(struct.pack(">f", self.FinalHeading))
        buffer.extend(struct.pack(">q", self.FinalTime))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = LMCPObject.LMCPObject.unpack(self, buffer, _pos)
        self.ResponseID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.CorrespondingAutomationRequestID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.TaskID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.OptionID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.VehicleID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.TaskWaypoints = [None] * _arraylen
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
                self.TaskWaypoints[x] = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
                _pos = self.TaskWaypoints[x].unpack(buffer, _pos)
            else:
                self.TaskWaypoints[x] = None
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
            self.FinalLocation = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
            _pos = self.FinalLocation.unpack(buffer, _pos)
        else:
            self.FinalLocation = None
        self.FinalHeading = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.FinalTime = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        LMCPObject.LMCPObject.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "ResponseID" and len(e.childNodes) > 0 :
                    self.ResponseID = int(e.childNodes[0].nodeValue)
                elif e.localName == "CorrespondingAutomationRequestID" and len(e.childNodes) > 0 :
                    self.CorrespondingAutomationRequestID = int(e.childNodes[0].nodeValue)
                elif e.localName == "TaskID" and len(e.childNodes) > 0 :
                    self.TaskID = int(e.childNodes[0].nodeValue)
                elif e.localName == "OptionID" and len(e.childNodes) > 0 :
                    self.OptionID = int(e.childNodes[0].nodeValue)
                elif e.localName == "VehicleID" and len(e.childNodes) > 0 :
                    self.VehicleID = int(e.childNodes[0].nodeValue)
                elif e.localName == "TaskWaypoints" and len(e.childNodes) > 0 :
                    self.TaskWaypoints = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            obj = seriesFactory.createObjectByName(c.getAttribute('Series'), c.localName)
                            if obj != None:
                                obj.unpackFromXMLNode(c, seriesFactory)
                                self.TaskWaypoints.append(obj)
                elif e.localName == "FinalLocation" and len(e.childNodes) > 0 :
                    for n in e.childNodes:
                        if n.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.FinalLocation = seriesFactory.createObjectByName(n.getAttribute('Series'), n.localName)
                            if self.FinalLocation != None:
                                self.FinalLocation.unpackFromXMLNode(n, seriesFactory)
                elif e.localName == "FinalHeading" and len(e.childNodes) > 0 :
                    self.FinalHeading = float(e.childNodes[0].nodeValue)
                elif e.localName == "FinalTime" and len(e.childNodes) > 0 :
                    self.FinalTime = int(e.childNodes[0].nodeValue)

        return

    def unpackFromDict(self, d, seriesFactory):
        LMCPObject.LMCPObject.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "ResponseID":
                self.ResponseID = d[key]
            elif key == "CorrespondingAutomationRequestID":
                self.CorrespondingAutomationRequestID = d[key]
            elif key == "TaskID":
                self.TaskID = d[key]
            elif key == "OptionID":
                self.OptionID = d[key]
            elif key == "VehicleID":
                self.VehicleID = d[key]
            elif key == "TaskWaypoints":
                self.TaskWaypoints = []
                for c in d[key]:
                    obj = seriesFactory.unpackFromDict(c)
                    if obj != None:
                        self.TaskWaypoints.append(obj)
            elif key == "FinalLocation":
                self.FinalLocation = seriesFactory.unpackFromDict(d[key])
            elif key == "FinalHeading":
                self.FinalHeading = d[key]
            elif key == "FinalTime":
                self.FinalTime = d[key]

        return

    def get_ResponseID(self):
        return self.ResponseID

    def set_ResponseID(self, value):
        self.ResponseID = int( value )

    def get_CorrespondingAutomationRequestID(self):
        return self.CorrespondingAutomationRequestID

    def set_CorrespondingAutomationRequestID(self, value):
        self.CorrespondingAutomationRequestID = int( value )

    def get_TaskID(self):
        return self.TaskID

    def set_TaskID(self, value):
        self.TaskID = int( value )

    def get_OptionID(self):
        return self.OptionID

    def set_OptionID(self, value):
        self.OptionID = int( value )

    def get_VehicleID(self):
        return self.VehicleID

    def set_VehicleID(self, value):
        self.VehicleID = int( value )

    def get_TaskWaypoints(self):
        return self.TaskWaypoints

    def get_FinalLocation(self):
        return self.FinalLocation

    def set_FinalLocation(self, value):
        self.FinalLocation = value 

    def get_FinalHeading(self):
        return self.FinalHeading

    def set_FinalHeading(self, value):
        self.FinalHeading = float( value )

    def get_FinalTime(self):
        return self.FinalTime

    def set_FinalTime(self, value):
        self.FinalTime = int( value )



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = LMCPObject.LMCPObject.toString(self)
        buf += "From TaskImplementationResponse:\n"
        buf +=    "ResponseID = " + str( self.ResponseID ) + "\n" 
        buf +=    "CorrespondingAutomationRequestID = " + str( self.CorrespondingAutomationRequestID ) + "\n" 
        buf +=    "TaskID = " + str( self.TaskID ) + "\n" 
        buf +=    "OptionID = " + str( self.OptionID ) + "\n" 
        buf +=    "VehicleID = " + str( self.VehicleID ) + "\n" 
        buf +=    "TaskWaypoints = " + str( self.TaskWaypoints ) + "\n" 
        buf +=    "FinalLocation = " + str( self.FinalLocation ) + "\n" 
        buf +=    "FinalHeading = " + str( self.FinalHeading ) + "\n" 
        buf +=    "FinalTime = " + str( self.FinalTime ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "UXTASK") or (len("UXTASK") == 0): # this should never happen
        	# Checks for "UXTASK" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/TaskImplementationResponse")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("UXTASK" + "/TaskImplementationResponse")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        LMCPObject.LMCPObject.toDictMembers(self, d)
        d['ResponseID'] = self.ResponseID
        d['CorrespondingAutomationRequestID'] = self.CorrespondingAutomationRequestID
        d['TaskID'] = self.TaskID
        d['OptionID'] = self.OptionID
        d['VehicleID'] = self.VehicleID
        d['TaskWaypoints'] = []
        for x in self.TaskWaypoints:
            if x == None:
                d['TaskWaypoints'].append(None)
            else:
                d['TaskWaypoints'].append(x.toDict())
        if self.FinalLocation == None:
            d['FinalLocation'] = None
        else:
            d['FinalLocation'] = self.FinalLocation.toDict()
        d['FinalHeading'] = self.FinalHeading
        d['FinalTime'] = self.FinalTime

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
        str = ws + '<TaskImplementationResponse Series="UXTASK" >\n';
        #str +=LMCPObject.LMCPObject.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</TaskImplementationResponse>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += LMCPObject.LMCPObject.toXMLMembersStr(self, ws)
        buf += ws + "<ResponseID>" + str(self.ResponseID) + "</ResponseID>\n"
        buf += ws + "<CorrespondingAutomationRequestID>" + str(self.CorrespondingAutomationRequestID) + "</CorrespondingAutomationRequestID>\n"
        buf += ws + "<TaskID>" + str(self.TaskID) + "</TaskID>\n"
        buf += ws + "<OptionID>" + str(self.OptionID) + "</OptionID>\n"
        buf += ws + "<VehicleID>" + str(self.VehicleID) + "</VehicleID>\n"
        buf += ws + "<TaskWaypoints>\n"
        for x in self.TaskWaypoints:
            if x == None:
                buf += ws + "    <null/>\n"
            else:
                buf += x.toXMLStr(ws + "    ") 
        buf += ws + "</TaskWaypoints>\n"
        if self.FinalLocation != None:
            buf += ws + "<FinalLocation>\n"
            buf += ws + self.FinalLocation.toXMLStr(ws + "    ") 
            buf += ws + "</FinalLocation>\n"
        buf += ws + "<FinalHeading>" + str(self.FinalHeading) + "</FinalHeading>\n"
        buf += ws + "<FinalTime>" + str(self.FinalTime) + "</FinalTime>\n"

        return buf
        
