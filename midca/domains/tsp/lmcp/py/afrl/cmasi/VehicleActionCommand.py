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
from midca.domains.tsp.lmcp.py.afrl.cmasi import CommandStatusType


class VehicleActionCommand(LMCPObject.LMCPObject):

    def __init__(self):

        self.LMCP_TYPE = 47
        self.SERIES_NAME = "CMASI"
        self.FULL_LMCP_TYPE_NAME = "afrl.cmasi.VehicleActionCommand"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 4849604199710720000
        self.SERIES_VERSION = 3

        #Define message fields
        self.CommandID = 0   #int64
        self.VehicleID = 0   #int64
        self.VehicleActionList = []   #VehicleAction
        self.Status = CommandStatusType.CommandStatusType.Pending   #CommandStatusType


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(LMCPObject.LMCPObject.pack(self))
        buffer.extend(struct.pack(">q", self.CommandID))
        buffer.extend(struct.pack(">q", self.VehicleID))
        buffer.extend(struct.pack(">H", len(self.VehicleActionList) ))
        for x in self.VehicleActionList:
           buffer.extend(struct.pack("B", x != None ))
           if x != None:
               buffer.extend(struct.pack(">q", x.SERIES_NAME_ID))
               buffer.extend(struct.pack(">I", x.LMCP_TYPE))
               buffer.extend(struct.pack(">H", x.SERIES_VERSION))
               buffer.extend(x.pack())
        buffer.extend(struct.pack(">i", self.Status))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = LMCPObject.LMCPObject.unpack(self, buffer, _pos)
        self.CommandID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.VehicleID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
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
        self.Status = struct.unpack_from(">i", buffer, _pos)[0]
        _pos += 4
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        LMCPObject.LMCPObject.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "CommandID" and len(e.childNodes) > 0 :
                    self.CommandID = int(e.childNodes[0].nodeValue)
                elif e.localName == "VehicleID" and len(e.childNodes) > 0 :
                    self.VehicleID = int(e.childNodes[0].nodeValue)
                elif e.localName == "VehicleActionList" and len(e.childNodes) > 0 :
                    self.VehicleActionList = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            obj = seriesFactory.createObjectByName(c.getAttribute('Series'), c.localName)
                            if obj != None:
                                obj.unpackFromXMLNode(c, seriesFactory)
                                self.VehicleActionList.append(obj)
                elif e.localName == "Status" and len(e.childNodes) > 0 :
                    self.Status = CommandStatusType.get_CommandStatusType_str(e.childNodes[0].nodeValue)

        return

    def unpackFromDict(self, d, seriesFactory):
        LMCPObject.LMCPObject.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "CommandID":
                self.CommandID = d[key]
            elif key == "VehicleID":
                self.VehicleID = d[key]
            elif key == "VehicleActionList":
                self.VehicleActionList = []
                for c in d[key]:
                    obj = seriesFactory.unpackFromDict(c)
                    if obj != None:
                        self.VehicleActionList.append(obj)
            elif key == "Status":
                self.Status = d[key]

        return

    def get_CommandID(self):
        return self.CommandID

    def set_CommandID(self, value):
        self.CommandID = int( value )

    def get_VehicleID(self):
        return self.VehicleID

    def set_VehicleID(self, value):
        self.VehicleID = int( value )

    def get_VehicleActionList(self):
        return self.VehicleActionList

    def get_Status(self):
        return self.Status

    def set_Status(self, value):
        self.Status = value 



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = LMCPObject.LMCPObject.toString(self)
        buf += "From VehicleActionCommand:\n"
        buf +=    "CommandID = " + str( self.CommandID ) + "\n" 
        buf +=    "VehicleID = " + str( self.VehicleID ) + "\n" 
        buf +=    "VehicleActionList = " + str( self.VehicleActionList ) + "\n" 
        buf +=    "Status = " + str( self.Status ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "CMASI") or (len("CMASI") == 0): # this should never happen
        	# Checks for "CMASI" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/VehicleActionCommand")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("CMASI" + "/VehicleActionCommand")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        LMCPObject.LMCPObject.toDictMembers(self, d)
        d['CommandID'] = self.CommandID
        d['VehicleID'] = self.VehicleID
        d['VehicleActionList'] = []
        for x in self.VehicleActionList:
            if x == None:
                d['VehicleActionList'].append(None)
            else:
                d['VehicleActionList'].append(x.toDict())
        d['Status'] = self.Status

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
        str = ws + '<VehicleActionCommand Series="CMASI" >\n';
        #str +=LMCPObject.LMCPObject.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</VehicleActionCommand>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += LMCPObject.LMCPObject.toXMLMembersStr(self, ws)
        buf += ws + "<CommandID>" + str(self.CommandID) + "</CommandID>\n"
        buf += ws + "<VehicleID>" + str(self.VehicleID) + "</VehicleID>\n"
        buf += ws + "<VehicleActionList>\n"
        for x in self.VehicleActionList:
            if x == None:
                buf += ws + "    <null/>\n"
            else:
                buf += x.toXMLStr(ws + "    ") 
        buf += ws + "</VehicleActionList>\n"
        buf += ws + "<Status>" + CommandStatusType.get_CommandStatusType_int(self.Status) + "</Status>\n"

        return buf
        
