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

from midca.domains.tsp.lmcp.py.afrl.cmasi import SimulationStatusType
from midca.domains.tsp.lmcp.py.afrl.cmasi import KeyValuePair


class SessionStatus(LMCPObject.LMCPObject):

    def __init__(self):

        self.LMCP_TYPE = 46
        self.SERIES_NAME = "CMASI"
        self.FULL_LMCP_TYPE_NAME = "afrl.cmasi.SessionStatus"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 4849604199710720000
        self.SERIES_VERSION = 3

        #Define message fields
        self.State = SimulationStatusType.SimulationStatusType.Stopped   #SimulationStatusType
        self.StartTime = 0   #int64
        self.ScenarioTime = 0   #int64
        self.RealTimeMultiple = 0   #real32
        self.Parameters = []   #KeyValuePair


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(LMCPObject.LMCPObject.pack(self))
        buffer.extend(struct.pack(">i", self.State))
        buffer.extend(struct.pack(">q", self.StartTime))
        buffer.extend(struct.pack(">q", self.ScenarioTime))
        buffer.extend(struct.pack(">f", self.RealTimeMultiple))
        buffer.extend(struct.pack(">H", len(self.Parameters) ))
        for x in self.Parameters:
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
        self.State = struct.unpack_from(">i", buffer, _pos)[0]
        _pos += 4
        self.StartTime = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.ScenarioTime = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.RealTimeMultiple = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.Parameters = [None] * _arraylen
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
                self.Parameters[x] = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
                _pos = self.Parameters[x].unpack(buffer, _pos)
            else:
                self.Parameters[x] = None
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        LMCPObject.LMCPObject.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "State" and len(e.childNodes) > 0 :
                    self.State = SimulationStatusType.get_SimulationStatusType_str(e.childNodes[0].nodeValue)
                elif e.localName == "StartTime" and len(e.childNodes) > 0 :
                    self.StartTime = int(e.childNodes[0].nodeValue)
                elif e.localName == "ScenarioTime" and len(e.childNodes) > 0 :
                    self.ScenarioTime = int(e.childNodes[0].nodeValue)
                elif e.localName == "RealTimeMultiple" and len(e.childNodes) > 0 :
                    self.RealTimeMultiple = float(e.childNodes[0].nodeValue)
                elif e.localName == "Parameters" and len(e.childNodes) > 0 :
                    self.Parameters = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            obj = seriesFactory.createObjectByName(c.getAttribute('Series'), c.localName)
                            if obj != None:
                                obj.unpackFromXMLNode(c, seriesFactory)
                                self.Parameters.append(obj)

        return

    def unpackFromDict(self, d, seriesFactory):
        LMCPObject.LMCPObject.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "State":
                self.State = d[key]
            elif key == "StartTime":
                self.StartTime = d[key]
            elif key == "ScenarioTime":
                self.ScenarioTime = d[key]
            elif key == "RealTimeMultiple":
                self.RealTimeMultiple = d[key]
            elif key == "Parameters":
                self.Parameters = []
                for c in d[key]:
                    obj = seriesFactory.unpackFromDict(c)
                    if obj != None:
                        self.Parameters.append(obj)

        return

    def get_State(self):
        return self.State

    def set_State(self, value):
        self.State = value 

    def get_StartTime(self):
        return self.StartTime

    def set_StartTime(self, value):
        self.StartTime = int( value )

    def get_ScenarioTime(self):
        return self.ScenarioTime

    def set_ScenarioTime(self, value):
        self.ScenarioTime = int( value )

    def get_RealTimeMultiple(self):
        return self.RealTimeMultiple

    def set_RealTimeMultiple(self, value):
        self.RealTimeMultiple = float( value )

    def get_Parameters(self):
        return self.Parameters



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = LMCPObject.LMCPObject.toString(self)
        buf += "From SessionStatus:\n"
        buf +=    "State = " + str( self.State ) + "\n" 
        buf +=    "StartTime = " + str( self.StartTime ) + "\n" 
        buf +=    "ScenarioTime = " + str( self.ScenarioTime ) + "\n" 
        buf +=    "RealTimeMultiple = " + str( self.RealTimeMultiple ) + "\n" 
        buf +=    "Parameters = " + str( self.Parameters ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "CMASI") or (len("CMASI") == 0): # this should never happen
        	# Checks for "CMASI" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/SessionStatus")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("CMASI" + "/SessionStatus")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        LMCPObject.LMCPObject.toDictMembers(self, d)
        d['State'] = self.State
        d['StartTime'] = self.StartTime
        d['ScenarioTime'] = self.ScenarioTime
        d['RealTimeMultiple'] = self.RealTimeMultiple
        d['Parameters'] = []
        for x in self.Parameters:
            if x == None:
                d['Parameters'].append(None)
            else:
                d['Parameters'].append(x.toDict())

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
        str = ws + '<SessionStatus Series="CMASI" >\n';
        #str +=LMCPObject.LMCPObject.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</SessionStatus>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += LMCPObject.LMCPObject.toXMLMembersStr(self, ws)
        buf += ws + "<State>" + SimulationStatusType.get_SimulationStatusType_int(self.State) + "</State>\n"
        buf += ws + "<StartTime>" + str(self.StartTime) + "</StartTime>\n"
        buf += ws + "<ScenarioTime>" + str(self.ScenarioTime) + "</ScenarioTime>\n"
        buf += ws + "<RealTimeMultiple>" + str(self.RealTimeMultiple) + "</RealTimeMultiple>\n"
        buf += ws + "<Parameters>\n"
        for x in self.Parameters:
            if x == None:
                buf += ws + "    <null/>\n"
            else:
                buf += x.toXMLStr(ws + "    ") 
        buf += ws + "</Parameters>\n"

        return buf
        
