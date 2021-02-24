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

from midca.domains.tsp.lmcp.py.afrl.cmasi import AutomationResponse
from midca.domains.tsp.lmcp.py.uxas.messages.task import PlanningState


class UniqueAutomationResponse(LMCPObject.LMCPObject):

    def __init__(self):

        self.LMCP_TYPE = 9
        self.SERIES_NAME = "UXTASK"
        self.FULL_LMCP_TYPE_NAME = "uxas.messages.task.UniqueAutomationResponse"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 6149757930721443840
        self.SERIES_VERSION = 8

        #Define message fields
        self.ResponseID = 0   #int64
        self.OriginalResponse = AutomationResponse.AutomationResponse()   #AutomationResponse
        self.FinalStates = []   #PlanningState


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(LMCPObject.LMCPObject.pack(self))
        buffer.extend(struct.pack(">q", self.ResponseID))
        buffer.extend(struct.pack("B", self.OriginalResponse != None ))
        if self.OriginalResponse != None:
            buffer.extend(struct.pack(">q", self.OriginalResponse.SERIES_NAME_ID))
            buffer.extend(struct.pack(">I", self.OriginalResponse.LMCP_TYPE))
            buffer.extend(struct.pack(">H", self.OriginalResponse.SERIES_VERSION))
            buffer.extend(self.OriginalResponse.pack())
        buffer.extend(struct.pack(">H", len(self.FinalStates) ))
        for x in self.FinalStates:
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
        self.ResponseID = struct.unpack_from(">q", buffer, _pos)[0]
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
            self.OriginalResponse = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
            _pos = self.OriginalResponse.unpack(buffer, _pos)
        else:
            self.OriginalResponse = None
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.FinalStates = [None] * _arraylen
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
                self.FinalStates[x] = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
                _pos = self.FinalStates[x].unpack(buffer, _pos)
            else:
                self.FinalStates[x] = None
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        LMCPObject.LMCPObject.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "ResponseID" and len(e.childNodes) > 0 :
                    self.ResponseID = int(e.childNodes[0].nodeValue)
                elif e.localName == "OriginalResponse" and len(e.childNodes) > 0 :
                    for n in e.childNodes:
                        if n.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.OriginalResponse = seriesFactory.createObjectByName(n.getAttribute('Series'), n.localName)
                            if self.OriginalResponse != None:
                                self.OriginalResponse.unpackFromXMLNode(n, seriesFactory)
                elif e.localName == "FinalStates" and len(e.childNodes) > 0 :
                    self.FinalStates = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            obj = seriesFactory.createObjectByName(c.getAttribute('Series'), c.localName)
                            if obj != None:
                                obj.unpackFromXMLNode(c, seriesFactory)
                                self.FinalStates.append(obj)

        return

    def unpackFromDict(self, d, seriesFactory):
        LMCPObject.LMCPObject.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "ResponseID":
                self.ResponseID = d[key]
            elif key == "OriginalResponse":
                self.OriginalResponse = seriesFactory.unpackFromDict(d[key])
            elif key == "FinalStates":
                self.FinalStates = []
                for c in d[key]:
                    obj = seriesFactory.unpackFromDict(c)
                    if obj != None:
                        self.FinalStates.append(obj)

        return

    def get_ResponseID(self):
        return self.ResponseID

    def set_ResponseID(self, value):
        self.ResponseID = int( value )

    def get_OriginalResponse(self):
        return self.OriginalResponse

    def set_OriginalResponse(self, value):
        self.OriginalResponse = value 

    def get_FinalStates(self):
        return self.FinalStates



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = LMCPObject.LMCPObject.toString(self)
        buf += "From UniqueAutomationResponse:\n"
        buf +=    "ResponseID = " + str( self.ResponseID ) + "\n" 
        buf +=    "OriginalResponse = " + str( self.OriginalResponse ) + "\n" 
        buf +=    "FinalStates = " + str( self.FinalStates ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "UXTASK") or (len("UXTASK") == 0): # this should never happen
        	# Checks for "UXTASK" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/UniqueAutomationResponse")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("UXTASK" + "/UniqueAutomationResponse")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        LMCPObject.LMCPObject.toDictMembers(self, d)
        d['ResponseID'] = self.ResponseID
        if self.OriginalResponse == None:
            d['OriginalResponse'] = None
        else:
            d['OriginalResponse'] = self.OriginalResponse.toDict()
        d['FinalStates'] = []
        for x in self.FinalStates:
            if x == None:
                d['FinalStates'].append(None)
            else:
                d['FinalStates'].append(x.toDict())

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
        str = ws + '<UniqueAutomationResponse Series="UXTASK" >\n';
        #str +=LMCPObject.LMCPObject.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</UniqueAutomationResponse>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += LMCPObject.LMCPObject.toXMLMembersStr(self, ws)
        buf += ws + "<ResponseID>" + str(self.ResponseID) + "</ResponseID>\n"
        if self.OriginalResponse != None:
            buf += ws + "<OriginalResponse>\n"
            buf += ws + self.OriginalResponse.toXMLStr(ws + "    ") 
            buf += ws + "</OriginalResponse>\n"
        buf += ws + "<FinalStates>\n"
        for x in self.FinalStates:
            if x == None:
                buf += ws + "    <null/>\n"
            else:
                buf += x.toXMLStr(ws + "    ") 
        buf += ws + "</FinalStates>\n"

        return buf
        
