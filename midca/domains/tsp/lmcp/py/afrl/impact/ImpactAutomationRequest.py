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

from midca.domains.tsp.lmcp.py.afrl.cmasi import AutomationRequest
from midca.domains.tsp.lmcp.py.afrl.impact import SpeedAltPair


class ImpactAutomationRequest(LMCPObject.LMCPObject):

    def __init__(self):

        self.LMCP_TYPE = 17
        self.SERIES_NAME = "IMPACT"
        self.FULL_LMCP_TYPE_NAME = "afrl.impact.ImpactAutomationRequest"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 5281966179208134656
        self.SERIES_VERSION = 14

        #Define message fields
        self.RequestID = 0   #int64
        self.TrialRequest = AutomationRequest.AutomationRequest()   #AutomationRequest
        self.OverridePlanningConditions = []   #SpeedAltPair
        self.PlayID = 0   #int64
        self.SolutionID = 0   #int64
        self.Sandbox = False   #bool


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(LMCPObject.LMCPObject.pack(self))
        buffer.extend(struct.pack(">q", self.RequestID))
        buffer.extend(struct.pack("B", self.TrialRequest != None ))
        if self.TrialRequest != None:
            buffer.extend(struct.pack(">q", self.TrialRequest.SERIES_NAME_ID))
            buffer.extend(struct.pack(">I", self.TrialRequest.LMCP_TYPE))
            buffer.extend(struct.pack(">H", self.TrialRequest.SERIES_VERSION))
            buffer.extend(self.TrialRequest.pack())
        buffer.extend(struct.pack(">H", len(self.OverridePlanningConditions) ))
        for x in self.OverridePlanningConditions:
           buffer.extend(struct.pack("B", x != None ))
           if x != None:
               buffer.extend(struct.pack(">q", x.SERIES_NAME_ID))
               buffer.extend(struct.pack(">I", x.LMCP_TYPE))
               buffer.extend(struct.pack(">H", x.SERIES_VERSION))
               buffer.extend(x.pack())
        buffer.extend(struct.pack(">q", self.PlayID))
        buffer.extend(struct.pack(">q", self.SolutionID))
        boolChar = 1 if self.Sandbox == True else 0
        buffer.extend(struct.pack(">B",boolChar))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = LMCPObject.LMCPObject.unpack(self, buffer, _pos)
        self.RequestID = struct.unpack_from(">q", buffer, _pos)[0]
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
            self.TrialRequest = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
            _pos = self.TrialRequest.unpack(buffer, _pos)
        else:
            self.TrialRequest = None
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.OverridePlanningConditions = [None] * _arraylen
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
                self.OverridePlanningConditions[x] = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
                _pos = self.OverridePlanningConditions[x].unpack(buffer, _pos)
            else:
                self.OverridePlanningConditions[x] = None
        self.PlayID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.SolutionID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        boolChar = struct.unpack_from(">B", buffer, _pos)[0]
        self.Sandbox = True if boolChar == 1 else False
        _pos += 1
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        LMCPObject.LMCPObject.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "RequestID" and len(e.childNodes) > 0 :
                    self.RequestID = int(e.childNodes[0].nodeValue)
                elif e.localName == "TrialRequest" and len(e.childNodes) > 0 :
                    for n in e.childNodes:
                        if n.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.TrialRequest = seriesFactory.createObjectByName(n.getAttribute('Series'), n.localName)
                            if self.TrialRequest != None:
                                self.TrialRequest.unpackFromXMLNode(n, seriesFactory)
                elif e.localName == "OverridePlanningConditions" and len(e.childNodes) > 0 :
                    self.OverridePlanningConditions = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            obj = seriesFactory.createObjectByName(c.getAttribute('Series'), c.localName)
                            if obj != None:
                                obj.unpackFromXMLNode(c, seriesFactory)
                                self.OverridePlanningConditions.append(obj)
                elif e.localName == "PlayID" and len(e.childNodes) > 0 :
                    self.PlayID = int(e.childNodes[0].nodeValue)
                elif e.localName == "SolutionID" and len(e.childNodes) > 0 :
                    self.SolutionID = int(e.childNodes[0].nodeValue)
                elif e.localName == "Sandbox" and len(e.childNodes) > 0 :
                    self.Sandbox = e.childNodes[0].nodeValue.lower() == 'true'

        return

    def unpackFromDict(self, d, seriesFactory):
        LMCPObject.LMCPObject.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "RequestID":
                self.RequestID = d[key]
            elif key == "TrialRequest":
                self.TrialRequest = seriesFactory.unpackFromDict(d[key])
            elif key == "OverridePlanningConditions":
                self.OverridePlanningConditions = []
                for c in d[key]:
                    obj = seriesFactory.unpackFromDict(c)
                    if obj != None:
                        self.OverridePlanningConditions.append(obj)
            elif key == "PlayID":
                self.PlayID = d[key]
            elif key == "SolutionID":
                self.SolutionID = d[key]
            elif key == "Sandbox":
                self.Sandbox = d[key]

        return

    def get_RequestID(self):
        return self.RequestID

    def set_RequestID(self, value):
        self.RequestID = int( value )

    def get_TrialRequest(self):
        return self.TrialRequest

    def set_TrialRequest(self, value):
        self.TrialRequest = value 

    def get_OverridePlanningConditions(self):
        return self.OverridePlanningConditions

    def get_PlayID(self):
        return self.PlayID

    def set_PlayID(self, value):
        self.PlayID = int( value )

    def get_SolutionID(self):
        return self.SolutionID

    def set_SolutionID(self, value):
        self.SolutionID = int( value )

    def get_Sandbox(self):
        return self.Sandbox

    def set_Sandbox(self, value):
        self.Sandbox = bool( value )



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = LMCPObject.LMCPObject.toString(self)
        buf += "From ImpactAutomationRequest:\n"
        buf +=    "RequestID = " + str( self.RequestID ) + "\n" 
        buf +=    "TrialRequest = " + str( self.TrialRequest ) + "\n" 
        buf +=    "OverridePlanningConditions = " + str( self.OverridePlanningConditions ) + "\n" 
        buf +=    "PlayID = " + str( self.PlayID ) + "\n" 
        buf +=    "SolutionID = " + str( self.SolutionID ) + "\n" 
        buf +=    "Sandbox = " + str( self.Sandbox ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "IMPACT") or (len("IMPACT") == 0): # this should never happen
        	# Checks for "IMPACT" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/ImpactAutomationRequest")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("IMPACT" + "/ImpactAutomationRequest")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        LMCPObject.LMCPObject.toDictMembers(self, d)
        d['RequestID'] = self.RequestID
        if self.TrialRequest == None:
            d['TrialRequest'] = None
        else:
            d['TrialRequest'] = self.TrialRequest.toDict()
        d['OverridePlanningConditions'] = []
        for x in self.OverridePlanningConditions:
            if x == None:
                d['OverridePlanningConditions'].append(None)
            else:
                d['OverridePlanningConditions'].append(x.toDict())
        d['PlayID'] = self.PlayID
        d['SolutionID'] = self.SolutionID
        d['Sandbox'] = self.Sandbox

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
        str = ws + '<ImpactAutomationRequest Series="IMPACT" >\n';
        #str +=LMCPObject.LMCPObject.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</ImpactAutomationRequest>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += LMCPObject.LMCPObject.toXMLMembersStr(self, ws)
        buf += ws + "<RequestID>" + str(self.RequestID) + "</RequestID>\n"
        if self.TrialRequest != None:
            buf += ws + "<TrialRequest>\n"
            buf += ws + self.TrialRequest.toXMLStr(ws + "    ") 
            buf += ws + "</TrialRequest>\n"
        buf += ws + "<OverridePlanningConditions>\n"
        for x in self.OverridePlanningConditions:
            if x == None:
                buf += ws + "    <null/>\n"
            else:
                buf += x.toXMLStr(ws + "    ") 
        buf += ws + "</OverridePlanningConditions>\n"
        buf += ws + "<PlayID>" + str(self.PlayID) + "</PlayID>\n"
        buf += ws + "<SolutionID>" + str(self.SolutionID) + "</SolutionID>\n"
        buf += ws + "<Sandbox>" + ('True' if self.Sandbox else 'False') + "</Sandbox>\n"

        return buf
        
