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

from midca.domains.tsp.lmcp.py.uxas.messages.task import PlanningState


class AssignmentCoordination(LMCPObject.LMCPObject):

    def __init__(self):

        self.LMCP_TYPE = 4
        self.SERIES_NAME = "UXTASK"
        self.FULL_LMCP_TYPE_NAME = "uxas.messages.task.AssignmentCoordination"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 6149757930721443840
        self.SERIES_VERSION = 8

        #Define message fields
        self.CoordinatedAutomationRequestID = 0   #int64
        self.PlanningState = PlanningState.PlanningState()   #PlanningState


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(LMCPObject.LMCPObject.pack(self))
        buffer.extend(struct.pack(">q", self.CoordinatedAutomationRequestID))
        buffer.extend(struct.pack("B", self.PlanningState != None ))
        if self.PlanningState != None:
            buffer.extend(struct.pack(">q", self.PlanningState.SERIES_NAME_ID))
            buffer.extend(struct.pack(">I", self.PlanningState.LMCP_TYPE))
            buffer.extend(struct.pack(">H", self.PlanningState.SERIES_VERSION))
            buffer.extend(self.PlanningState.pack())

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = LMCPObject.LMCPObject.unpack(self, buffer, _pos)
        self.CoordinatedAutomationRequestID = struct.unpack_from(">q", buffer, _pos)[0]
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
            self.PlanningState = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
            _pos = self.PlanningState.unpack(buffer, _pos)
        else:
            self.PlanningState = None
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        LMCPObject.LMCPObject.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "CoordinatedAutomationRequestID" and len(e.childNodes) > 0 :
                    self.CoordinatedAutomationRequestID = int(e.childNodes[0].nodeValue)
                elif e.localName == "PlanningState" and len(e.childNodes) > 0 :
                    for n in e.childNodes:
                        if n.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.PlanningState = seriesFactory.createObjectByName(n.getAttribute('Series'), n.localName)
                            if self.PlanningState != None:
                                self.PlanningState.unpackFromXMLNode(n, seriesFactory)

        return

    def unpackFromDict(self, d, seriesFactory):
        LMCPObject.LMCPObject.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "CoordinatedAutomationRequestID":
                self.CoordinatedAutomationRequestID = d[key]
            elif key == "PlanningState":
                self.PlanningState = seriesFactory.unpackFromDict(d[key])

        return

    def get_CoordinatedAutomationRequestID(self):
        return self.CoordinatedAutomationRequestID

    def set_CoordinatedAutomationRequestID(self, value):
        self.CoordinatedAutomationRequestID = int( value )

    def get_PlanningState(self):
        return self.PlanningState

    def set_PlanningState(self, value):
        self.PlanningState = value 



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = LMCPObject.LMCPObject.toString(self)
        buf += "From AssignmentCoordination:\n"
        buf +=    "CoordinatedAutomationRequestID = " + str( self.CoordinatedAutomationRequestID ) + "\n" 
        buf +=    "PlanningState = " + str( self.PlanningState ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "UXTASK") or (len("UXTASK") == 0): # this should never happen
        	# Checks for "UXTASK" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/AssignmentCoordination")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("UXTASK" + "/AssignmentCoordination")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        LMCPObject.LMCPObject.toDictMembers(self, d)
        d['CoordinatedAutomationRequestID'] = self.CoordinatedAutomationRequestID
        if self.PlanningState == None:
            d['PlanningState'] = None
        else:
            d['PlanningState'] = self.PlanningState.toDict()

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
        str = ws + '<AssignmentCoordination Series="UXTASK" >\n';
        #str +=LMCPObject.LMCPObject.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</AssignmentCoordination>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += LMCPObject.LMCPObject.toXMLMembersStr(self, ws)
        buf += ws + "<CoordinatedAutomationRequestID>" + str(self.CoordinatedAutomationRequestID) + "</CoordinatedAutomationRequestID>\n"
        if self.PlanningState != None:
            buf += ws + "<PlanningState>\n"
            buf += ws + self.PlanningState.toXMLStr(ws + "    ") 
            buf += ws + "</PlanningState>\n"

        return buf
        
