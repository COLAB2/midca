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
from midca.domains.tsp.lmcp.py.uxas.messages.task import PlanningState


class RendezvousTask(Task.Task):

    def __init__(self):
        Task.Task.__init__(self)
        self.LMCP_TYPE = 2
        self.SERIES_NAME = "UXTASK"
        self.FULL_LMCP_TYPE_NAME = "uxas.messages.task.RendezvousTask"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 6149757930721443840
        self.SERIES_VERSION = 8

        #Define message fields
        self.NumberOfParticipants = 0   #byte
        self.RendezvousStates = []   #PlanningState


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(Task.Task.pack(self))
        buffer.extend(struct.pack(">B", self.NumberOfParticipants))
        buffer.extend(struct.pack(">H", len(self.RendezvousStates) ))
        for x in self.RendezvousStates:
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
        _pos = Task.Task.unpack(self, buffer, _pos)
        self.NumberOfParticipants = struct.unpack_from(">B", buffer, _pos)[0]
        _pos += 1
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.RendezvousStates = [None] * _arraylen
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
                self.RendezvousStates[x] = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
                _pos = self.RendezvousStates[x].unpack(buffer, _pos)
            else:
                self.RendezvousStates[x] = None
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        Task.Task.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "NumberOfParticipants" and len(e.childNodes) > 0 :
                    self.NumberOfParticipants = int(e.childNodes[0].nodeValue)
                elif e.localName == "RendezvousStates" and len(e.childNodes) > 0 :
                    self.RendezvousStates = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            obj = seriesFactory.createObjectByName(c.getAttribute('Series'), c.localName)
                            if obj != None:
                                obj.unpackFromXMLNode(c, seriesFactory)
                                self.RendezvousStates.append(obj)

        return

    def unpackFromDict(self, d, seriesFactory):
        Task.Task.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "NumberOfParticipants":
                self.NumberOfParticipants = d[key]
            elif key == "RendezvousStates":
                self.RendezvousStates = []
                for c in d[key]:
                    obj = seriesFactory.unpackFromDict(c)
                    if obj != None:
                        self.RendezvousStates.append(obj)

        return

    def get_NumberOfParticipants(self):
        return self.NumberOfParticipants

    def set_NumberOfParticipants(self, value):
        self.NumberOfParticipants = int( value )

    def get_RendezvousStates(self):
        return self.RendezvousStates



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = Task.Task.toString(self)
        buf += "From RendezvousTask:\n"
        buf +=    "NumberOfParticipants = " + str( self.NumberOfParticipants ) + "\n" 
        buf +=    "RendezvousStates = " + str( self.RendezvousStates ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "UXTASK") or (len("UXTASK") == 0): # this should never happen
        	# Checks for "UXTASK" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/RendezvousTask")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("UXTASK" + "/RendezvousTask")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        Task.Task.toDictMembers(self, d)
        d['NumberOfParticipants'] = self.NumberOfParticipants
        d['RendezvousStates'] = []
        for x in self.RendezvousStates:
            if x == None:
                d['RendezvousStates'].append(None)
            else:
                d['RendezvousStates'].append(x.toDict())

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
        str = ws + '<RendezvousTask Series="UXTASK" >\n';
        #str +=Task.Task.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</RendezvousTask>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += Task.Task.toXMLMembersStr(self, ws)
        buf += ws + "<NumberOfParticipants>" + str(self.NumberOfParticipants) + "</NumberOfParticipants>\n"
        buf += ws + "<RendezvousStates>\n"
        for x in self.RendezvousStates:
            if x == None:
                buf += ws + "    <null/>\n"
            else:
                buf += x.toXMLStr(ws + "    ") 
        buf += ws + "</RendezvousStates>\n"

        return buf
        
