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
from midca.domains.tsp.lmcp.py.afrl.cmasi import LoiterAction


class LoiterTask(Task.Task):

    def __init__(self):
        Task.Task.__init__(self)
        self.LMCP_TYPE = 34
        self.SERIES_NAME = "CMASI"
        self.FULL_LMCP_TYPE_NAME = "afrl.cmasi.LoiterTask"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 4849604199710720000
        self.SERIES_VERSION = 3

        #Define message fields
        self.DesiredAction = LoiterAction.LoiterAction()   #LoiterAction


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(Task.Task.pack(self))
        buffer.extend(struct.pack("B", self.DesiredAction != None ))
        if self.DesiredAction != None:
            buffer.extend(struct.pack(">q", self.DesiredAction.SERIES_NAME_ID))
            buffer.extend(struct.pack(">I", self.DesiredAction.LMCP_TYPE))
            buffer.extend(struct.pack(">H", self.DesiredAction.SERIES_VERSION))
            buffer.extend(self.DesiredAction.pack())

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = Task.Task.unpack(self, buffer, _pos)
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
            self.DesiredAction = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
            _pos = self.DesiredAction.unpack(buffer, _pos)
        else:
            self.DesiredAction = None
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        Task.Task.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "DesiredAction" and len(e.childNodes) > 0 :
                    for n in e.childNodes:
                        if n.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.DesiredAction = seriesFactory.createObjectByName(n.getAttribute('Series'), n.localName)
                            if self.DesiredAction != None:
                                self.DesiredAction.unpackFromXMLNode(n, seriesFactory)

        return

    def unpackFromDict(self, d, seriesFactory):
        Task.Task.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "DesiredAction":
                self.DesiredAction = seriesFactory.unpackFromDict(d[key])

        return

    def get_DesiredAction(self):
        return self.DesiredAction

    def set_DesiredAction(self, value):
        self.DesiredAction = value 



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = Task.Task.toString(self)
        buf += "From LoiterTask:\n"
        buf +=    "DesiredAction = " + str( self.DesiredAction ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "CMASI") or (len("CMASI") == 0): # this should never happen
        	# Checks for "CMASI" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/LoiterTask")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("CMASI" + "/LoiterTask")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        Task.Task.toDictMembers(self, d)
        if self.DesiredAction == None:
            d['DesiredAction'] = None
        else:
            d['DesiredAction'] = self.DesiredAction.toDict()

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
        str = ws + '<LoiterTask Series="CMASI" >\n';
        #str +=Task.Task.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</LoiterTask>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += Task.Task.toXMLMembersStr(self, ws)
        if self.DesiredAction != None:
            buf += ws + "<DesiredAction>\n"
            buf += ws + self.DesiredAction.toXMLStr(ws + "    ") 
            buf += ws + "</DesiredAction>\n"

        return buf
        
