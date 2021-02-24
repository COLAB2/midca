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

from midca.domains.tsp.lmcp.py.uxas.messages.task import TaskAssignment


class TaskResume(LMCPObject.LMCPObject):

    def __init__(self):

        self.LMCP_TYPE = 23
        self.SERIES_NAME = "UXTASK"
        self.FULL_LMCP_TYPE_NAME = "uxas.messages.task.TaskResume"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 6149757930721443840
        self.SERIES_VERSION = 8

        #Define message fields
        self.TaskID = 0   #int64
        self.RestartCompletely = False   #bool
        self.ReAssign = None   #TaskAssignment


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(LMCPObject.LMCPObject.pack(self))
        buffer.extend(struct.pack(">q", self.TaskID))
        boolChar = 1 if self.RestartCompletely == True else 0
        buffer.extend(struct.pack(">B",boolChar))
        buffer.extend(struct.pack("B", self.ReAssign != None ))
        if self.ReAssign != None:
            buffer.extend(struct.pack(">q", self.ReAssign.SERIES_NAME_ID))
            buffer.extend(struct.pack(">I", self.ReAssign.LMCP_TYPE))
            buffer.extend(struct.pack(">H", self.ReAssign.SERIES_VERSION))
            buffer.extend(self.ReAssign.pack())

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = LMCPObject.LMCPObject.unpack(self, buffer, _pos)
        self.TaskID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        boolChar = struct.unpack_from(">B", buffer, _pos)[0]
        self.RestartCompletely = True if boolChar == 1 else False
        _pos += 1
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
            self.ReAssign = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
            _pos = self.ReAssign.unpack(buffer, _pos)
        else:
            self.ReAssign = None
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        LMCPObject.LMCPObject.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "TaskID" and len(e.childNodes) > 0 :
                    self.TaskID = int(e.childNodes[0].nodeValue)
                elif e.localName == "RestartCompletely" and len(e.childNodes) > 0 :
                    self.RestartCompletely = e.childNodes[0].nodeValue.lower() == 'true'
                elif e.localName == "ReAssign" and len(e.childNodes) > 0 :
                    for n in e.childNodes:
                        if n.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.ReAssign = seriesFactory.createObjectByName(n.getAttribute('Series'), n.localName)
                            if self.ReAssign != None:
                                self.ReAssign.unpackFromXMLNode(n, seriesFactory)

        return

    def unpackFromDict(self, d, seriesFactory):
        LMCPObject.LMCPObject.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "TaskID":
                self.TaskID = d[key]
            elif key == "RestartCompletely":
                self.RestartCompletely = d[key]
            elif key == "ReAssign":
                self.ReAssign = seriesFactory.unpackFromDict(d[key])

        return

    def get_TaskID(self):
        return self.TaskID

    def set_TaskID(self, value):
        self.TaskID = int( value )

    def get_RestartCompletely(self):
        return self.RestartCompletely

    def set_RestartCompletely(self, value):
        self.RestartCompletely = bool( value )

    def get_ReAssign(self):
        return self.ReAssign

    def set_ReAssign(self, value):
        self.ReAssign = value 



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = LMCPObject.LMCPObject.toString(self)
        buf += "From TaskResume:\n"
        buf +=    "TaskID = " + str( self.TaskID ) + "\n" 
        buf +=    "RestartCompletely = " + str( self.RestartCompletely ) + "\n" 
        buf +=    "ReAssign = " + str( self.ReAssign ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "UXTASK") or (len("UXTASK") == 0): # this should never happen
        	# Checks for "UXTASK" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/TaskResume")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("UXTASK" + "/TaskResume")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        LMCPObject.LMCPObject.toDictMembers(self, d)
        d['TaskID'] = self.TaskID
        d['RestartCompletely'] = self.RestartCompletely
        if self.ReAssign == None:
            d['ReAssign'] = None
        else:
            d['ReAssign'] = self.ReAssign.toDict()

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
        str = ws + '<TaskResume Series="UXTASK" >\n';
        #str +=LMCPObject.LMCPObject.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</TaskResume>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += LMCPObject.LMCPObject.toXMLMembersStr(self, ws)
        buf += ws + "<TaskID>" + str(self.TaskID) + "</TaskID>\n"
        buf += ws + "<RestartCompletely>" + ('True' if self.RestartCompletely else 'False') + "</RestartCompletely>\n"
        if self.ReAssign != None:
            buf += ws + "<ReAssign>\n"
            buf += ws + self.ReAssign.toXMLStr(ws + "    ") 
            buf += ws + "</ReAssign>\n"

        return buf
        
