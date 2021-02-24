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



class TaskComplete(LMCPObject.LMCPObject):

    def __init__(self):

        self.LMCP_TYPE = 28
        self.SERIES_NAME = "UXTASK"
        self.FULL_LMCP_TYPE_NAME = "uxas.messages.task.TaskComplete"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 6149757930721443840
        self.SERIES_VERSION = 8

        #Define message fields
        self.TaskID = 0   #int64
        self.EntitiesInvolved = []   #int64
        self.TimeTaskCompleted = 0   #int64


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(LMCPObject.LMCPObject.pack(self))
        buffer.extend(struct.pack(">q", self.TaskID))
        buffer.extend(struct.pack(">H", len(self.EntitiesInvolved) ))
        for x in self.EntitiesInvolved:
            buffer.extend(struct.pack(">q", x ))
        buffer.extend(struct.pack(">q", self.TimeTaskCompleted))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = LMCPObject.LMCPObject.unpack(self, buffer, _pos)
        self.TaskID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.EntitiesInvolved = [None] * _arraylen
        if _arraylen > 0:
            self.EntitiesInvolved = struct.unpack_from(">" + repr(_arraylen) + "q", buffer, _pos )
            _pos += 8 * _arraylen
        self.TimeTaskCompleted = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        LMCPObject.LMCPObject.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "TaskID" and len(e.childNodes) > 0 :
                    self.TaskID = int(e.childNodes[0].nodeValue)
                elif e.localName == "EntitiesInvolved" and len(e.childNodes) > 0 :
                    self.EntitiesInvolved = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.EntitiesInvolved.append( int(c.childNodes[0].nodeValue) )
                elif e.localName == "TimeTaskCompleted" and len(e.childNodes) > 0 :
                    self.TimeTaskCompleted = int(e.childNodes[0].nodeValue)

        return

    def unpackFromDict(self, d, seriesFactory):
        LMCPObject.LMCPObject.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "TaskID":
                self.TaskID = d[key]
            elif key == "EntitiesInvolved":
                self.EntitiesInvolved = []
                for c in d[key]:
                    self.EntitiesInvolved.append( c )
            elif key == "TimeTaskCompleted":
                self.TimeTaskCompleted = d[key]

        return

    def get_TaskID(self):
        return self.TaskID

    def set_TaskID(self, value):
        self.TaskID = int( value )

    def get_EntitiesInvolved(self):
        return self.EntitiesInvolved

    def get_TimeTaskCompleted(self):
        return self.TimeTaskCompleted

    def set_TimeTaskCompleted(self, value):
        self.TimeTaskCompleted = int( value )



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = LMCPObject.LMCPObject.toString(self)
        buf += "From TaskComplete:\n"
        buf +=    "TaskID = " + str( self.TaskID ) + "\n" 
        buf +=    "EntitiesInvolved = " + str( self.EntitiesInvolved ) + "\n" 
        buf +=    "TimeTaskCompleted = " + str( self.TimeTaskCompleted ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "UXTASK") or (len("UXTASK") == 0): # this should never happen
        	# Checks for "UXTASK" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/TaskComplete")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("UXTASK" + "/TaskComplete")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        LMCPObject.LMCPObject.toDictMembers(self, d)
        d['TaskID'] = self.TaskID
        d['EntitiesInvolved'] = []
        for x in self.EntitiesInvolved:
            d['EntitiesInvolved'].append(x)
        d['TimeTaskCompleted'] = self.TimeTaskCompleted

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
        str = ws + '<TaskComplete Series="UXTASK" >\n';
        #str +=LMCPObject.LMCPObject.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</TaskComplete>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += LMCPObject.LMCPObject.toXMLMembersStr(self, ws)
        buf += ws + "<TaskID>" + str(self.TaskID) + "</TaskID>\n"
        buf += ws + "<EntitiesInvolved>\n"
        for x in self.EntitiesInvolved:
            buf += ws + "<int64>" + str(x) + "</int64>\n"
        buf += ws + "</EntitiesInvolved>\n"
        buf += ws + "<TimeTaskCompleted>" + str(self.TimeTaskCompleted) + "</TimeTaskCompleted>\n"

        return buf
        
