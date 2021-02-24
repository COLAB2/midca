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



class AutomationRequest(LMCPObject.LMCPObject):

    def __init__(self):

        self.LMCP_TYPE = 40
        self.SERIES_NAME = "CMASI"
        self.FULL_LMCP_TYPE_NAME = "afrl.cmasi.AutomationRequest"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 4849604199710720000
        self.SERIES_VERSION = 3

        #Define message fields
        self.EntityList = []   #int64
        self.TaskList = []   #int64
        self.TaskRelationships = ""   #string
        self.OperatingRegion = 0   #int64
        self.RedoAllTasks = False   #bool


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(LMCPObject.LMCPObject.pack(self))
        buffer.extend(struct.pack(">H", len(self.EntityList) ))
        for x in self.EntityList:
            buffer.extend(struct.pack(">q", x ))
        buffer.extend(struct.pack(">H", len(self.TaskList) ))
        for x in self.TaskList:
            buffer.extend(struct.pack(">q", x ))
        buffer.extend(struct.pack(">H", len(self.TaskRelationships) ))
        if len(self.TaskRelationships) > 0:
            if (sys.version_info > (3, 0)):
                buffer.extend(struct.pack( repr(len(self.TaskRelationships)) + "s", self.TaskRelationships.encode('utf-8')))
            else:
                buffer.extend(struct.pack( repr(len(self.TaskRelationships)) + "s", self.TaskRelationships))
        buffer.extend(struct.pack(">q", self.OperatingRegion))
        boolChar = 1 if self.RedoAllTasks == True else 0
        buffer.extend(struct.pack(">B",boolChar))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = LMCPObject.LMCPObject.unpack(self, buffer, _pos)
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.EntityList = [None] * _arraylen
        if _arraylen > 0:
            self.EntityList = struct.unpack_from(">" + repr(_arraylen) + "q", buffer, _pos )
            _pos += 8 * _arraylen
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.TaskList = [None] * _arraylen
        if _arraylen > 0:
            self.TaskList = struct.unpack_from(">" + repr(_arraylen) + "q", buffer, _pos )
            _pos += 8 * _arraylen
        _strlen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        if _strlen > 0:
            if (sys.version_info > (3, 0)):
                self.TaskRelationships = struct.unpack_from( repr(_strlen) + "s", buffer, _pos )[0].decode('utf-8')
            else:
                 self.TaskRelationships = struct.unpack_from( repr(_strlen) + "s", buffer, _pos )[0]
            _pos += _strlen
        else:
             self.TaskRelationships = ""
        self.OperatingRegion = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        boolChar = struct.unpack_from(">B", buffer, _pos)[0]
        self.RedoAllTasks = True if boolChar == 1 else False
        _pos += 1
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        LMCPObject.LMCPObject.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "EntityList" and len(e.childNodes) > 0 :
                    self.EntityList = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.EntityList.append( int(c.childNodes[0].nodeValue) )
                elif e.localName == "TaskList" and len(e.childNodes) > 0 :
                    self.TaskList = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.TaskList.append( int(c.childNodes[0].nodeValue) )
                elif e.localName == "TaskRelationships" :
                    self.TaskRelationships = str(e.childNodes[0].nodeValue) if len(e.childNodes) > 0 else ""
                elif e.localName == "OperatingRegion" and len(e.childNodes) > 0 :
                    self.OperatingRegion = int(e.childNodes[0].nodeValue)
                elif e.localName == "RedoAllTasks" and len(e.childNodes) > 0 :
                    self.RedoAllTasks = e.childNodes[0].nodeValue.lower() == 'true'

        return

    def unpackFromDict(self, d, seriesFactory):
        LMCPObject.LMCPObject.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "EntityList":
                self.EntityList = []
                for c in d[key]:
                    self.EntityList.append( c )
            elif key == "TaskList":
                self.TaskList = []
                for c in d[key]:
                    self.TaskList.append( c )
            elif key == "TaskRelationships":
                self.TaskRelationships = d[key]
            elif key == "OperatingRegion":
                self.OperatingRegion = d[key]
            elif key == "RedoAllTasks":
                self.RedoAllTasks = d[key]

        return

    def get_EntityList(self):
        return self.EntityList

    def get_TaskList(self):
        return self.TaskList

    def get_TaskRelationships(self):
        return self.TaskRelationships

    def set_TaskRelationships(self, value):
        self.TaskRelationships = str( value )

    def get_OperatingRegion(self):
        return self.OperatingRegion

    def set_OperatingRegion(self, value):
        self.OperatingRegion = int( value )

    def get_RedoAllTasks(self):
        return self.RedoAllTasks

    def set_RedoAllTasks(self, value):
        self.RedoAllTasks = bool( value )



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = LMCPObject.LMCPObject.toString(self)
        buf += "From AutomationRequest:\n"
        buf +=    "EntityList = " + str( self.EntityList ) + "\n" 
        buf +=    "TaskList = " + str( self.TaskList ) + "\n" 
        buf +=    "TaskRelationships = " + str( self.TaskRelationships ) + "\n" 
        buf +=    "OperatingRegion = " + str( self.OperatingRegion ) + "\n" 
        buf +=    "RedoAllTasks = " + str( self.RedoAllTasks ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "CMASI") or (len("CMASI") == 0): # this should never happen
        	# Checks for "CMASI" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/AutomationRequest")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("CMASI" + "/AutomationRequest")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        LMCPObject.LMCPObject.toDictMembers(self, d)
        d['EntityList'] = []
        for x in self.EntityList:
            d['EntityList'].append(x)
        d['TaskList'] = []
        for x in self.TaskList:
            d['TaskList'].append(x)
        d['TaskRelationships'] = self.TaskRelationships
        d['OperatingRegion'] = self.OperatingRegion
        d['RedoAllTasks'] = self.RedoAllTasks

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
        str = ws + '<AutomationRequest Series="CMASI" >\n';
        #str +=LMCPObject.LMCPObject.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</AutomationRequest>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += LMCPObject.LMCPObject.toXMLMembersStr(self, ws)
        buf += ws + "<EntityList>\n"
        for x in self.EntityList:
            buf += ws + "<int64>" + str(x) + "</int64>\n"
        buf += ws + "</EntityList>\n"
        buf += ws + "<TaskList>\n"
        for x in self.TaskList:
            buf += ws + "<int64>" + str(x) + "</int64>\n"
        buf += ws + "</TaskList>\n"
        buf += ws + "<TaskRelationships>" + str(self.TaskRelationships) + "</TaskRelationships>\n"
        buf += ws + "<OperatingRegion>" + str(self.OperatingRegion) + "</OperatingRegion>\n"
        buf += ws + "<RedoAllTasks>" + ('True' if self.RedoAllTasks else 'False') + "</RedoAllTasks>\n"

        return buf
        
