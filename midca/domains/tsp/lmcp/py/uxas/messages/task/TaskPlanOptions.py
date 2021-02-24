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

from midca.domains.tsp.lmcp.py.uxas.messages.task import TaskOption


class TaskPlanOptions(LMCPObject.LMCPObject):

    def __init__(self):

        self.LMCP_TYPE = 21
        self.SERIES_NAME = "UXTASK"
        self.FULL_LMCP_TYPE_NAME = "uxas.messages.task.TaskPlanOptions"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 6149757930721443840
        self.SERIES_VERSION = 8

        #Define message fields
        self.CorrespondingAutomationRequestID = 0   #int64
        self.TaskID = 0   #int64
        self.Composition = ""   #string
        self.Options = []   #TaskOption


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(LMCPObject.LMCPObject.pack(self))
        buffer.extend(struct.pack(">q", self.CorrespondingAutomationRequestID))
        buffer.extend(struct.pack(">q", self.TaskID))
        buffer.extend(struct.pack(">H", len(self.Composition) ))
        if len(self.Composition) > 0:
            if (sys.version_info > (3, 0)):
                buffer.extend(struct.pack( repr(len(self.Composition)) + "s", self.Composition.encode('utf-8')))
            else:
                buffer.extend(struct.pack( repr(len(self.Composition)) + "s", self.Composition))
        buffer.extend(struct.pack(">H", len(self.Options) ))
        for x in self.Options:
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
        self.CorrespondingAutomationRequestID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.TaskID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        _strlen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        if _strlen > 0:
            if (sys.version_info > (3, 0)):
                self.Composition = struct.unpack_from( repr(_strlen) + "s", buffer, _pos )[0].decode('utf-8')
            else:
                 self.Composition = struct.unpack_from( repr(_strlen) + "s", buffer, _pos )[0]
            _pos += _strlen
        else:
             self.Composition = ""
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.Options = [None] * _arraylen
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
                self.Options[x] = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
                _pos = self.Options[x].unpack(buffer, _pos)
            else:
                self.Options[x] = None
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        LMCPObject.LMCPObject.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "CorrespondingAutomationRequestID" and len(e.childNodes) > 0 :
                    self.CorrespondingAutomationRequestID = int(e.childNodes[0].nodeValue)
                elif e.localName == "TaskID" and len(e.childNodes) > 0 :
                    self.TaskID = int(e.childNodes[0].nodeValue)
                elif e.localName == "Composition" :
                    self.Composition = str(e.childNodes[0].nodeValue) if len(e.childNodes) > 0 else ""
                elif e.localName == "Options" and len(e.childNodes) > 0 :
                    self.Options = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            obj = seriesFactory.createObjectByName(c.getAttribute('Series'), c.localName)
                            if obj != None:
                                obj.unpackFromXMLNode(c, seriesFactory)
                                self.Options.append(obj)

        return

    def unpackFromDict(self, d, seriesFactory):
        LMCPObject.LMCPObject.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "CorrespondingAutomationRequestID":
                self.CorrespondingAutomationRequestID = d[key]
            elif key == "TaskID":
                self.TaskID = d[key]
            elif key == "Composition":
                self.Composition = d[key]
            elif key == "Options":
                self.Options = []
                for c in d[key]:
                    obj = seriesFactory.unpackFromDict(c)
                    if obj != None:
                        self.Options.append(obj)

        return

    def get_CorrespondingAutomationRequestID(self):
        return self.CorrespondingAutomationRequestID

    def set_CorrespondingAutomationRequestID(self, value):
        self.CorrespondingAutomationRequestID = int( value )

    def get_TaskID(self):
        return self.TaskID

    def set_TaskID(self, value):
        self.TaskID = int( value )

    def get_Composition(self):
        return self.Composition

    def set_Composition(self, value):
        self.Composition = str( value )

    def get_Options(self):
        return self.Options



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = LMCPObject.LMCPObject.toString(self)
        buf += "From TaskPlanOptions:\n"
        buf +=    "CorrespondingAutomationRequestID = " + str( self.CorrespondingAutomationRequestID ) + "\n" 
        buf +=    "TaskID = " + str( self.TaskID ) + "\n" 
        buf +=    "Composition = " + str( self.Composition ) + "\n" 
        buf +=    "Options = " + str( self.Options ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "UXTASK") or (len("UXTASK") == 0): # this should never happen
        	# Checks for "UXTASK" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/TaskPlanOptions")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("UXTASK" + "/TaskPlanOptions")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        LMCPObject.LMCPObject.toDictMembers(self, d)
        d['CorrespondingAutomationRequestID'] = self.CorrespondingAutomationRequestID
        d['TaskID'] = self.TaskID
        d['Composition'] = self.Composition
        d['Options'] = []
        for x in self.Options:
            if x == None:
                d['Options'].append(None)
            else:
                d['Options'].append(x.toDict())

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
        str = ws + '<TaskPlanOptions Series="UXTASK" >\n';
        #str +=LMCPObject.LMCPObject.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</TaskPlanOptions>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += LMCPObject.LMCPObject.toXMLMembersStr(self, ws)
        buf += ws + "<CorrespondingAutomationRequestID>" + str(self.CorrespondingAutomationRequestID) + "</CorrespondingAutomationRequestID>\n"
        buf += ws + "<TaskID>" + str(self.TaskID) + "</TaskID>\n"
        buf += ws + "<Composition>" + str(self.Composition) + "</Composition>\n"
        buf += ws + "<Options>\n"
        for x in self.Options:
            if x == None:
                buf += ws + "    <null/>\n"
            else:
                buf += x.toXMLStr(ws + "    ") 
        buf += ws + "</Options>\n"

        return buf
        
