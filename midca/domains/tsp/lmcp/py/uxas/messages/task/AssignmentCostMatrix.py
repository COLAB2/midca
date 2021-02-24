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

from midca.domains.tsp.lmcp.py.uxas.messages.task import TaskOptionCost


class AssignmentCostMatrix(LMCPObject.LMCPObject):

    def __init__(self):

        self.LMCP_TYPE = 16
        self.SERIES_NAME = "UXTASK"
        self.FULL_LMCP_TYPE_NAME = "uxas.messages.task.AssignmentCostMatrix"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 6149757930721443840
        self.SERIES_VERSION = 8

        #Define message fields
        self.CorrespondingAutomationRequestID = 0   #int64
        self.TaskLevelRelationship = ""   #string
        self.TaskList = []   #int64
        self.OperatingRegion = 0   #int64
        self.CostMatrix = []   #TaskOptionCost


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(LMCPObject.LMCPObject.pack(self))
        buffer.extend(struct.pack(">q", self.CorrespondingAutomationRequestID))
        buffer.extend(struct.pack(">H", len(self.TaskLevelRelationship) ))
        if len(self.TaskLevelRelationship) > 0:
            if (sys.version_info > (3, 0)):
                buffer.extend(struct.pack( repr(len(self.TaskLevelRelationship)) + "s", self.TaskLevelRelationship.encode('utf-8')))
            else:
                buffer.extend(struct.pack( repr(len(self.TaskLevelRelationship)) + "s", self.TaskLevelRelationship))
        buffer.extend(struct.pack(">H", len(self.TaskList) ))
        for x in self.TaskList:
            buffer.extend(struct.pack(">q", x ))
        buffer.extend(struct.pack(">q", self.OperatingRegion))
        buffer.extend(struct.pack(">H", len(self.CostMatrix) ))
        for x in self.CostMatrix:
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
        _strlen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        if _strlen > 0:
            if (sys.version_info > (3, 0)):
                self.TaskLevelRelationship = struct.unpack_from( repr(_strlen) + "s", buffer, _pos )[0].decode('utf-8')
            else:
                 self.TaskLevelRelationship = struct.unpack_from( repr(_strlen) + "s", buffer, _pos )[0]
            _pos += _strlen
        else:
             self.TaskLevelRelationship = ""
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.TaskList = [None] * _arraylen
        if _arraylen > 0:
            self.TaskList = struct.unpack_from(">" + repr(_arraylen) + "q", buffer, _pos )
            _pos += 8 * _arraylen
        self.OperatingRegion = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.CostMatrix = [None] * _arraylen
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
                self.CostMatrix[x] = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
                _pos = self.CostMatrix[x].unpack(buffer, _pos)
            else:
                self.CostMatrix[x] = None
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        LMCPObject.LMCPObject.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "CorrespondingAutomationRequestID" and len(e.childNodes) > 0 :
                    self.CorrespondingAutomationRequestID = int(e.childNodes[0].nodeValue)
                elif e.localName == "TaskLevelRelationship" :
                    self.TaskLevelRelationship = str(e.childNodes[0].nodeValue) if len(e.childNodes) > 0 else ""
                elif e.localName == "TaskList" and len(e.childNodes) > 0 :
                    self.TaskList = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.TaskList.append( int(c.childNodes[0].nodeValue) )
                elif e.localName == "OperatingRegion" and len(e.childNodes) > 0 :
                    self.OperatingRegion = int(e.childNodes[0].nodeValue)
                elif e.localName == "CostMatrix" and len(e.childNodes) > 0 :
                    self.CostMatrix = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            obj = seriesFactory.createObjectByName(c.getAttribute('Series'), c.localName)
                            if obj != None:
                                obj.unpackFromXMLNode(c, seriesFactory)
                                self.CostMatrix.append(obj)

        return

    def unpackFromDict(self, d, seriesFactory):
        LMCPObject.LMCPObject.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "CorrespondingAutomationRequestID":
                self.CorrespondingAutomationRequestID = d[key]
            elif key == "TaskLevelRelationship":
                self.TaskLevelRelationship = d[key]
            elif key == "TaskList":
                self.TaskList = []
                for c in d[key]:
                    self.TaskList.append( c )
            elif key == "OperatingRegion":
                self.OperatingRegion = d[key]
            elif key == "CostMatrix":
                self.CostMatrix = []
                for c in d[key]:
                    obj = seriesFactory.unpackFromDict(c)
                    if obj != None:
                        self.CostMatrix.append(obj)

        return

    def get_CorrespondingAutomationRequestID(self):
        return self.CorrespondingAutomationRequestID

    def set_CorrespondingAutomationRequestID(self, value):
        self.CorrespondingAutomationRequestID = int( value )

    def get_TaskLevelRelationship(self):
        return self.TaskLevelRelationship

    def set_TaskLevelRelationship(self, value):
        self.TaskLevelRelationship = str( value )

    def get_TaskList(self):
        return self.TaskList

    def get_OperatingRegion(self):
        return self.OperatingRegion

    def set_OperatingRegion(self, value):
        self.OperatingRegion = int( value )

    def get_CostMatrix(self):
        return self.CostMatrix



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = LMCPObject.LMCPObject.toString(self)
        buf += "From AssignmentCostMatrix:\n"
        buf +=    "CorrespondingAutomationRequestID = " + str( self.CorrespondingAutomationRequestID ) + "\n" 
        buf +=    "TaskLevelRelationship = " + str( self.TaskLevelRelationship ) + "\n" 
        buf +=    "TaskList = " + str( self.TaskList ) + "\n" 
        buf +=    "OperatingRegion = " + str( self.OperatingRegion ) + "\n" 
        buf +=    "CostMatrix = " + str( self.CostMatrix ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "UXTASK") or (len("UXTASK") == 0): # this should never happen
        	# Checks for "UXTASK" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/AssignmentCostMatrix")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("UXTASK" + "/AssignmentCostMatrix")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        LMCPObject.LMCPObject.toDictMembers(self, d)
        d['CorrespondingAutomationRequestID'] = self.CorrespondingAutomationRequestID
        d['TaskLevelRelationship'] = self.TaskLevelRelationship
        d['TaskList'] = []
        for x in self.TaskList:
            d['TaskList'].append(x)
        d['OperatingRegion'] = self.OperatingRegion
        d['CostMatrix'] = []
        for x in self.CostMatrix:
            if x == None:
                d['CostMatrix'].append(None)
            else:
                d['CostMatrix'].append(x.toDict())

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
        str = ws + '<AssignmentCostMatrix Series="UXTASK" >\n';
        #str +=LMCPObject.LMCPObject.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</AssignmentCostMatrix>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += LMCPObject.LMCPObject.toXMLMembersStr(self, ws)
        buf += ws + "<CorrespondingAutomationRequestID>" + str(self.CorrespondingAutomationRequestID) + "</CorrespondingAutomationRequestID>\n"
        buf += ws + "<TaskLevelRelationship>" + str(self.TaskLevelRelationship) + "</TaskLevelRelationship>\n"
        buf += ws + "<TaskList>\n"
        for x in self.TaskList:
            buf += ws + "<int64>" + str(x) + "</int64>\n"
        buf += ws + "</TaskList>\n"
        buf += ws + "<OperatingRegion>" + str(self.OperatingRegion) + "</OperatingRegion>\n"
        buf += ws + "<CostMatrix>\n"
        for x in self.CostMatrix:
            if x == None:
                buf += ws + "    <null/>\n"
            else:
                buf += x.toXMLStr(ws + "    ") 
        buf += ws + "</CostMatrix>\n"

        return buf
        
