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



class BatchSummaryRequest(LMCPObject.LMCPObject):

    def __init__(self):

        self.LMCP_TYPE = 12
        self.SERIES_NAME = "IMPACT"
        self.FULL_LMCP_TYPE_NAME = "afrl.impact.BatchSummaryRequest"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 5281966179208134656
        self.SERIES_VERSION = 14

        #Define message fields
        self.RequestID = 0   #int64
        self.Vehicles = []   #int64
        self.TaskList = []   #int64
        self.TaskRelationships = ""   #string
        self.InterTaskPercentage = []   #real32
        self.OperatingRegion = 0   #int64


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(LMCPObject.LMCPObject.pack(self))
        buffer.extend(struct.pack(">q", self.RequestID))
        buffer.extend(struct.pack(">H", len(self.Vehicles) ))
        for x in self.Vehicles:
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
        buffer.extend(struct.pack(">H", len(self.InterTaskPercentage) ))
        for x in self.InterTaskPercentage:
            buffer.extend(struct.pack(">f", x ))
        buffer.extend(struct.pack(">q", self.OperatingRegion))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = LMCPObject.LMCPObject.unpack(self, buffer, _pos)
        self.RequestID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.Vehicles = [None] * _arraylen
        if _arraylen > 0:
            self.Vehicles = struct.unpack_from(">" + repr(_arraylen) + "q", buffer, _pos )
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
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.InterTaskPercentage = [None] * _arraylen
        if _arraylen > 0:
            self.InterTaskPercentage = struct.unpack_from(">" + repr(_arraylen) + "f", buffer, _pos )
            _pos += 4 * _arraylen
        self.OperatingRegion = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        LMCPObject.LMCPObject.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "RequestID" and len(e.childNodes) > 0 :
                    self.RequestID = int(e.childNodes[0].nodeValue)
                elif e.localName == "Vehicles" and len(e.childNodes) > 0 :
                    self.Vehicles = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.Vehicles.append( int(c.childNodes[0].nodeValue) )
                elif e.localName == "TaskList" and len(e.childNodes) > 0 :
                    self.TaskList = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.TaskList.append( int(c.childNodes[0].nodeValue) )
                elif e.localName == "TaskRelationships" :
                    self.TaskRelationships = str(e.childNodes[0].nodeValue) if len(e.childNodes) > 0 else ""
                elif e.localName == "InterTaskPercentage" and len(e.childNodes) > 0 :
                    self.InterTaskPercentage = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.InterTaskPercentage.append( float(c.childNodes[0].nodeValue) )
                elif e.localName == "OperatingRegion" and len(e.childNodes) > 0 :
                    self.OperatingRegion = int(e.childNodes[0].nodeValue)

        return

    def unpackFromDict(self, d, seriesFactory):
        LMCPObject.LMCPObject.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "RequestID":
                self.RequestID = d[key]
            elif key == "Vehicles":
                self.Vehicles = []
                for c in d[key]:
                    self.Vehicles.append( c )
            elif key == "TaskList":
                self.TaskList = []
                for c in d[key]:
                    self.TaskList.append( c )
            elif key == "TaskRelationships":
                self.TaskRelationships = d[key]
            elif key == "InterTaskPercentage":
                self.InterTaskPercentage = []
                for c in d[key]:
                    self.InterTaskPercentage.append( c )
            elif key == "OperatingRegion":
                self.OperatingRegion = d[key]

        return

    def get_RequestID(self):
        return self.RequestID

    def set_RequestID(self, value):
        self.RequestID = int( value )

    def get_Vehicles(self):
        return self.Vehicles

    def get_TaskList(self):
        return self.TaskList

    def get_TaskRelationships(self):
        return self.TaskRelationships

    def set_TaskRelationships(self, value):
        self.TaskRelationships = str( value )

    def get_InterTaskPercentage(self):
        return self.InterTaskPercentage

    def get_OperatingRegion(self):
        return self.OperatingRegion

    def set_OperatingRegion(self, value):
        self.OperatingRegion = int( value )



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = LMCPObject.LMCPObject.toString(self)
        buf += "From BatchSummaryRequest:\n"
        buf +=    "RequestID = " + str( self.RequestID ) + "\n" 
        buf +=    "Vehicles = " + str( self.Vehicles ) + "\n" 
        buf +=    "TaskList = " + str( self.TaskList ) + "\n" 
        buf +=    "TaskRelationships = " + str( self.TaskRelationships ) + "\n" 
        buf +=    "InterTaskPercentage = " + str( self.InterTaskPercentage ) + "\n" 
        buf +=    "OperatingRegion = " + str( self.OperatingRegion ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "IMPACT") or (len("IMPACT") == 0): # this should never happen
        	# Checks for "IMPACT" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/BatchSummaryRequest")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("IMPACT" + "/BatchSummaryRequest")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        LMCPObject.LMCPObject.toDictMembers(self, d)
        d['RequestID'] = self.RequestID
        d['Vehicles'] = []
        for x in self.Vehicles:
            d['Vehicles'].append(x)
        d['TaskList'] = []
        for x in self.TaskList:
            d['TaskList'].append(x)
        d['TaskRelationships'] = self.TaskRelationships
        d['InterTaskPercentage'] = []
        for x in self.InterTaskPercentage:
            d['InterTaskPercentage'].append(x)
        d['OperatingRegion'] = self.OperatingRegion

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
        str = ws + '<BatchSummaryRequest Series="IMPACT" >\n';
        #str +=LMCPObject.LMCPObject.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</BatchSummaryRequest>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += LMCPObject.LMCPObject.toXMLMembersStr(self, ws)
        buf += ws + "<RequestID>" + str(self.RequestID) + "</RequestID>\n"
        buf += ws + "<Vehicles>\n"
        for x in self.Vehicles:
            buf += ws + "<int64>" + str(x) + "</int64>\n"
        buf += ws + "</Vehicles>\n"
        buf += ws + "<TaskList>\n"
        for x in self.TaskList:
            buf += ws + "<int64>" + str(x) + "</int64>\n"
        buf += ws + "</TaskList>\n"
        buf += ws + "<TaskRelationships>" + str(self.TaskRelationships) + "</TaskRelationships>\n"
        buf += ws + "<InterTaskPercentage>\n"
        for x in self.InterTaskPercentage:
            buf += ws + "<real32>" + str(x) + "</real32>\n"
        buf += ws + "</InterTaskPercentage>\n"
        buf += ws + "<OperatingRegion>" + str(self.OperatingRegion) + "</OperatingRegion>\n"

        return buf
        
