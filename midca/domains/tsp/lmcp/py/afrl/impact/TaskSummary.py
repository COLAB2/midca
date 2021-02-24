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

from midca.domains.tsp.lmcp.py.afrl.impact import VehicleSummary


class TaskSummary(LMCPObject.LMCPObject):

    def __init__(self):

        self.LMCP_TYPE = 14
        self.SERIES_NAME = "IMPACT"
        self.FULL_LMCP_TYPE_NAME = "afrl.impact.TaskSummary"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 5281966179208134656
        self.SERIES_VERSION = 14

        #Define message fields
        self.TaskID = 0   #int64
        self.PerformingVehicles = []   #VehicleSummary
        self.BestEffort = 100.0   #real32


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(LMCPObject.LMCPObject.pack(self))
        buffer.extend(struct.pack(">q", self.TaskID))
        buffer.extend(struct.pack(">H", len(self.PerformingVehicles) ))
        for x in self.PerformingVehicles:
           buffer.extend(struct.pack("B", x != None ))
           if x != None:
               buffer.extend(struct.pack(">q", x.SERIES_NAME_ID))
               buffer.extend(struct.pack(">I", x.LMCP_TYPE))
               buffer.extend(struct.pack(">H", x.SERIES_VERSION))
               buffer.extend(x.pack())
        buffer.extend(struct.pack(">f", self.BestEffort))

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
        self.PerformingVehicles = [None] * _arraylen
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
                self.PerformingVehicles[x] = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
                _pos = self.PerformingVehicles[x].unpack(buffer, _pos)
            else:
                self.PerformingVehicles[x] = None
        self.BestEffort = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        LMCPObject.LMCPObject.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "TaskID" and len(e.childNodes) > 0 :
                    self.TaskID = int(e.childNodes[0].nodeValue)
                elif e.localName == "PerformingVehicles" and len(e.childNodes) > 0 :
                    self.PerformingVehicles = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            obj = seriesFactory.createObjectByName(c.getAttribute('Series'), c.localName)
                            if obj != None:
                                obj.unpackFromXMLNode(c, seriesFactory)
                                self.PerformingVehicles.append(obj)
                elif e.localName == "BestEffort" and len(e.childNodes) > 0 :
                    self.BestEffort = float(e.childNodes[0].nodeValue)

        return

    def unpackFromDict(self, d, seriesFactory):
        LMCPObject.LMCPObject.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "TaskID":
                self.TaskID = d[key]
            elif key == "PerformingVehicles":
                self.PerformingVehicles = []
                for c in d[key]:
                    obj = seriesFactory.unpackFromDict(c)
                    if obj != None:
                        self.PerformingVehicles.append(obj)
            elif key == "BestEffort":
                self.BestEffort = d[key]

        return

    def get_TaskID(self):
        return self.TaskID

    def set_TaskID(self, value):
        self.TaskID = int( value )

    def get_PerformingVehicles(self):
        return self.PerformingVehicles

    def get_BestEffort(self):
        return self.BestEffort

    def set_BestEffort(self, value):
        self.BestEffort = float( value )



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = LMCPObject.LMCPObject.toString(self)
        buf += "From TaskSummary:\n"
        buf +=    "TaskID = " + str( self.TaskID ) + "\n" 
        buf +=    "PerformingVehicles = " + str( self.PerformingVehicles ) + "\n" 
        buf +=    "BestEffort = " + str( self.BestEffort ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "IMPACT") or (len("IMPACT") == 0): # this should never happen
        	# Checks for "IMPACT" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/TaskSummary")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("IMPACT" + "/TaskSummary")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        LMCPObject.LMCPObject.toDictMembers(self, d)
        d['TaskID'] = self.TaskID
        d['PerformingVehicles'] = []
        for x in self.PerformingVehicles:
            if x == None:
                d['PerformingVehicles'].append(None)
            else:
                d['PerformingVehicles'].append(x.toDict())
        d['BestEffort'] = self.BestEffort

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
        str = ws + '<TaskSummary Series="IMPACT" >\n';
        #str +=LMCPObject.LMCPObject.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</TaskSummary>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += LMCPObject.LMCPObject.toXMLMembersStr(self, ws)
        buf += ws + "<TaskID>" + str(self.TaskID) + "</TaskID>\n"
        buf += ws + "<PerformingVehicles>\n"
        for x in self.PerformingVehicles:
            if x == None:
                buf += ws + "    <null/>\n"
            else:
                buf += x.toXMLStr(ws + "    ") 
        buf += ws + "</PerformingVehicles>\n"
        buf += ws + "<BestEffort>" + str(self.BestEffort) + "</BestEffort>\n"

        return buf
        
