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



class BatchRoutePlanRequest(LMCPObject.LMCPObject):

    def __init__(self):

        self.LMCP_TYPE = 9
        self.SERIES_NAME = "IMPACT"
        self.FULL_LMCP_TYPE_NAME = "afrl.impact.BatchRoutePlanRequest"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 5281966179208134656
        self.SERIES_VERSION = 14

        #Define message fields
        self.RequestID = 0   #int64
        self.Vehicles = []   #int64
        self.TaskList = []   #int64
        self.OperatingRegion = 0   #int64
        self.ComputeTaskToTaskTiming = False   #bool
        self.ComputeInterTaskToTaskTiming = False   #bool
        self.InterTaskPercentage = []   #real32


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
        buffer.extend(struct.pack(">q", self.OperatingRegion))
        boolChar = 1 if self.ComputeTaskToTaskTiming == True else 0
        buffer.extend(struct.pack(">B",boolChar))
        boolChar = 1 if self.ComputeInterTaskToTaskTiming == True else 0
        buffer.extend(struct.pack(">B",boolChar))
        buffer.extend(struct.pack(">H", len(self.InterTaskPercentage) ))
        for x in self.InterTaskPercentage:
            buffer.extend(struct.pack(">f", x ))

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
        self.OperatingRegion = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        boolChar = struct.unpack_from(">B", buffer, _pos)[0]
        self.ComputeTaskToTaskTiming = True if boolChar == 1 else False
        _pos += 1
        boolChar = struct.unpack_from(">B", buffer, _pos)[0]
        self.ComputeInterTaskToTaskTiming = True if boolChar == 1 else False
        _pos += 1
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.InterTaskPercentage = [None] * _arraylen
        if _arraylen > 0:
            self.InterTaskPercentage = struct.unpack_from(">" + repr(_arraylen) + "f", buffer, _pos )
            _pos += 4 * _arraylen
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
                elif e.localName == "OperatingRegion" and len(e.childNodes) > 0 :
                    self.OperatingRegion = int(e.childNodes[0].nodeValue)
                elif e.localName == "ComputeTaskToTaskTiming" and len(e.childNodes) > 0 :
                    self.ComputeTaskToTaskTiming = e.childNodes[0].nodeValue.lower() == 'true'
                elif e.localName == "ComputeInterTaskToTaskTiming" and len(e.childNodes) > 0 :
                    self.ComputeInterTaskToTaskTiming = e.childNodes[0].nodeValue.lower() == 'true'
                elif e.localName == "InterTaskPercentage" and len(e.childNodes) > 0 :
                    self.InterTaskPercentage = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.InterTaskPercentage.append( float(c.childNodes[0].nodeValue) )

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
            elif key == "OperatingRegion":
                self.OperatingRegion = d[key]
            elif key == "ComputeTaskToTaskTiming":
                self.ComputeTaskToTaskTiming = d[key]
            elif key == "ComputeInterTaskToTaskTiming":
                self.ComputeInterTaskToTaskTiming = d[key]
            elif key == "InterTaskPercentage":
                self.InterTaskPercentage = []
                for c in d[key]:
                    self.InterTaskPercentage.append( c )

        return

    def get_RequestID(self):
        return self.RequestID

    def set_RequestID(self, value):
        self.RequestID = int( value )

    def get_Vehicles(self):
        return self.Vehicles

    def get_TaskList(self):
        return self.TaskList

    def get_OperatingRegion(self):
        return self.OperatingRegion

    def set_OperatingRegion(self, value):
        self.OperatingRegion = int( value )

    def get_ComputeTaskToTaskTiming(self):
        return self.ComputeTaskToTaskTiming

    def set_ComputeTaskToTaskTiming(self, value):
        self.ComputeTaskToTaskTiming = bool( value )

    def get_ComputeInterTaskToTaskTiming(self):
        return self.ComputeInterTaskToTaskTiming

    def set_ComputeInterTaskToTaskTiming(self, value):
        self.ComputeInterTaskToTaskTiming = bool( value )

    def get_InterTaskPercentage(self):
        return self.InterTaskPercentage



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = LMCPObject.LMCPObject.toString(self)
        buf += "From BatchRoutePlanRequest:\n"
        buf +=    "RequestID = " + str( self.RequestID ) + "\n" 
        buf +=    "Vehicles = " + str( self.Vehicles ) + "\n" 
        buf +=    "TaskList = " + str( self.TaskList ) + "\n" 
        buf +=    "OperatingRegion = " + str( self.OperatingRegion ) + "\n" 
        buf +=    "ComputeTaskToTaskTiming = " + str( self.ComputeTaskToTaskTiming ) + "\n" 
        buf +=    "ComputeInterTaskToTaskTiming = " + str( self.ComputeInterTaskToTaskTiming ) + "\n" 
        buf +=    "InterTaskPercentage = " + str( self.InterTaskPercentage ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "IMPACT") or (len("IMPACT") == 0): # this should never happen
        	# Checks for "IMPACT" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/BatchRoutePlanRequest")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("IMPACT" + "/BatchRoutePlanRequest")
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
        d['OperatingRegion'] = self.OperatingRegion
        d['ComputeTaskToTaskTiming'] = self.ComputeTaskToTaskTiming
        d['ComputeInterTaskToTaskTiming'] = self.ComputeInterTaskToTaskTiming
        d['InterTaskPercentage'] = []
        for x in self.InterTaskPercentage:
            d['InterTaskPercentage'].append(x)

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
        str = ws + '<BatchRoutePlanRequest Series="IMPACT" >\n';
        #str +=LMCPObject.LMCPObject.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</BatchRoutePlanRequest>\n";
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
        buf += ws + "<OperatingRegion>" + str(self.OperatingRegion) + "</OperatingRegion>\n"
        buf += ws + "<ComputeTaskToTaskTiming>" + ('True' if self.ComputeTaskToTaskTiming else 'False') + "</ComputeTaskToTaskTiming>\n"
        buf += ws + "<ComputeInterTaskToTaskTiming>" + ('True' if self.ComputeInterTaskToTaskTiming else 'False') + "</ComputeInterTaskToTaskTiming>\n"
        buf += ws + "<InterTaskPercentage>\n"
        for x in self.InterTaskPercentage:
            buf += ws + "<real32>" + str(x) + "</real32>\n"
        buf += ws + "</InterTaskPercentage>\n"

        return buf
        
