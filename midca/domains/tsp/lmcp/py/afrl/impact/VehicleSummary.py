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

from midca.domains.tsp.lmcp.py.afrl.cmasi import Waypoint


class VehicleSummary(LMCPObject.LMCPObject):

    def __init__(self):

        self.LMCP_TYPE = 15
        self.SERIES_NAME = "IMPACT"
        self.FULL_LMCP_TYPE_NAME = "afrl.impact.VehicleSummary"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 5281966179208134656
        self.SERIES_VERSION = 14

        #Define message fields
        self.VehicleID = 0   #int64
        self.DestinationTaskID = 0   #int64
        self.InitialTaskID = 0   #int64
        self.InitialTaskPercentage = 0   #real32
        self.EstimateTimeToTaskPercentage = 0   #int64
        self.TimeToArrive = 0   #int64
        self.TimeOnTask = 0   #int64
        self.EnergyRemaining = 0   #real32
        self.BeyondCommRange = False   #bool
        self.ConflictsWithROZ = False   #bool
        self.ROZIDs = []   #int64
        self.WaypointList = []   #Waypoint
        self.FirstWaypoint = 0   #int64


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(LMCPObject.LMCPObject.pack(self))
        buffer.extend(struct.pack(">q", self.VehicleID))
        buffer.extend(struct.pack(">q", self.DestinationTaskID))
        buffer.extend(struct.pack(">q", self.InitialTaskID))
        buffer.extend(struct.pack(">f", self.InitialTaskPercentage))
        buffer.extend(struct.pack(">q", self.EstimateTimeToTaskPercentage))
        buffer.extend(struct.pack(">q", self.TimeToArrive))
        buffer.extend(struct.pack(">q", self.TimeOnTask))
        buffer.extend(struct.pack(">f", self.EnergyRemaining))
        boolChar = 1 if self.BeyondCommRange == True else 0
        buffer.extend(struct.pack(">B",boolChar))
        boolChar = 1 if self.ConflictsWithROZ == True else 0
        buffer.extend(struct.pack(">B",boolChar))
        buffer.extend(struct.pack(">H", len(self.ROZIDs) ))
        for x in self.ROZIDs:
            buffer.extend(struct.pack(">q", x ))
        buffer.extend(struct.pack(">H", len(self.WaypointList) ))
        for x in self.WaypointList:
           buffer.extend(struct.pack("B", x != None ))
           if x != None:
               buffer.extend(struct.pack(">q", x.SERIES_NAME_ID))
               buffer.extend(struct.pack(">I", x.LMCP_TYPE))
               buffer.extend(struct.pack(">H", x.SERIES_VERSION))
               buffer.extend(x.pack())
        buffer.extend(struct.pack(">q", self.FirstWaypoint))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = LMCPObject.LMCPObject.unpack(self, buffer, _pos)
        self.VehicleID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.DestinationTaskID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.InitialTaskID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.InitialTaskPercentage = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.EstimateTimeToTaskPercentage = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.TimeToArrive = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.TimeOnTask = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.EnergyRemaining = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        boolChar = struct.unpack_from(">B", buffer, _pos)[0]
        self.BeyondCommRange = True if boolChar == 1 else False
        _pos += 1
        boolChar = struct.unpack_from(">B", buffer, _pos)[0]
        self.ConflictsWithROZ = True if boolChar == 1 else False
        _pos += 1
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.ROZIDs = [None] * _arraylen
        if _arraylen > 0:
            self.ROZIDs = struct.unpack_from(">" + repr(_arraylen) + "q", buffer, _pos )
            _pos += 8 * _arraylen
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.WaypointList = [None] * _arraylen
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
                self.WaypointList[x] = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
                _pos = self.WaypointList[x].unpack(buffer, _pos)
            else:
                self.WaypointList[x] = None
        self.FirstWaypoint = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        LMCPObject.LMCPObject.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "VehicleID" and len(e.childNodes) > 0 :
                    self.VehicleID = int(e.childNodes[0].nodeValue)
                elif e.localName == "DestinationTaskID" and len(e.childNodes) > 0 :
                    self.DestinationTaskID = int(e.childNodes[0].nodeValue)
                elif e.localName == "InitialTaskID" and len(e.childNodes) > 0 :
                    self.InitialTaskID = int(e.childNodes[0].nodeValue)
                elif e.localName == "InitialTaskPercentage" and len(e.childNodes) > 0 :
                    self.InitialTaskPercentage = float(e.childNodes[0].nodeValue)
                elif e.localName == "EstimateTimeToTaskPercentage" and len(e.childNodes) > 0 :
                    self.EstimateTimeToTaskPercentage = int(e.childNodes[0].nodeValue)
                elif e.localName == "TimeToArrive" and len(e.childNodes) > 0 :
                    self.TimeToArrive = int(e.childNodes[0].nodeValue)
                elif e.localName == "TimeOnTask" and len(e.childNodes) > 0 :
                    self.TimeOnTask = int(e.childNodes[0].nodeValue)
                elif e.localName == "EnergyRemaining" and len(e.childNodes) > 0 :
                    self.EnergyRemaining = float(e.childNodes[0].nodeValue)
                elif e.localName == "BeyondCommRange" and len(e.childNodes) > 0 :
                    self.BeyondCommRange = e.childNodes[0].nodeValue.lower() == 'true'
                elif e.localName == "ConflictsWithROZ" and len(e.childNodes) > 0 :
                    self.ConflictsWithROZ = e.childNodes[0].nodeValue.lower() == 'true'
                elif e.localName == "ROZIDs" and len(e.childNodes) > 0 :
                    self.ROZIDs = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.ROZIDs.append( int(c.childNodes[0].nodeValue) )
                elif e.localName == "WaypointList" and len(e.childNodes) > 0 :
                    self.WaypointList = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            obj = seriesFactory.createObjectByName(c.getAttribute('Series'), c.localName)
                            if obj != None:
                                obj.unpackFromXMLNode(c, seriesFactory)
                                self.WaypointList.append(obj)
                elif e.localName == "FirstWaypoint" and len(e.childNodes) > 0 :
                    self.FirstWaypoint = int(e.childNodes[0].nodeValue)

        return

    def unpackFromDict(self, d, seriesFactory):
        LMCPObject.LMCPObject.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "VehicleID":
                self.VehicleID = d[key]
            elif key == "DestinationTaskID":
                self.DestinationTaskID = d[key]
            elif key == "InitialTaskID":
                self.InitialTaskID = d[key]
            elif key == "InitialTaskPercentage":
                self.InitialTaskPercentage = d[key]
            elif key == "EstimateTimeToTaskPercentage":
                self.EstimateTimeToTaskPercentage = d[key]
            elif key == "TimeToArrive":
                self.TimeToArrive = d[key]
            elif key == "TimeOnTask":
                self.TimeOnTask = d[key]
            elif key == "EnergyRemaining":
                self.EnergyRemaining = d[key]
            elif key == "BeyondCommRange":
                self.BeyondCommRange = d[key]
            elif key == "ConflictsWithROZ":
                self.ConflictsWithROZ = d[key]
            elif key == "ROZIDs":
                self.ROZIDs = []
                for c in d[key]:
                    self.ROZIDs.append( c )
            elif key == "WaypointList":
                self.WaypointList = []
                for c in d[key]:
                    obj = seriesFactory.unpackFromDict(c)
                    if obj != None:
                        self.WaypointList.append(obj)
            elif key == "FirstWaypoint":
                self.FirstWaypoint = d[key]

        return

    def get_VehicleID(self):
        return self.VehicleID

    def set_VehicleID(self, value):
        self.VehicleID = int( value )

    def get_DestinationTaskID(self):
        return self.DestinationTaskID

    def set_DestinationTaskID(self, value):
        self.DestinationTaskID = int( value )

    def get_InitialTaskID(self):
        return self.InitialTaskID

    def set_InitialTaskID(self, value):
        self.InitialTaskID = int( value )

    def get_InitialTaskPercentage(self):
        return self.InitialTaskPercentage

    def set_InitialTaskPercentage(self, value):
        self.InitialTaskPercentage = float( value )

    def get_EstimateTimeToTaskPercentage(self):
        return self.EstimateTimeToTaskPercentage

    def set_EstimateTimeToTaskPercentage(self, value):
        self.EstimateTimeToTaskPercentage = int( value )

    def get_TimeToArrive(self):
        return self.TimeToArrive

    def set_TimeToArrive(self, value):
        self.TimeToArrive = int( value )

    def get_TimeOnTask(self):
        return self.TimeOnTask

    def set_TimeOnTask(self, value):
        self.TimeOnTask = int( value )

    def get_EnergyRemaining(self):
        return self.EnergyRemaining

    def set_EnergyRemaining(self, value):
        self.EnergyRemaining = float( value )

    def get_BeyondCommRange(self):
        return self.BeyondCommRange

    def set_BeyondCommRange(self, value):
        self.BeyondCommRange = bool( value )

    def get_ConflictsWithROZ(self):
        return self.ConflictsWithROZ

    def set_ConflictsWithROZ(self, value):
        self.ConflictsWithROZ = bool( value )

    def get_ROZIDs(self):
        return self.ROZIDs

    def get_WaypointList(self):
        return self.WaypointList

    def get_FirstWaypoint(self):
        return self.FirstWaypoint

    def set_FirstWaypoint(self, value):
        self.FirstWaypoint = int( value )



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = LMCPObject.LMCPObject.toString(self)
        buf += "From VehicleSummary:\n"
        buf +=    "VehicleID = " + str( self.VehicleID ) + "\n" 
        buf +=    "DestinationTaskID = " + str( self.DestinationTaskID ) + "\n" 
        buf +=    "InitialTaskID = " + str( self.InitialTaskID ) + "\n" 
        buf +=    "InitialTaskPercentage = " + str( self.InitialTaskPercentage ) + "\n" 
        buf +=    "EstimateTimeToTaskPercentage = " + str( self.EstimateTimeToTaskPercentage ) + "\n" 
        buf +=    "TimeToArrive = " + str( self.TimeToArrive ) + "\n" 
        buf +=    "TimeOnTask = " + str( self.TimeOnTask ) + "\n" 
        buf +=    "EnergyRemaining = " + str( self.EnergyRemaining ) + "\n" 
        buf +=    "BeyondCommRange = " + str( self.BeyondCommRange ) + "\n" 
        buf +=    "ConflictsWithROZ = " + str( self.ConflictsWithROZ ) + "\n" 
        buf +=    "ROZIDs = " + str( self.ROZIDs ) + "\n" 
        buf +=    "WaypointList = " + str( self.WaypointList ) + "\n" 
        buf +=    "FirstWaypoint = " + str( self.FirstWaypoint ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "IMPACT") or (len("IMPACT") == 0): # this should never happen
        	# Checks for "IMPACT" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/VehicleSummary")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("IMPACT" + "/VehicleSummary")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        LMCPObject.LMCPObject.toDictMembers(self, d)
        d['VehicleID'] = self.VehicleID
        d['DestinationTaskID'] = self.DestinationTaskID
        d['InitialTaskID'] = self.InitialTaskID
        d['InitialTaskPercentage'] = self.InitialTaskPercentage
        d['EstimateTimeToTaskPercentage'] = self.EstimateTimeToTaskPercentage
        d['TimeToArrive'] = self.TimeToArrive
        d['TimeOnTask'] = self.TimeOnTask
        d['EnergyRemaining'] = self.EnergyRemaining
        d['BeyondCommRange'] = self.BeyondCommRange
        d['ConflictsWithROZ'] = self.ConflictsWithROZ
        d['ROZIDs'] = []
        for x in self.ROZIDs:
            d['ROZIDs'].append(x)
        d['WaypointList'] = []
        for x in self.WaypointList:
            if x == None:
                d['WaypointList'].append(None)
            else:
                d['WaypointList'].append(x.toDict())
        d['FirstWaypoint'] = self.FirstWaypoint

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
        str = ws + '<VehicleSummary Series="IMPACT" >\n';
        #str +=LMCPObject.LMCPObject.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</VehicleSummary>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += LMCPObject.LMCPObject.toXMLMembersStr(self, ws)
        buf += ws + "<VehicleID>" + str(self.VehicleID) + "</VehicleID>\n"
        buf += ws + "<DestinationTaskID>" + str(self.DestinationTaskID) + "</DestinationTaskID>\n"
        buf += ws + "<InitialTaskID>" + str(self.InitialTaskID) + "</InitialTaskID>\n"
        buf += ws + "<InitialTaskPercentage>" + str(self.InitialTaskPercentage) + "</InitialTaskPercentage>\n"
        buf += ws + "<EstimateTimeToTaskPercentage>" + str(self.EstimateTimeToTaskPercentage) + "</EstimateTimeToTaskPercentage>\n"
        buf += ws + "<TimeToArrive>" + str(self.TimeToArrive) + "</TimeToArrive>\n"
        buf += ws + "<TimeOnTask>" + str(self.TimeOnTask) + "</TimeOnTask>\n"
        buf += ws + "<EnergyRemaining>" + str(self.EnergyRemaining) + "</EnergyRemaining>\n"
        buf += ws + "<BeyondCommRange>" + ('True' if self.BeyondCommRange else 'False') + "</BeyondCommRange>\n"
        buf += ws + "<ConflictsWithROZ>" + ('True' if self.ConflictsWithROZ else 'False') + "</ConflictsWithROZ>\n"
        buf += ws + "<ROZIDs>\n"
        for x in self.ROZIDs:
            buf += ws + "<int64>" + str(x) + "</int64>\n"
        buf += ws + "</ROZIDs>\n"
        buf += ws + "<WaypointList>\n"
        for x in self.WaypointList:
            if x == None:
                buf += ws + "    <null/>\n"
            else:
                buf += x.toXMLStr(ws + "    ") 
        buf += ws + "</WaypointList>\n"
        buf += ws + "<FirstWaypoint>" + str(self.FirstWaypoint) + "</FirstWaypoint>\n"

        return buf
        
