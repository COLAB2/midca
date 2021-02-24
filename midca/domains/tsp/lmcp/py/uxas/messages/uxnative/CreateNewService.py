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

from midca.domains.tsp.lmcp.py.afrl.cmasi import EntityConfiguration
from midca.domains.tsp.lmcp.py.afrl.cmasi import EntityState
from midca.domains.tsp.lmcp.py.afrl.cmasi import MissionCommand
from midca.domains.tsp.lmcp.py.afrl.impact import AreaOfInterest
from midca.domains.tsp.lmcp.py.afrl.impact import LineOfInterest
from midca.domains.tsp.lmcp.py.afrl.impact import PointOfInterest
from midca.domains.tsp.lmcp.py.afrl.cmasi import KeepInZone
from midca.domains.tsp.lmcp.py.afrl.cmasi import KeepOutZone
from midca.domains.tsp.lmcp.py.afrl.cmasi import OperatingRegion


class CreateNewService(LMCPObject.LMCPObject):

    def __init__(self):

        self.LMCP_TYPE = 3
        self.SERIES_NAME = "UXNATIVE"
        self.FULL_LMCP_TYPE_NAME = "uxas.messages.uxnative.CreateNewService"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 6149751333668345413
        self.SERIES_VERSION = 9

        #Define message fields
        self.ServiceID = 0   #int64
        self.XmlConfiguration = ""   #string
        self.EntityConfigurations = []   #EntityConfiguration
        self.EntityStates = []   #EntityState
        self.MissionCommands = []   #MissionCommand
        self.Areas = []   #AreaOfInterest
        self.Lines = []   #LineOfInterest
        self.Points = []   #PointOfInterest
        self.KeepInZones = []   #KeepInZone
        self.KeepOutZones = []   #KeepOutZone
        self.OperatingRegions = []   #OperatingRegion


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(LMCPObject.LMCPObject.pack(self))
        buffer.extend(struct.pack(">q", self.ServiceID))
        buffer.extend(struct.pack(">H", len(self.XmlConfiguration) ))
        if len(self.XmlConfiguration) > 0:
            if (sys.version_info > (3, 0)):
                buffer.extend(struct.pack( repr(len(self.XmlConfiguration)) + "s", self.XmlConfiguration.encode('utf-8')))
            else:
                buffer.extend(struct.pack( repr(len(self.XmlConfiguration)) + "s", self.XmlConfiguration))
        buffer.extend(struct.pack(">H", len(self.EntityConfigurations) ))
        for x in self.EntityConfigurations:
           buffer.extend(struct.pack("B", x != None ))
           if x != None:
               buffer.extend(struct.pack(">q", x.SERIES_NAME_ID))
               buffer.extend(struct.pack(">I", x.LMCP_TYPE))
               buffer.extend(struct.pack(">H", x.SERIES_VERSION))
               buffer.extend(x.pack())
        buffer.extend(struct.pack(">H", len(self.EntityStates) ))
        for x in self.EntityStates:
           buffer.extend(struct.pack("B", x != None ))
           if x != None:
               buffer.extend(struct.pack(">q", x.SERIES_NAME_ID))
               buffer.extend(struct.pack(">I", x.LMCP_TYPE))
               buffer.extend(struct.pack(">H", x.SERIES_VERSION))
               buffer.extend(x.pack())
        buffer.extend(struct.pack(">H", len(self.MissionCommands) ))
        for x in self.MissionCommands:
           buffer.extend(struct.pack("B", x != None ))
           if x != None:
               buffer.extend(struct.pack(">q", x.SERIES_NAME_ID))
               buffer.extend(struct.pack(">I", x.LMCP_TYPE))
               buffer.extend(struct.pack(">H", x.SERIES_VERSION))
               buffer.extend(x.pack())
        buffer.extend(struct.pack(">H", len(self.Areas) ))
        for x in self.Areas:
           buffer.extend(struct.pack("B", x != None ))
           if x != None:
               buffer.extend(struct.pack(">q", x.SERIES_NAME_ID))
               buffer.extend(struct.pack(">I", x.LMCP_TYPE))
               buffer.extend(struct.pack(">H", x.SERIES_VERSION))
               buffer.extend(x.pack())
        buffer.extend(struct.pack(">H", len(self.Lines) ))
        for x in self.Lines:
           buffer.extend(struct.pack("B", x != None ))
           if x != None:
               buffer.extend(struct.pack(">q", x.SERIES_NAME_ID))
               buffer.extend(struct.pack(">I", x.LMCP_TYPE))
               buffer.extend(struct.pack(">H", x.SERIES_VERSION))
               buffer.extend(x.pack())
        buffer.extend(struct.pack(">H", len(self.Points) ))
        for x in self.Points:
           buffer.extend(struct.pack("B", x != None ))
           if x != None:
               buffer.extend(struct.pack(">q", x.SERIES_NAME_ID))
               buffer.extend(struct.pack(">I", x.LMCP_TYPE))
               buffer.extend(struct.pack(">H", x.SERIES_VERSION))
               buffer.extend(x.pack())
        buffer.extend(struct.pack(">H", len(self.KeepInZones) ))
        for x in self.KeepInZones:
           buffer.extend(struct.pack("B", x != None ))
           if x != None:
               buffer.extend(struct.pack(">q", x.SERIES_NAME_ID))
               buffer.extend(struct.pack(">I", x.LMCP_TYPE))
               buffer.extend(struct.pack(">H", x.SERIES_VERSION))
               buffer.extend(x.pack())
        buffer.extend(struct.pack(">H", len(self.KeepOutZones) ))
        for x in self.KeepOutZones:
           buffer.extend(struct.pack("B", x != None ))
           if x != None:
               buffer.extend(struct.pack(">q", x.SERIES_NAME_ID))
               buffer.extend(struct.pack(">I", x.LMCP_TYPE))
               buffer.extend(struct.pack(">H", x.SERIES_VERSION))
               buffer.extend(x.pack())
        buffer.extend(struct.pack(">H", len(self.OperatingRegions) ))
        for x in self.OperatingRegions:
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
        self.ServiceID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        _strlen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        if _strlen > 0:
            if (sys.version_info > (3, 0)):
                self.XmlConfiguration = struct.unpack_from( repr(_strlen) + "s", buffer, _pos )[0].decode('utf-8')
            else:
                 self.XmlConfiguration = struct.unpack_from( repr(_strlen) + "s", buffer, _pos )[0]
            _pos += _strlen
        else:
             self.XmlConfiguration = ""
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.EntityConfigurations = [None] * _arraylen
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
                self.EntityConfigurations[x] = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
                _pos = self.EntityConfigurations[x].unpack(buffer, _pos)
            else:
                self.EntityConfigurations[x] = None
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.EntityStates = [None] * _arraylen
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
                self.EntityStates[x] = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
                _pos = self.EntityStates[x].unpack(buffer, _pos)
            else:
                self.EntityStates[x] = None
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.MissionCommands = [None] * _arraylen
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
                self.MissionCommands[x] = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
                _pos = self.MissionCommands[x].unpack(buffer, _pos)
            else:
                self.MissionCommands[x] = None
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.Areas = [None] * _arraylen
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
                self.Areas[x] = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
                _pos = self.Areas[x].unpack(buffer, _pos)
            else:
                self.Areas[x] = None
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.Lines = [None] * _arraylen
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
                self.Lines[x] = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
                _pos = self.Lines[x].unpack(buffer, _pos)
            else:
                self.Lines[x] = None
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.Points = [None] * _arraylen
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
                self.Points[x] = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
                _pos = self.Points[x].unpack(buffer, _pos)
            else:
                self.Points[x] = None
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.KeepInZones = [None] * _arraylen
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
                self.KeepInZones[x] = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
                _pos = self.KeepInZones[x].unpack(buffer, _pos)
            else:
                self.KeepInZones[x] = None
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.KeepOutZones = [None] * _arraylen
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
                self.KeepOutZones[x] = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
                _pos = self.KeepOutZones[x].unpack(buffer, _pos)
            else:
                self.KeepOutZones[x] = None
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.OperatingRegions = [None] * _arraylen
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
                self.OperatingRegions[x] = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
                _pos = self.OperatingRegions[x].unpack(buffer, _pos)
            else:
                self.OperatingRegions[x] = None
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        LMCPObject.LMCPObject.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "ServiceID" and len(e.childNodes) > 0 :
                    self.ServiceID = int(e.childNodes[0].nodeValue)
                elif e.localName == "XmlConfiguration" :
                    self.XmlConfiguration = str(e.childNodes[0].nodeValue) if len(e.childNodes) > 0 else ""
                elif e.localName == "EntityConfigurations" and len(e.childNodes) > 0 :
                    self.EntityConfigurations = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            obj = seriesFactory.createObjectByName(c.getAttribute('Series'), c.localName)
                            if obj != None:
                                obj.unpackFromXMLNode(c, seriesFactory)
                                self.EntityConfigurations.append(obj)
                elif e.localName == "EntityStates" and len(e.childNodes) > 0 :
                    self.EntityStates = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            obj = seriesFactory.createObjectByName(c.getAttribute('Series'), c.localName)
                            if obj != None:
                                obj.unpackFromXMLNode(c, seriesFactory)
                                self.EntityStates.append(obj)
                elif e.localName == "MissionCommands" and len(e.childNodes) > 0 :
                    self.MissionCommands = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            obj = seriesFactory.createObjectByName(c.getAttribute('Series'), c.localName)
                            if obj != None:
                                obj.unpackFromXMLNode(c, seriesFactory)
                                self.MissionCommands.append(obj)
                elif e.localName == "Areas" and len(e.childNodes) > 0 :
                    self.Areas = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            obj = seriesFactory.createObjectByName(c.getAttribute('Series'), c.localName)
                            if obj != None:
                                obj.unpackFromXMLNode(c, seriesFactory)
                                self.Areas.append(obj)
                elif e.localName == "Lines" and len(e.childNodes) > 0 :
                    self.Lines = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            obj = seriesFactory.createObjectByName(c.getAttribute('Series'), c.localName)
                            if obj != None:
                                obj.unpackFromXMLNode(c, seriesFactory)
                                self.Lines.append(obj)
                elif e.localName == "Points" and len(e.childNodes) > 0 :
                    self.Points = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            obj = seriesFactory.createObjectByName(c.getAttribute('Series'), c.localName)
                            if obj != None:
                                obj.unpackFromXMLNode(c, seriesFactory)
                                self.Points.append(obj)
                elif e.localName == "KeepInZones" and len(e.childNodes) > 0 :
                    self.KeepInZones = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            obj = seriesFactory.createObjectByName(c.getAttribute('Series'), c.localName)
                            if obj != None:
                                obj.unpackFromXMLNode(c, seriesFactory)
                                self.KeepInZones.append(obj)
                elif e.localName == "KeepOutZones" and len(e.childNodes) > 0 :
                    self.KeepOutZones = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            obj = seriesFactory.createObjectByName(c.getAttribute('Series'), c.localName)
                            if obj != None:
                                obj.unpackFromXMLNode(c, seriesFactory)
                                self.KeepOutZones.append(obj)
                elif e.localName == "OperatingRegions" and len(e.childNodes) > 0 :
                    self.OperatingRegions = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            obj = seriesFactory.createObjectByName(c.getAttribute('Series'), c.localName)
                            if obj != None:
                                obj.unpackFromXMLNode(c, seriesFactory)
                                self.OperatingRegions.append(obj)

        return

    def unpackFromDict(self, d, seriesFactory):
        LMCPObject.LMCPObject.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "ServiceID":
                self.ServiceID = d[key]
            elif key == "XmlConfiguration":
                self.XmlConfiguration = d[key]
            elif key == "EntityConfigurations":
                self.EntityConfigurations = []
                for c in d[key]:
                    obj = seriesFactory.unpackFromDict(c)
                    if obj != None:
                        self.EntityConfigurations.append(obj)
            elif key == "EntityStates":
                self.EntityStates = []
                for c in d[key]:
                    obj = seriesFactory.unpackFromDict(c)
                    if obj != None:
                        self.EntityStates.append(obj)
            elif key == "MissionCommands":
                self.MissionCommands = []
                for c in d[key]:
                    obj = seriesFactory.unpackFromDict(c)
                    if obj != None:
                        self.MissionCommands.append(obj)
            elif key == "Areas":
                self.Areas = []
                for c in d[key]:
                    obj = seriesFactory.unpackFromDict(c)
                    if obj != None:
                        self.Areas.append(obj)
            elif key == "Lines":
                self.Lines = []
                for c in d[key]:
                    obj = seriesFactory.unpackFromDict(c)
                    if obj != None:
                        self.Lines.append(obj)
            elif key == "Points":
                self.Points = []
                for c in d[key]:
                    obj = seriesFactory.unpackFromDict(c)
                    if obj != None:
                        self.Points.append(obj)
            elif key == "KeepInZones":
                self.KeepInZones = []
                for c in d[key]:
                    obj = seriesFactory.unpackFromDict(c)
                    if obj != None:
                        self.KeepInZones.append(obj)
            elif key == "KeepOutZones":
                self.KeepOutZones = []
                for c in d[key]:
                    obj = seriesFactory.unpackFromDict(c)
                    if obj != None:
                        self.KeepOutZones.append(obj)
            elif key == "OperatingRegions":
                self.OperatingRegions = []
                for c in d[key]:
                    obj = seriesFactory.unpackFromDict(c)
                    if obj != None:
                        self.OperatingRegions.append(obj)

        return

    def get_ServiceID(self):
        return self.ServiceID

    def set_ServiceID(self, value):
        self.ServiceID = int( value )

    def get_XmlConfiguration(self):
        return self.XmlConfiguration

    def set_XmlConfiguration(self, value):
        self.XmlConfiguration = str( value )

    def get_EntityConfigurations(self):
        return self.EntityConfigurations

    def get_EntityStates(self):
        return self.EntityStates

    def get_MissionCommands(self):
        return self.MissionCommands

    def get_Areas(self):
        return self.Areas

    def get_Lines(self):
        return self.Lines

    def get_Points(self):
        return self.Points

    def get_KeepInZones(self):
        return self.KeepInZones

    def get_KeepOutZones(self):
        return self.KeepOutZones

    def get_OperatingRegions(self):
        return self.OperatingRegions



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = LMCPObject.LMCPObject.toString(self)
        buf += "From CreateNewService:\n"
        buf +=    "ServiceID = " + str( self.ServiceID ) + "\n" 
        buf +=    "XmlConfiguration = " + str( self.XmlConfiguration ) + "\n" 
        buf +=    "EntityConfigurations = " + str( self.EntityConfigurations ) + "\n" 
        buf +=    "EntityStates = " + str( self.EntityStates ) + "\n" 
        buf +=    "MissionCommands = " + str( self.MissionCommands ) + "\n" 
        buf +=    "Areas = " + str( self.Areas ) + "\n" 
        buf +=    "Lines = " + str( self.Lines ) + "\n" 
        buf +=    "Points = " + str( self.Points ) + "\n" 
        buf +=    "KeepInZones = " + str( self.KeepInZones ) + "\n" 
        buf +=    "KeepOutZones = " + str( self.KeepOutZones ) + "\n" 
        buf +=    "OperatingRegions = " + str( self.OperatingRegions ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "UXNATIVE") or (len("UXNATIVE") == 0): # this should never happen
        	# Checks for "UXNATIVE" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/CreateNewService")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("UXNATIVE" + "/CreateNewService")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        LMCPObject.LMCPObject.toDictMembers(self, d)
        d['ServiceID'] = self.ServiceID
        d['XmlConfiguration'] = self.XmlConfiguration
        d['EntityConfigurations'] = []
        for x in self.EntityConfigurations:
            if x == None:
                d['EntityConfigurations'].append(None)
            else:
                d['EntityConfigurations'].append(x.toDict())
        d['EntityStates'] = []
        for x in self.EntityStates:
            if x == None:
                d['EntityStates'].append(None)
            else:
                d['EntityStates'].append(x.toDict())
        d['MissionCommands'] = []
        for x in self.MissionCommands:
            if x == None:
                d['MissionCommands'].append(None)
            else:
                d['MissionCommands'].append(x.toDict())
        d['Areas'] = []
        for x in self.Areas:
            if x == None:
                d['Areas'].append(None)
            else:
                d['Areas'].append(x.toDict())
        d['Lines'] = []
        for x in self.Lines:
            if x == None:
                d['Lines'].append(None)
            else:
                d['Lines'].append(x.toDict())
        d['Points'] = []
        for x in self.Points:
            if x == None:
                d['Points'].append(None)
            else:
                d['Points'].append(x.toDict())
        d['KeepInZones'] = []
        for x in self.KeepInZones:
            if x == None:
                d['KeepInZones'].append(None)
            else:
                d['KeepInZones'].append(x.toDict())
        d['KeepOutZones'] = []
        for x in self.KeepOutZones:
            if x == None:
                d['KeepOutZones'].append(None)
            else:
                d['KeepOutZones'].append(x.toDict())
        d['OperatingRegions'] = []
        for x in self.OperatingRegions:
            if x == None:
                d['OperatingRegions'].append(None)
            else:
                d['OperatingRegions'].append(x.toDict())

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
        str = ws + '<CreateNewService Series="UXNATIVE" >\n';
        #str +=LMCPObject.LMCPObject.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</CreateNewService>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += LMCPObject.LMCPObject.toXMLMembersStr(self, ws)
        buf += ws + "<ServiceID>" + str(self.ServiceID) + "</ServiceID>\n"
        buf += ws + "<XmlConfiguration>" + str(self.XmlConfiguration) + "</XmlConfiguration>\n"
        buf += ws + "<EntityConfigurations>\n"
        for x in self.EntityConfigurations:
            if x == None:
                buf += ws + "    <null/>\n"
            else:
                buf += x.toXMLStr(ws + "    ") 
        buf += ws + "</EntityConfigurations>\n"
        buf += ws + "<EntityStates>\n"
        for x in self.EntityStates:
            if x == None:
                buf += ws + "    <null/>\n"
            else:
                buf += x.toXMLStr(ws + "    ") 
        buf += ws + "</EntityStates>\n"
        buf += ws + "<MissionCommands>\n"
        for x in self.MissionCommands:
            if x == None:
                buf += ws + "    <null/>\n"
            else:
                buf += x.toXMLStr(ws + "    ") 
        buf += ws + "</MissionCommands>\n"
        buf += ws + "<Areas>\n"
        for x in self.Areas:
            if x == None:
                buf += ws + "    <null/>\n"
            else:
                buf += x.toXMLStr(ws + "    ") 
        buf += ws + "</Areas>\n"
        buf += ws + "<Lines>\n"
        for x in self.Lines:
            if x == None:
                buf += ws + "    <null/>\n"
            else:
                buf += x.toXMLStr(ws + "    ") 
        buf += ws + "</Lines>\n"
        buf += ws + "<Points>\n"
        for x in self.Points:
            if x == None:
                buf += ws + "    <null/>\n"
            else:
                buf += x.toXMLStr(ws + "    ") 
        buf += ws + "</Points>\n"
        buf += ws + "<KeepInZones>\n"
        for x in self.KeepInZones:
            if x == None:
                buf += ws + "    <null/>\n"
            else:
                buf += x.toXMLStr(ws + "    ") 
        buf += ws + "</KeepInZones>\n"
        buf += ws + "<KeepOutZones>\n"
        for x in self.KeepOutZones:
            if x == None:
                buf += ws + "    <null/>\n"
            else:
                buf += x.toXMLStr(ws + "    ") 
        buf += ws + "</KeepOutZones>\n"
        buf += ws + "<OperatingRegions>\n"
        for x in self.OperatingRegions:
            if x == None:
                buf += ws + "    <null/>\n"
            else:
                buf += x.toXMLStr(ws + "    ") 
        buf += ws + "</OperatingRegions>\n"

        return buf
        
