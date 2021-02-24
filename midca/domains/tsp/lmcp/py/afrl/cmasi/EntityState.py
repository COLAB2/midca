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

from midca.domains.tsp.lmcp.py.afrl.cmasi import Location3D
from midca.domains.tsp.lmcp.py.afrl.cmasi import PayloadState
from midca.domains.tsp.lmcp.py.afrl.cmasi import NavigationMode
from midca.domains.tsp.lmcp.py.afrl.cmasi import KeyValuePair


class EntityState(LMCPObject.LMCPObject):

    def __init__(self):

        self.LMCP_TYPE = 14
        self.SERIES_NAME = "CMASI"
        self.FULL_LMCP_TYPE_NAME = "afrl.cmasi.EntityState"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 4849604199710720000
        self.SERIES_VERSION = 3

        #Define message fields
        self.ID = 0   #int64
        self.u = 0   #real32
        self.v = 0   #real32
        self.w = 0   #real32
        self.udot = 0   #real32
        self.vdot = 0   #real32
        self.wdot = 0   #real32
        self.Heading = 0   #real32
        self.Pitch = 0   #real32
        self.Roll = 0   #real32
        self.p = 0   #real32
        self.q = 0   #real32
        self.r = 0   #real32
        self.Course = 0   #real32
        self.Groundspeed = 0   #real32
        self.Location = Location3D.Location3D()   #Location3D
        self.EnergyAvailable = 0   #real32
        self.ActualEnergyRate = 0   #real32
        self.PayloadStateList = []   #PayloadState
        self.CurrentWaypoint = 0   #int64
        self.CurrentCommand = 0   #int64
        self.Mode = NavigationMode.NavigationMode.Waypoint   #NavigationMode
        self.AssociatedTasks = []   #int64
        self.Time = 0   #int64
        self.Info = []   #KeyValuePair


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(LMCPObject.LMCPObject.pack(self))
        buffer.extend(struct.pack(">q", self.ID))
        buffer.extend(struct.pack(">f", self.u))
        buffer.extend(struct.pack(">f", self.v))
        buffer.extend(struct.pack(">f", self.w))
        buffer.extend(struct.pack(">f", self.udot))
        buffer.extend(struct.pack(">f", self.vdot))
        buffer.extend(struct.pack(">f", self.wdot))
        buffer.extend(struct.pack(">f", self.Heading))
        buffer.extend(struct.pack(">f", self.Pitch))
        buffer.extend(struct.pack(">f", self.Roll))
        buffer.extend(struct.pack(">f", self.p))
        buffer.extend(struct.pack(">f", self.q))
        buffer.extend(struct.pack(">f", self.r))
        buffer.extend(struct.pack(">f", self.Course))
        buffer.extend(struct.pack(">f", self.Groundspeed))
        buffer.extend(struct.pack("B", self.Location != None ))
        if self.Location != None:
            buffer.extend(struct.pack(">q", self.Location.SERIES_NAME_ID))
            buffer.extend(struct.pack(">I", self.Location.LMCP_TYPE))
            buffer.extend(struct.pack(">H", self.Location.SERIES_VERSION))
            buffer.extend(self.Location.pack())
        buffer.extend(struct.pack(">f", self.EnergyAvailable))
        buffer.extend(struct.pack(">f", self.ActualEnergyRate))
        buffer.extend(struct.pack(">H", len(self.PayloadStateList) ))
        for x in self.PayloadStateList:
           buffer.extend(struct.pack("B", x != None ))
           if x != None:
               buffer.extend(struct.pack(">q", x.SERIES_NAME_ID))
               buffer.extend(struct.pack(">I", x.LMCP_TYPE))
               buffer.extend(struct.pack(">H", x.SERIES_VERSION))
               buffer.extend(x.pack())
        buffer.extend(struct.pack(">q", self.CurrentWaypoint))
        buffer.extend(struct.pack(">q", self.CurrentCommand))
        buffer.extend(struct.pack(">i", self.Mode))
        buffer.extend(struct.pack(">H", len(self.AssociatedTasks) ))
        for x in self.AssociatedTasks:
            buffer.extend(struct.pack(">q", x ))
        buffer.extend(struct.pack(">q", self.Time))
        buffer.extend(struct.pack(">H", len(self.Info) ))
        for x in self.Info:
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
        self.ID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.u = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.v = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.w = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.udot = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.vdot = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.wdot = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.Heading = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.Pitch = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.Roll = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.p = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.q = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.r = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.Course = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.Groundspeed = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
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
            self.Location = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
            _pos = self.Location.unpack(buffer, _pos)
        else:
            self.Location = None
        self.EnergyAvailable = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.ActualEnergyRate = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.PayloadStateList = [None] * _arraylen
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
                self.PayloadStateList[x] = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
                _pos = self.PayloadStateList[x].unpack(buffer, _pos)
            else:
                self.PayloadStateList[x] = None
        self.CurrentWaypoint = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.CurrentCommand = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.Mode = struct.unpack_from(">i", buffer, _pos)[0]
        _pos += 4
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.AssociatedTasks = [None] * _arraylen
        if _arraylen > 0:
            self.AssociatedTasks = struct.unpack_from(">" + repr(_arraylen) + "q", buffer, _pos )
            _pos += 8 * _arraylen
        self.Time = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.Info = [None] * _arraylen
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
                self.Info[x] = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
                _pos = self.Info[x].unpack(buffer, _pos)
            else:
                self.Info[x] = None
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        LMCPObject.LMCPObject.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "ID" and len(e.childNodes) > 0 :
                    self.ID = int(e.childNodes[0].nodeValue)
                elif e.localName == "u" and len(e.childNodes) > 0 :
                    self.u = float(e.childNodes[0].nodeValue)
                elif e.localName == "v" and len(e.childNodes) > 0 :
                    self.v = float(e.childNodes[0].nodeValue)
                elif e.localName == "w" and len(e.childNodes) > 0 :
                    self.w = float(e.childNodes[0].nodeValue)
                elif e.localName == "udot" and len(e.childNodes) > 0 :
                    self.udot = float(e.childNodes[0].nodeValue)
                elif e.localName == "vdot" and len(e.childNodes) > 0 :
                    self.vdot = float(e.childNodes[0].nodeValue)
                elif e.localName == "wdot" and len(e.childNodes) > 0 :
                    self.wdot = float(e.childNodes[0].nodeValue)
                elif e.localName == "Heading" and len(e.childNodes) > 0 :
                    self.Heading = float(e.childNodes[0].nodeValue)
                elif e.localName == "Pitch" and len(e.childNodes) > 0 :
                    self.Pitch = float(e.childNodes[0].nodeValue)
                elif e.localName == "Roll" and len(e.childNodes) > 0 :
                    self.Roll = float(e.childNodes[0].nodeValue)
                elif e.localName == "p" and len(e.childNodes) > 0 :
                    self.p = float(e.childNodes[0].nodeValue)
                elif e.localName == "q" and len(e.childNodes) > 0 :
                    self.q = float(e.childNodes[0].nodeValue)
                elif e.localName == "r" and len(e.childNodes) > 0 :
                    self.r = float(e.childNodes[0].nodeValue)
                elif e.localName == "Course" and len(e.childNodes) > 0 :
                    self.Course = float(e.childNodes[0].nodeValue)
                elif e.localName == "Groundspeed" and len(e.childNodes) > 0 :
                    self.Groundspeed = float(e.childNodes[0].nodeValue)
                elif e.localName == "Location" and len(e.childNodes) > 0 :
                    for n in e.childNodes:
                        if n.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.Location = seriesFactory.createObjectByName(n.getAttribute('Series'), n.localName)
                            if self.Location != None:
                                self.Location.unpackFromXMLNode(n, seriesFactory)
                elif e.localName == "EnergyAvailable" and len(e.childNodes) > 0 :
                    self.EnergyAvailable = float(e.childNodes[0].nodeValue)
                elif e.localName == "ActualEnergyRate" and len(e.childNodes) > 0 :
                    self.ActualEnergyRate = float(e.childNodes[0].nodeValue)
                elif e.localName == "PayloadStateList" and len(e.childNodes) > 0 :
                    self.PayloadStateList = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            obj = seriesFactory.createObjectByName(c.getAttribute('Series'), c.localName)
                            if obj != None:
                                obj.unpackFromXMLNode(c, seriesFactory)
                                self.PayloadStateList.append(obj)
                elif e.localName == "CurrentWaypoint" and len(e.childNodes) > 0 :
                    self.CurrentWaypoint = int(e.childNodes[0].nodeValue)
                elif e.localName == "CurrentCommand" and len(e.childNodes) > 0 :
                    self.CurrentCommand = int(e.childNodes[0].nodeValue)
                elif e.localName == "Mode" and len(e.childNodes) > 0 :
                    self.Mode = NavigationMode.get_NavigationMode_str(e.childNodes[0].nodeValue)
                elif e.localName == "AssociatedTasks" and len(e.childNodes) > 0 :
                    self.AssociatedTasks = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.AssociatedTasks.append( int(c.childNodes[0].nodeValue) )
                elif e.localName == "Time" and len(e.childNodes) > 0 :
                    self.Time = int(e.childNodes[0].nodeValue)
                elif e.localName == "Info" and len(e.childNodes) > 0 :
                    self.Info = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            obj = seriesFactory.createObjectByName(c.getAttribute('Series'), c.localName)
                            if obj != None:
                                obj.unpackFromXMLNode(c, seriesFactory)
                                self.Info.append(obj)

        return

    def unpackFromDict(self, d, seriesFactory):
        LMCPObject.LMCPObject.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "ID":
                self.ID = d[key]
            elif key == "u":
                self.u = d[key]
            elif key == "v":
                self.v = d[key]
            elif key == "w":
                self.w = d[key]
            elif key == "udot":
                self.udot = d[key]
            elif key == "vdot":
                self.vdot = d[key]
            elif key == "wdot":
                self.wdot = d[key]
            elif key == "Heading":
                self.Heading = d[key]
            elif key == "Pitch":
                self.Pitch = d[key]
            elif key == "Roll":
                self.Roll = d[key]
            elif key == "p":
                self.p = d[key]
            elif key == "q":
                self.q = d[key]
            elif key == "r":
                self.r = d[key]
            elif key == "Course":
                self.Course = d[key]
            elif key == "Groundspeed":
                self.Groundspeed = d[key]
            elif key == "Location":
                self.Location = seriesFactory.unpackFromDict(d[key])
            elif key == "EnergyAvailable":
                self.EnergyAvailable = d[key]
            elif key == "ActualEnergyRate":
                self.ActualEnergyRate = d[key]
            elif key == "PayloadStateList":
                self.PayloadStateList = []
                for c in d[key]:
                    obj = seriesFactory.unpackFromDict(c)
                    if obj != None:
                        self.PayloadStateList.append(obj)
            elif key == "CurrentWaypoint":
                self.CurrentWaypoint = d[key]
            elif key == "CurrentCommand":
                self.CurrentCommand = d[key]
            elif key == "Mode":
                self.Mode = d[key]
            elif key == "AssociatedTasks":
                self.AssociatedTasks = []
                for c in d[key]:
                    self.AssociatedTasks.append( c )
            elif key == "Time":
                self.Time = d[key]
            elif key == "Info":
                self.Info = []
                for c in d[key]:
                    obj = seriesFactory.unpackFromDict(c)
                    if obj != None:
                        self.Info.append(obj)

        return

    def get_ID(self):
        return self.ID

    def set_ID(self, value):
        self.ID = int( value )

    def get_u(self):
        return self.u

    def set_u(self, value):
        self.u = float( value )

    def get_v(self):
        return self.v

    def set_v(self, value):
        self.v = float( value )

    def get_w(self):
        return self.w

    def set_w(self, value):
        self.w = float( value )

    def get_udot(self):
        return self.udot

    def set_udot(self, value):
        self.udot = float( value )

    def get_vdot(self):
        return self.vdot

    def set_vdot(self, value):
        self.vdot = float( value )

    def get_wdot(self):
        return self.wdot

    def set_wdot(self, value):
        self.wdot = float( value )

    def get_Heading(self):
        return self.Heading

    def set_Heading(self, value):
        self.Heading = float( value )

    def get_Pitch(self):
        return self.Pitch

    def set_Pitch(self, value):
        self.Pitch = float( value )

    def get_Roll(self):
        return self.Roll

    def set_Roll(self, value):
        self.Roll = float( value )

    def get_p(self):
        return self.p

    def set_p(self, value):
        self.p = float( value )

    def get_q(self):
        return self.q

    def set_q(self, value):
        self.q = float( value )

    def get_r(self):
        return self.r

    def set_r(self, value):
        self.r = float( value )

    def get_Course(self):
        return self.Course

    def set_Course(self, value):
        self.Course = float( value )

    def get_Groundspeed(self):
        return self.Groundspeed

    def set_Groundspeed(self, value):
        self.Groundspeed = float( value )

    def get_Location(self):
        return self.Location

    def set_Location(self, value):
        self.Location = value 

    def get_EnergyAvailable(self):
        return self.EnergyAvailable

    def set_EnergyAvailable(self, value):
        self.EnergyAvailable = float( value )

    def get_ActualEnergyRate(self):
        return self.ActualEnergyRate

    def set_ActualEnergyRate(self, value):
        self.ActualEnergyRate = float( value )

    def get_PayloadStateList(self):
        return self.PayloadStateList

    def get_CurrentWaypoint(self):
        return self.CurrentWaypoint

    def set_CurrentWaypoint(self, value):
        self.CurrentWaypoint = int( value )

    def get_CurrentCommand(self):
        return self.CurrentCommand

    def set_CurrentCommand(self, value):
        self.CurrentCommand = int( value )

    def get_Mode(self):
        return self.Mode

    def set_Mode(self, value):
        self.Mode = value 

    def get_AssociatedTasks(self):
        return self.AssociatedTasks

    def get_Time(self):
        return self.Time

    def set_Time(self, value):
        self.Time = int( value )

    def get_Info(self):
        return self.Info



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = LMCPObject.LMCPObject.toString(self)
        buf += "From EntityState:\n"
        buf +=    "ID = " + str( self.ID ) + "\n" 
        buf +=    "u = " + str( self.u ) + "\n" 
        buf +=    "v = " + str( self.v ) + "\n" 
        buf +=    "w = " + str( self.w ) + "\n" 
        buf +=    "udot = " + str( self.udot ) + "\n" 
        buf +=    "vdot = " + str( self.vdot ) + "\n" 
        buf +=    "wdot = " + str( self.wdot ) + "\n" 
        buf +=    "Heading = " + str( self.Heading ) + "\n" 
        buf +=    "Pitch = " + str( self.Pitch ) + "\n" 
        buf +=    "Roll = " + str( self.Roll ) + "\n" 
        buf +=    "p = " + str( self.p ) + "\n" 
        buf +=    "q = " + str( self.q ) + "\n" 
        buf +=    "r = " + str( self.r ) + "\n" 
        buf +=    "Course = " + str( self.Course ) + "\n" 
        buf +=    "Groundspeed = " + str( self.Groundspeed ) + "\n" 
        buf +=    "Location = " + str( self.Location ) + "\n" 
        buf +=    "EnergyAvailable = " + str( self.EnergyAvailable ) + "\n" 
        buf +=    "ActualEnergyRate = " + str( self.ActualEnergyRate ) + "\n" 
        buf +=    "PayloadStateList = " + str( self.PayloadStateList ) + "\n" 
        buf +=    "CurrentWaypoint = " + str( self.CurrentWaypoint ) + "\n" 
        buf +=    "CurrentCommand = " + str( self.CurrentCommand ) + "\n" 
        buf +=    "Mode = " + str( self.Mode ) + "\n" 
        buf +=    "AssociatedTasks = " + str( self.AssociatedTasks ) + "\n" 
        buf +=    "Time = " + str( self.Time ) + "\n" 
        buf +=    "Info = " + str( self.Info ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "CMASI") or (len("CMASI") == 0): # this should never happen
        	# Checks for "CMASI" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/EntityState")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("CMASI" + "/EntityState")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        LMCPObject.LMCPObject.toDictMembers(self, d)
        d['ID'] = self.ID
        d['u'] = self.u
        d['v'] = self.v
        d['w'] = self.w
        d['udot'] = self.udot
        d['vdot'] = self.vdot
        d['wdot'] = self.wdot
        d['Heading'] = self.Heading
        d['Pitch'] = self.Pitch
        d['Roll'] = self.Roll
        d['p'] = self.p
        d['q'] = self.q
        d['r'] = self.r
        d['Course'] = self.Course
        d['Groundspeed'] = self.Groundspeed
        if self.Location == None:
            d['Location'] = None
        else:
            d['Location'] = self.Location.toDict()
        d['EnergyAvailable'] = self.EnergyAvailable
        d['ActualEnergyRate'] = self.ActualEnergyRate
        d['PayloadStateList'] = []
        for x in self.PayloadStateList:
            if x == None:
                d['PayloadStateList'].append(None)
            else:
                d['PayloadStateList'].append(x.toDict())
        d['CurrentWaypoint'] = self.CurrentWaypoint
        d['CurrentCommand'] = self.CurrentCommand
        d['Mode'] = self.Mode
        d['AssociatedTasks'] = []
        for x in self.AssociatedTasks:
            d['AssociatedTasks'].append(x)
        d['Time'] = self.Time
        d['Info'] = []
        for x in self.Info:
            if x == None:
                d['Info'].append(None)
            else:
                d['Info'].append(x.toDict())

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
        str = ws + '<EntityState Series="CMASI" >\n';
        #str +=LMCPObject.LMCPObject.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</EntityState>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += LMCPObject.LMCPObject.toXMLMembersStr(self, ws)
        buf += ws + "<ID>" + str(self.ID) + "</ID>\n"
        buf += ws + "<u>" + str(self.u) + "</u>\n"
        buf += ws + "<v>" + str(self.v) + "</v>\n"
        buf += ws + "<w>" + str(self.w) + "</w>\n"
        buf += ws + "<udot>" + str(self.udot) + "</udot>\n"
        buf += ws + "<vdot>" + str(self.vdot) + "</vdot>\n"
        buf += ws + "<wdot>" + str(self.wdot) + "</wdot>\n"
        buf += ws + "<Heading>" + str(self.Heading) + "</Heading>\n"
        buf += ws + "<Pitch>" + str(self.Pitch) + "</Pitch>\n"
        buf += ws + "<Roll>" + str(self.Roll) + "</Roll>\n"
        buf += ws + "<p>" + str(self.p) + "</p>\n"
        buf += ws + "<q>" + str(self.q) + "</q>\n"
        buf += ws + "<r>" + str(self.r) + "</r>\n"
        buf += ws + "<Course>" + str(self.Course) + "</Course>\n"
        buf += ws + "<Groundspeed>" + str(self.Groundspeed) + "</Groundspeed>\n"
        if self.Location != None:
            buf += ws + "<Location>\n"
            buf += ws + self.Location.toXMLStr(ws + "    ") 
            buf += ws + "</Location>\n"
        buf += ws + "<EnergyAvailable>" + str(self.EnergyAvailable) + "</EnergyAvailable>\n"
        buf += ws + "<ActualEnergyRate>" + str(self.ActualEnergyRate) + "</ActualEnergyRate>\n"
        buf += ws + "<PayloadStateList>\n"
        for x in self.PayloadStateList:
            if x == None:
                buf += ws + "    <null/>\n"
            else:
                buf += x.toXMLStr(ws + "    ") 
        buf += ws + "</PayloadStateList>\n"
        buf += ws + "<CurrentWaypoint>" + str(self.CurrentWaypoint) + "</CurrentWaypoint>\n"
        buf += ws + "<CurrentCommand>" + str(self.CurrentCommand) + "</CurrentCommand>\n"
        buf += ws + "<Mode>" + NavigationMode.get_NavigationMode_int(self.Mode) + "</Mode>\n"
        buf += ws + "<AssociatedTasks>\n"
        for x in self.AssociatedTasks:
            buf += ws + "<int64>" + str(x) + "</int64>\n"
        buf += ws + "</AssociatedTasks>\n"
        buf += ws + "<Time>" + str(self.Time) + "</Time>\n"
        buf += ws + "<Info>\n"
        for x in self.Info:
            if x == None:
                buf += ws + "    <null/>\n"
            else:
                buf += x.toXMLStr(ws + "    ") 
        buf += ws + "</Info>\n"

        return buf
        
