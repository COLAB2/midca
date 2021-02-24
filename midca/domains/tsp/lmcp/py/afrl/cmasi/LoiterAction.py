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

from midca.domains.tsp.lmcp.py.afrl.cmasi import NavigationAction
from midca.domains.tsp.lmcp.py.afrl.cmasi import LoiterType
from midca.domains.tsp.lmcp.py.afrl.cmasi import LoiterDirection
from midca.domains.tsp.lmcp.py.afrl.cmasi import Location3D


class LoiterAction(NavigationAction.NavigationAction):

    def __init__(self):
        NavigationAction.NavigationAction.__init__(self)
        self.LMCP_TYPE = 33
        self.SERIES_NAME = "CMASI"
        self.FULL_LMCP_TYPE_NAME = "afrl.cmasi.LoiterAction"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 4849604199710720000
        self.SERIES_VERSION = 3

        #Define message fields
        self.LoiterType = LoiterType.LoiterType.VehicleDefault   #LoiterType
        self.Radius = 0   #real32
        self.Axis = 0   #real32
        self.Length = 0   #real32
        self.Direction = LoiterDirection.LoiterDirection.VehicleDefault   #LoiterDirection
        self.Duration = 0   #int64
        self.Airspeed = 0   #real32
        self.Location = Location3D.Location3D()   #Location3D


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(NavigationAction.NavigationAction.pack(self))
        buffer.extend(struct.pack(">i", self.LoiterType))
        buffer.extend(struct.pack(">f", self.Radius))
        buffer.extend(struct.pack(">f", self.Axis))
        buffer.extend(struct.pack(">f", self.Length))
        buffer.extend(struct.pack(">i", self.Direction))
        buffer.extend(struct.pack(">q", self.Duration))
        buffer.extend(struct.pack(">f", self.Airspeed))
        buffer.extend(struct.pack("B", self.Location != None ))
        if self.Location != None:
            buffer.extend(struct.pack(">q", self.Location.SERIES_NAME_ID))
            buffer.extend(struct.pack(">I", self.Location.LMCP_TYPE))
            buffer.extend(struct.pack(">H", self.Location.SERIES_VERSION))
            buffer.extend(self.Location.pack())

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = NavigationAction.NavigationAction.unpack(self, buffer, _pos)
        self.LoiterType = struct.unpack_from(">i", buffer, _pos)[0]
        _pos += 4
        self.Radius = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.Axis = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.Length = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.Direction = struct.unpack_from(">i", buffer, _pos)[0]
        _pos += 4
        self.Duration = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.Airspeed = struct.unpack_from(">f", buffer, _pos)[0]
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
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        NavigationAction.NavigationAction.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "LoiterType" and len(e.childNodes) > 0 :
                    self.LoiterType = LoiterType.get_LoiterType_str(e.childNodes[0].nodeValue)
                elif e.localName == "Radius" and len(e.childNodes) > 0 :
                    self.Radius = float(e.childNodes[0].nodeValue)
                elif e.localName == "Axis" and len(e.childNodes) > 0 :
                    self.Axis = float(e.childNodes[0].nodeValue)
                elif e.localName == "Length" and len(e.childNodes) > 0 :
                    self.Length = float(e.childNodes[0].nodeValue)
                elif e.localName == "Direction" and len(e.childNodes) > 0 :
                    self.Direction = LoiterDirection.get_LoiterDirection_str(e.childNodes[0].nodeValue)
                elif e.localName == "Duration" and len(e.childNodes) > 0 :
                    self.Duration = int(e.childNodes[0].nodeValue)
                elif e.localName == "Airspeed" and len(e.childNodes) > 0 :
                    self.Airspeed = float(e.childNodes[0].nodeValue)
                elif e.localName == "Location" and len(e.childNodes) > 0 :
                    for n in e.childNodes:
                        if n.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.Location = seriesFactory.createObjectByName(n.getAttribute('Series'), n.localName)
                            if self.Location != None:
                                self.Location.unpackFromXMLNode(n, seriesFactory)

        return

    def unpackFromDict(self, d, seriesFactory):
        NavigationAction.NavigationAction.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "LoiterType":
                self.LoiterType = d[key]
            elif key == "Radius":
                self.Radius = d[key]
            elif key == "Axis":
                self.Axis = d[key]
            elif key == "Length":
                self.Length = d[key]
            elif key == "Direction":
                self.Direction = d[key]
            elif key == "Duration":
                self.Duration = d[key]
            elif key == "Airspeed":
                self.Airspeed = d[key]
            elif key == "Location":
                self.Location = seriesFactory.unpackFromDict(d[key])

        return

    def get_LoiterType(self):
        return self.LoiterType

    def set_LoiterType(self, value):
        self.LoiterType = value 

    def get_Radius(self):
        return self.Radius

    def set_Radius(self, value):
        self.Radius = float( value )

    def get_Axis(self):
        return self.Axis

    def set_Axis(self, value):
        self.Axis = float( value )

    def get_Length(self):
        return self.Length

    def set_Length(self, value):
        self.Length = float( value )

    def get_Direction(self):
        return self.Direction

    def set_Direction(self, value):
        self.Direction = value 

    def get_Duration(self):
        return self.Duration

    def set_Duration(self, value):
        self.Duration = int( value )

    def get_Airspeed(self):
        return self.Airspeed

    def set_Airspeed(self, value):
        self.Airspeed = float( value )

    def get_Location(self):
        return self.Location

    def set_Location(self, value):
        self.Location = value 



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = NavigationAction.NavigationAction.toString(self)
        buf += "From LoiterAction:\n"
        buf +=    "LoiterType = " + str( self.LoiterType ) + "\n" 
        buf +=    "Radius = " + str( self.Radius ) + "\n" 
        buf +=    "Axis = " + str( self.Axis ) + "\n" 
        buf +=    "Length = " + str( self.Length ) + "\n" 
        buf +=    "Direction = " + str( self.Direction ) + "\n" 
        buf +=    "Duration = " + str( self.Duration ) + "\n" 
        buf +=    "Airspeed = " + str( self.Airspeed ) + "\n" 
        buf +=    "Location = " + str( self.Location ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "CMASI") or (len("CMASI") == 0): # this should never happen
        	# Checks for "CMASI" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/LoiterAction")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("CMASI" + "/LoiterAction")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        NavigationAction.NavigationAction.toDictMembers(self, d)
        d['LoiterType'] = self.LoiterType
        d['Radius'] = self.Radius
        d['Axis'] = self.Axis
        d['Length'] = self.Length
        d['Direction'] = self.Direction
        d['Duration'] = self.Duration
        d['Airspeed'] = self.Airspeed
        if self.Location == None:
            d['Location'] = None
        else:
            d['Location'] = self.Location.toDict()

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
        str = ws + '<LoiterAction Series="CMASI" >\n';
        #str +=NavigationAction.NavigationAction.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</LoiterAction>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += NavigationAction.NavigationAction.toXMLMembersStr(self, ws)
        buf += ws + "<LoiterType>" + LoiterType.get_LoiterType_int(self.LoiterType) + "</LoiterType>\n"
        buf += ws + "<Radius>" + str(self.Radius) + "</Radius>\n"
        buf += ws + "<Axis>" + str(self.Axis) + "</Axis>\n"
        buf += ws + "<Length>" + str(self.Length) + "</Length>\n"
        buf += ws + "<Direction>" + LoiterDirection.get_LoiterDirection_int(self.Direction) + "</Direction>\n"
        buf += ws + "<Duration>" + str(self.Duration) + "</Duration>\n"
        buf += ws + "<Airspeed>" + str(self.Airspeed) + "</Airspeed>\n"
        if self.Location != None:
            buf += ws + "<Location>\n"
            buf += ws + self.Location.toXMLStr(ws + "    ") 
            buf += ws + "</Location>\n"

        return buf
        
