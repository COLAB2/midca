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
from midca.domains.tsp.lmcp.py.afrl.cmasi import SpeedType
from midca.domains.tsp.lmcp.py.afrl.cmasi import AltitudeType


class FlightDirectorAction(NavigationAction.NavigationAction):

    def __init__(self):
        NavigationAction.NavigationAction.__init__(self)
        self.LMCP_TYPE = 54
        self.SERIES_NAME = "CMASI"
        self.FULL_LMCP_TYPE_NAME = "afrl.cmasi.FlightDirectorAction"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 4849604199710720000
        self.SERIES_VERSION = 3

        #Define message fields
        self.Speed = 0   #real32
        self.SpeedType = SpeedType.SpeedType.Airspeed   #SpeedType
        self.Heading = 0   #real32
        self.Altitude = 0   #real32
        self.AltitudeType = AltitudeType.AltitudeType.MSL   #AltitudeType
        self.ClimbRate = 0   #real32


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(NavigationAction.NavigationAction.pack(self))
        buffer.extend(struct.pack(">f", self.Speed))
        buffer.extend(struct.pack(">i", self.SpeedType))
        buffer.extend(struct.pack(">f", self.Heading))
        buffer.extend(struct.pack(">f", self.Altitude))
        buffer.extend(struct.pack(">i", self.AltitudeType))
        buffer.extend(struct.pack(">f", self.ClimbRate))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = NavigationAction.NavigationAction.unpack(self, buffer, _pos)
        self.Speed = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.SpeedType = struct.unpack_from(">i", buffer, _pos)[0]
        _pos += 4
        self.Heading = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.Altitude = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.AltitudeType = struct.unpack_from(">i", buffer, _pos)[0]
        _pos += 4
        self.ClimbRate = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        NavigationAction.NavigationAction.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "Speed" and len(e.childNodes) > 0 :
                    self.Speed = float(e.childNodes[0].nodeValue)
                elif e.localName == "SpeedType" and len(e.childNodes) > 0 :
                    self.SpeedType = SpeedType.get_SpeedType_str(e.childNodes[0].nodeValue)
                elif e.localName == "Heading" and len(e.childNodes) > 0 :
                    self.Heading = float(e.childNodes[0].nodeValue)
                elif e.localName == "Altitude" and len(e.childNodes) > 0 :
                    self.Altitude = float(e.childNodes[0].nodeValue)
                elif e.localName == "AltitudeType" and len(e.childNodes) > 0 :
                    self.AltitudeType = AltitudeType.get_AltitudeType_str(e.childNodes[0].nodeValue)
                elif e.localName == "ClimbRate" and len(e.childNodes) > 0 :
                    self.ClimbRate = float(e.childNodes[0].nodeValue)

        return

    def unpackFromDict(self, d, seriesFactory):
        NavigationAction.NavigationAction.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "Speed":
                self.Speed = d[key]
            elif key == "SpeedType":
                self.SpeedType = d[key]
            elif key == "Heading":
                self.Heading = d[key]
            elif key == "Altitude":
                self.Altitude = d[key]
            elif key == "AltitudeType":
                self.AltitudeType = d[key]
            elif key == "ClimbRate":
                self.ClimbRate = d[key]

        return

    def get_Speed(self):
        return self.Speed

    def set_Speed(self, value):
        self.Speed = float( value )

    def get_SpeedType(self):
        return self.SpeedType

    def set_SpeedType(self, value):
        self.SpeedType = value 

    def get_Heading(self):
        return self.Heading

    def set_Heading(self, value):
        self.Heading = float( value )

    def get_Altitude(self):
        return self.Altitude

    def set_Altitude(self, value):
        self.Altitude = float( value )

    def get_AltitudeType(self):
        return self.AltitudeType

    def set_AltitudeType(self, value):
        self.AltitudeType = value 

    def get_ClimbRate(self):
        return self.ClimbRate

    def set_ClimbRate(self, value):
        self.ClimbRate = float( value )



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = NavigationAction.NavigationAction.toString(self)
        buf += "From FlightDirectorAction:\n"
        buf +=    "Speed = " + str( self.Speed ) + "\n" 
        buf +=    "SpeedType = " + str( self.SpeedType ) + "\n" 
        buf +=    "Heading = " + str( self.Heading ) + "\n" 
        buf +=    "Altitude = " + str( self.Altitude ) + "\n" 
        buf +=    "AltitudeType = " + str( self.AltitudeType ) + "\n" 
        buf +=    "ClimbRate = " + str( self.ClimbRate ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "CMASI") or (len("CMASI") == 0): # this should never happen
        	# Checks for "CMASI" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/FlightDirectorAction")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("CMASI" + "/FlightDirectorAction")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        NavigationAction.NavigationAction.toDictMembers(self, d)
        d['Speed'] = self.Speed
        d['SpeedType'] = self.SpeedType
        d['Heading'] = self.Heading
        d['Altitude'] = self.Altitude
        d['AltitudeType'] = self.AltitudeType
        d['ClimbRate'] = self.ClimbRate

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
        str = ws + '<FlightDirectorAction Series="CMASI" >\n';
        #str +=NavigationAction.NavigationAction.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</FlightDirectorAction>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += NavigationAction.NavigationAction.toXMLMembersStr(self, ws)
        buf += ws + "<Speed>" + str(self.Speed) + "</Speed>\n"
        buf += ws + "<SpeedType>" + SpeedType.get_SpeedType_int(self.SpeedType) + "</SpeedType>\n"
        buf += ws + "<Heading>" + str(self.Heading) + "</Heading>\n"
        buf += ws + "<Altitude>" + str(self.Altitude) + "</Altitude>\n"
        buf += ws + "<AltitudeType>" + AltitudeType.get_AltitudeType_int(self.AltitudeType) + "</AltitudeType>\n"
        buf += ws + "<ClimbRate>" + str(self.ClimbRate) + "</ClimbRate>\n"

        return buf
        
