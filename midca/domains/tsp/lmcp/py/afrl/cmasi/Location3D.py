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

from midca.domains.tsp.lmcp.py.afrl.cmasi import AltitudeType


class Location3D(LMCPObject.LMCPObject):

    def __init__(self):

        self.LMCP_TYPE = 3
        self.SERIES_NAME = "CMASI"
        self.FULL_LMCP_TYPE_NAME = "afrl.cmasi.Location3D"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 4849604199710720000
        self.SERIES_VERSION = 3

        #Define message fields
        self.Latitude = 0   #real64
        self.Longitude = 0   #real64
        self.Altitude = 0   #real32
        self.AltitudeType = AltitudeType.AltitudeType.MSL   #AltitudeType


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(LMCPObject.LMCPObject.pack(self))
        buffer.extend(struct.pack(">d", self.Latitude))
        buffer.extend(struct.pack(">d", self.Longitude))
        buffer.extend(struct.pack(">f", self.Altitude))
        buffer.extend(struct.pack(">i", self.AltitudeType))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = LMCPObject.LMCPObject.unpack(self, buffer, _pos)
        self.Latitude = struct.unpack_from(">d", buffer, _pos)[0]
        _pos += 8
        self.Longitude = struct.unpack_from(">d", buffer, _pos)[0]
        _pos += 8
        self.Altitude = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.AltitudeType = struct.unpack_from(">i", buffer, _pos)[0]
        _pos += 4
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        LMCPObject.LMCPObject.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "Latitude" and len(e.childNodes) > 0 :
                    self.Latitude = float(e.childNodes[0].nodeValue)
                elif e.localName == "Longitude" and len(e.childNodes) > 0 :
                    self.Longitude = float(e.childNodes[0].nodeValue)
                elif e.localName == "Altitude" and len(e.childNodes) > 0 :
                    self.Altitude = float(e.childNodes[0].nodeValue)
                elif e.localName == "AltitudeType" and len(e.childNodes) > 0 :
                    self.AltitudeType = AltitudeType.get_AltitudeType_str(e.childNodes[0].nodeValue)

        return

    def unpackFromDict(self, d, seriesFactory):
        LMCPObject.LMCPObject.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "Latitude":
                self.Latitude = d[key]
            elif key == "Longitude":
                self.Longitude = d[key]
            elif key == "Altitude":
                self.Altitude = d[key]
            elif key == "AltitudeType":
                self.AltitudeType = d[key]

        return

    def get_Latitude(self):
        return self.Latitude

    def set_Latitude(self, value):
        self.Latitude = float( value )

    def get_Longitude(self):
        return self.Longitude

    def set_Longitude(self, value):
        self.Longitude = float( value )

    def get_Altitude(self):
        return self.Altitude

    def set_Altitude(self, value):
        self.Altitude = float( value )

    def get_AltitudeType(self):
        return self.AltitudeType

    def set_AltitudeType(self, value):
        self.AltitudeType = value 



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = LMCPObject.LMCPObject.toString(self)
        buf += "From Location3D:\n"
        buf +=    "Latitude = " + str( self.Latitude ) + "\n" 
        buf +=    "Longitude = " + str( self.Longitude ) + "\n" 
        buf +=    "Altitude = " + str( self.Altitude ) + "\n" 
        buf +=    "AltitudeType = " + str( self.AltitudeType ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "CMASI") or (len("CMASI") == 0): # this should never happen
        	# Checks for "CMASI" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/Location3D")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("CMASI" + "/Location3D")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        LMCPObject.LMCPObject.toDictMembers(self, d)
        d['Latitude'] = self.Latitude
        d['Longitude'] = self.Longitude
        d['Altitude'] = self.Altitude
        d['AltitudeType'] = self.AltitudeType

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
        str = ws + '<Location3D Series="CMASI" >\n';
        #str +=LMCPObject.LMCPObject.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</Location3D>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += LMCPObject.LMCPObject.toXMLMembersStr(self, ws)
        buf += ws + "<Latitude>" + str(self.Latitude) + "</Latitude>\n"
        buf += ws + "<Longitude>" + str(self.Longitude) + "</Longitude>\n"
        buf += ws + "<Altitude>" + str(self.Altitude) + "</Altitude>\n"
        buf += ws + "<AltitudeType>" + AltitudeType.get_AltitudeType_int(self.AltitudeType) + "</AltitudeType>\n"

        return buf
        
