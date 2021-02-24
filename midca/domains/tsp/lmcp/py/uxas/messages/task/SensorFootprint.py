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

from midca.domains.tsp.lmcp.py.afrl.cmasi import WavelengthBand


class SensorFootprint(LMCPObject.LMCPObject):

    def __init__(self):

        self.LMCP_TYPE = 12
        self.SERIES_NAME = "UXTASK"
        self.FULL_LMCP_TYPE_NAME = "uxas.messages.task.SensorFootprint"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 6149757930721443840
        self.SERIES_VERSION = 8

        #Define message fields
        self.FootprintResponseID = 0   #int64
        self.VehicleID = 0   #int64
        self.CameraID = 0   #int64
        self.GimbalID = 0   #int64
        self.HorizontalFOV = 0   #real32
        self.AglAltitude = 0   #real32
        self.GimbalElevation = 0   #real32
        self.AspectRatio = 0   #real32
        self.AchievedGSD = 0   #real32
        self.CameraWavelength = WavelengthBand.WavelengthBand.AllAny   #WavelengthBand
        self.HorizontalToLeadingEdge = 0   #real32
        self.HorizontalToTrailingEdge = 0   #real32
        self.HorizontalToCenter = 0   #real32
        self.WidthCenter = 0   #real32
        self.SlantRangeToCenter = 0   #real32


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(LMCPObject.LMCPObject.pack(self))
        buffer.extend(struct.pack(">q", self.FootprintResponseID))
        buffer.extend(struct.pack(">q", self.VehicleID))
        buffer.extend(struct.pack(">q", self.CameraID))
        buffer.extend(struct.pack(">q", self.GimbalID))
        buffer.extend(struct.pack(">f", self.HorizontalFOV))
        buffer.extend(struct.pack(">f", self.AglAltitude))
        buffer.extend(struct.pack(">f", self.GimbalElevation))
        buffer.extend(struct.pack(">f", self.AspectRatio))
        buffer.extend(struct.pack(">f", self.AchievedGSD))
        buffer.extend(struct.pack(">i", self.CameraWavelength))
        buffer.extend(struct.pack(">f", self.HorizontalToLeadingEdge))
        buffer.extend(struct.pack(">f", self.HorizontalToTrailingEdge))
        buffer.extend(struct.pack(">f", self.HorizontalToCenter))
        buffer.extend(struct.pack(">f", self.WidthCenter))
        buffer.extend(struct.pack(">f", self.SlantRangeToCenter))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = LMCPObject.LMCPObject.unpack(self, buffer, _pos)
        self.FootprintResponseID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.VehicleID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.CameraID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.GimbalID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.HorizontalFOV = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.AglAltitude = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.GimbalElevation = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.AspectRatio = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.AchievedGSD = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.CameraWavelength = struct.unpack_from(">i", buffer, _pos)[0]
        _pos += 4
        self.HorizontalToLeadingEdge = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.HorizontalToTrailingEdge = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.HorizontalToCenter = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.WidthCenter = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.SlantRangeToCenter = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        LMCPObject.LMCPObject.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "FootprintResponseID" and len(e.childNodes) > 0 :
                    self.FootprintResponseID = int(e.childNodes[0].nodeValue)
                elif e.localName == "VehicleID" and len(e.childNodes) > 0 :
                    self.VehicleID = int(e.childNodes[0].nodeValue)
                elif e.localName == "CameraID" and len(e.childNodes) > 0 :
                    self.CameraID = int(e.childNodes[0].nodeValue)
                elif e.localName == "GimbalID" and len(e.childNodes) > 0 :
                    self.GimbalID = int(e.childNodes[0].nodeValue)
                elif e.localName == "HorizontalFOV" and len(e.childNodes) > 0 :
                    self.HorizontalFOV = float(e.childNodes[0].nodeValue)
                elif e.localName == "AglAltitude" and len(e.childNodes) > 0 :
                    self.AglAltitude = float(e.childNodes[0].nodeValue)
                elif e.localName == "GimbalElevation" and len(e.childNodes) > 0 :
                    self.GimbalElevation = float(e.childNodes[0].nodeValue)
                elif e.localName == "AspectRatio" and len(e.childNodes) > 0 :
                    self.AspectRatio = float(e.childNodes[0].nodeValue)
                elif e.localName == "AchievedGSD" and len(e.childNodes) > 0 :
                    self.AchievedGSD = float(e.childNodes[0].nodeValue)
                elif e.localName == "CameraWavelength" and len(e.childNodes) > 0 :
                    self.CameraWavelength = WavelengthBand.get_WavelengthBand_str(e.childNodes[0].nodeValue)
                elif e.localName == "HorizontalToLeadingEdge" and len(e.childNodes) > 0 :
                    self.HorizontalToLeadingEdge = float(e.childNodes[0].nodeValue)
                elif e.localName == "HorizontalToTrailingEdge" and len(e.childNodes) > 0 :
                    self.HorizontalToTrailingEdge = float(e.childNodes[0].nodeValue)
                elif e.localName == "HorizontalToCenter" and len(e.childNodes) > 0 :
                    self.HorizontalToCenter = float(e.childNodes[0].nodeValue)
                elif e.localName == "WidthCenter" and len(e.childNodes) > 0 :
                    self.WidthCenter = float(e.childNodes[0].nodeValue)
                elif e.localName == "SlantRangeToCenter" and len(e.childNodes) > 0 :
                    self.SlantRangeToCenter = float(e.childNodes[0].nodeValue)

        return

    def unpackFromDict(self, d, seriesFactory):
        LMCPObject.LMCPObject.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "FootprintResponseID":
                self.FootprintResponseID = d[key]
            elif key == "VehicleID":
                self.VehicleID = d[key]
            elif key == "CameraID":
                self.CameraID = d[key]
            elif key == "GimbalID":
                self.GimbalID = d[key]
            elif key == "HorizontalFOV":
                self.HorizontalFOV = d[key]
            elif key == "AglAltitude":
                self.AglAltitude = d[key]
            elif key == "GimbalElevation":
                self.GimbalElevation = d[key]
            elif key == "AspectRatio":
                self.AspectRatio = d[key]
            elif key == "AchievedGSD":
                self.AchievedGSD = d[key]
            elif key == "CameraWavelength":
                self.CameraWavelength = d[key]
            elif key == "HorizontalToLeadingEdge":
                self.HorizontalToLeadingEdge = d[key]
            elif key == "HorizontalToTrailingEdge":
                self.HorizontalToTrailingEdge = d[key]
            elif key == "HorizontalToCenter":
                self.HorizontalToCenter = d[key]
            elif key == "WidthCenter":
                self.WidthCenter = d[key]
            elif key == "SlantRangeToCenter":
                self.SlantRangeToCenter = d[key]

        return

    def get_FootprintResponseID(self):
        return self.FootprintResponseID

    def set_FootprintResponseID(self, value):
        self.FootprintResponseID = int( value )

    def get_VehicleID(self):
        return self.VehicleID

    def set_VehicleID(self, value):
        self.VehicleID = int( value )

    def get_CameraID(self):
        return self.CameraID

    def set_CameraID(self, value):
        self.CameraID = int( value )

    def get_GimbalID(self):
        return self.GimbalID

    def set_GimbalID(self, value):
        self.GimbalID = int( value )

    def get_HorizontalFOV(self):
        return self.HorizontalFOV

    def set_HorizontalFOV(self, value):
        self.HorizontalFOV = float( value )

    def get_AglAltitude(self):
        return self.AglAltitude

    def set_AglAltitude(self, value):
        self.AglAltitude = float( value )

    def get_GimbalElevation(self):
        return self.GimbalElevation

    def set_GimbalElevation(self, value):
        self.GimbalElevation = float( value )

    def get_AspectRatio(self):
        return self.AspectRatio

    def set_AspectRatio(self, value):
        self.AspectRatio = float( value )

    def get_AchievedGSD(self):
        return self.AchievedGSD

    def set_AchievedGSD(self, value):
        self.AchievedGSD = float( value )

    def get_CameraWavelength(self):
        return self.CameraWavelength

    def set_CameraWavelength(self, value):
        self.CameraWavelength = value 

    def get_HorizontalToLeadingEdge(self):
        return self.HorizontalToLeadingEdge

    def set_HorizontalToLeadingEdge(self, value):
        self.HorizontalToLeadingEdge = float( value )

    def get_HorizontalToTrailingEdge(self):
        return self.HorizontalToTrailingEdge

    def set_HorizontalToTrailingEdge(self, value):
        self.HorizontalToTrailingEdge = float( value )

    def get_HorizontalToCenter(self):
        return self.HorizontalToCenter

    def set_HorizontalToCenter(self, value):
        self.HorizontalToCenter = float( value )

    def get_WidthCenter(self):
        return self.WidthCenter

    def set_WidthCenter(self, value):
        self.WidthCenter = float( value )

    def get_SlantRangeToCenter(self):
        return self.SlantRangeToCenter

    def set_SlantRangeToCenter(self, value):
        self.SlantRangeToCenter = float( value )



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = LMCPObject.LMCPObject.toString(self)
        buf += "From SensorFootprint:\n"
        buf +=    "FootprintResponseID = " + str( self.FootprintResponseID ) + "\n" 
        buf +=    "VehicleID = " + str( self.VehicleID ) + "\n" 
        buf +=    "CameraID = " + str( self.CameraID ) + "\n" 
        buf +=    "GimbalID = " + str( self.GimbalID ) + "\n" 
        buf +=    "HorizontalFOV = " + str( self.HorizontalFOV ) + "\n" 
        buf +=    "AglAltitude = " + str( self.AglAltitude ) + "\n" 
        buf +=    "GimbalElevation = " + str( self.GimbalElevation ) + "\n" 
        buf +=    "AspectRatio = " + str( self.AspectRatio ) + "\n" 
        buf +=    "AchievedGSD = " + str( self.AchievedGSD ) + "\n" 
        buf +=    "CameraWavelength = " + str( self.CameraWavelength ) + "\n" 
        buf +=    "HorizontalToLeadingEdge = " + str( self.HorizontalToLeadingEdge ) + "\n" 
        buf +=    "HorizontalToTrailingEdge = " + str( self.HorizontalToTrailingEdge ) + "\n" 
        buf +=    "HorizontalToCenter = " + str( self.HorizontalToCenter ) + "\n" 
        buf +=    "WidthCenter = " + str( self.WidthCenter ) + "\n" 
        buf +=    "SlantRangeToCenter = " + str( self.SlantRangeToCenter ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "UXTASK") or (len("UXTASK") == 0): # this should never happen
        	# Checks for "UXTASK" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/SensorFootprint")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("UXTASK" + "/SensorFootprint")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        LMCPObject.LMCPObject.toDictMembers(self, d)
        d['FootprintResponseID'] = self.FootprintResponseID
        d['VehicleID'] = self.VehicleID
        d['CameraID'] = self.CameraID
        d['GimbalID'] = self.GimbalID
        d['HorizontalFOV'] = self.HorizontalFOV
        d['AglAltitude'] = self.AglAltitude
        d['GimbalElevation'] = self.GimbalElevation
        d['AspectRatio'] = self.AspectRatio
        d['AchievedGSD'] = self.AchievedGSD
        d['CameraWavelength'] = self.CameraWavelength
        d['HorizontalToLeadingEdge'] = self.HorizontalToLeadingEdge
        d['HorizontalToTrailingEdge'] = self.HorizontalToTrailingEdge
        d['HorizontalToCenter'] = self.HorizontalToCenter
        d['WidthCenter'] = self.WidthCenter
        d['SlantRangeToCenter'] = self.SlantRangeToCenter

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
        str = ws + '<SensorFootprint Series="UXTASK" >\n';
        #str +=LMCPObject.LMCPObject.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</SensorFootprint>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += LMCPObject.LMCPObject.toXMLMembersStr(self, ws)
        buf += ws + "<FootprintResponseID>" + str(self.FootprintResponseID) + "</FootprintResponseID>\n"
        buf += ws + "<VehicleID>" + str(self.VehicleID) + "</VehicleID>\n"
        buf += ws + "<CameraID>" + str(self.CameraID) + "</CameraID>\n"
        buf += ws + "<GimbalID>" + str(self.GimbalID) + "</GimbalID>\n"
        buf += ws + "<HorizontalFOV>" + str(self.HorizontalFOV) + "</HorizontalFOV>\n"
        buf += ws + "<AglAltitude>" + str(self.AglAltitude) + "</AglAltitude>\n"
        buf += ws + "<GimbalElevation>" + str(self.GimbalElevation) + "</GimbalElevation>\n"
        buf += ws + "<AspectRatio>" + str(self.AspectRatio) + "</AspectRatio>\n"
        buf += ws + "<AchievedGSD>" + str(self.AchievedGSD) + "</AchievedGSD>\n"
        buf += ws + "<CameraWavelength>" + WavelengthBand.get_WavelengthBand_int(self.CameraWavelength) + "</CameraWavelength>\n"
        buf += ws + "<HorizontalToLeadingEdge>" + str(self.HorizontalToLeadingEdge) + "</HorizontalToLeadingEdge>\n"
        buf += ws + "<HorizontalToTrailingEdge>" + str(self.HorizontalToTrailingEdge) + "</HorizontalToTrailingEdge>\n"
        buf += ws + "<HorizontalToCenter>" + str(self.HorizontalToCenter) + "</HorizontalToCenter>\n"
        buf += ws + "<WidthCenter>" + str(self.WidthCenter) + "</WidthCenter>\n"
        buf += ws + "<SlantRangeToCenter>" + str(self.SlantRangeToCenter) + "</SlantRangeToCenter>\n"

        return buf
        
