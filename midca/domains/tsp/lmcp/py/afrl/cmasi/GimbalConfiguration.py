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

from midca.domains.tsp.lmcp.py.afrl.cmasi import PayloadConfiguration
from midca.domains.tsp.lmcp.py.afrl.cmasi import GimbalPointingMode


class GimbalConfiguration(PayloadConfiguration.PayloadConfiguration):

    def __init__(self):
        PayloadConfiguration.PayloadConfiguration.__init__(self)
        self.LMCP_TYPE = 24
        self.SERIES_NAME = "CMASI"
        self.FULL_LMCP_TYPE_NAME = "afrl.cmasi.GimbalConfiguration"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 4849604199710720000
        self.SERIES_VERSION = 3

        #Define message fields
        self.SupportedPointingModes = []   #GimbalPointingMode
        self.MinAzimuth = -180   #real32
        self.MaxAzimuth = 180   #real32
        self.IsAzimuthClamped = False   #bool
        self.MinElevation = -180   #real32
        self.MaxElevation = 180   #real32
        self.IsElevationClamped = False   #bool
        self.MinRotation = 0   #real32
        self.MaxRotation = 0   #real32
        self.IsRotationClamped = True   #bool
        self.MaxAzimuthSlewRate = 0   #real32
        self.MaxElevationSlewRate = 0   #real32
        self.MaxRotationRate = 0   #real32
        self.ContainedPayloadList = []   #int64


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(PayloadConfiguration.PayloadConfiguration.pack(self))
        buffer.extend(struct.pack(">H", len(self.SupportedPointingModes) ))
        for x in self.SupportedPointingModes:
            buffer.extend(struct.pack(">i", x ))
        buffer.extend(struct.pack(">f", self.MinAzimuth))
        buffer.extend(struct.pack(">f", self.MaxAzimuth))
        boolChar = 1 if self.IsAzimuthClamped == True else 0
        buffer.extend(struct.pack(">B",boolChar))
        buffer.extend(struct.pack(">f", self.MinElevation))
        buffer.extend(struct.pack(">f", self.MaxElevation))
        boolChar = 1 if self.IsElevationClamped == True else 0
        buffer.extend(struct.pack(">B",boolChar))
        buffer.extend(struct.pack(">f", self.MinRotation))
        buffer.extend(struct.pack(">f", self.MaxRotation))
        boolChar = 1 if self.IsRotationClamped == True else 0
        buffer.extend(struct.pack(">B",boolChar))
        buffer.extend(struct.pack(">f", self.MaxAzimuthSlewRate))
        buffer.extend(struct.pack(">f", self.MaxElevationSlewRate))
        buffer.extend(struct.pack(">f", self.MaxRotationRate))
        buffer.extend(struct.pack(">H", len(self.ContainedPayloadList) ))
        for x in self.ContainedPayloadList:
            buffer.extend(struct.pack(">q", x ))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = PayloadConfiguration.PayloadConfiguration.unpack(self, buffer, _pos)
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.SupportedPointingModes = [None] * _arraylen
        if _arraylen > 0:
            self.SupportedPointingModes = struct.unpack_from(">" + repr(_arraylen) + "i", buffer, _pos )
            _pos += 4 * _arraylen
        self.MinAzimuth = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.MaxAzimuth = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        boolChar = struct.unpack_from(">B", buffer, _pos)[0]
        self.IsAzimuthClamped = True if boolChar == 1 else False
        _pos += 1
        self.MinElevation = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.MaxElevation = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        boolChar = struct.unpack_from(">B", buffer, _pos)[0]
        self.IsElevationClamped = True if boolChar == 1 else False
        _pos += 1
        self.MinRotation = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.MaxRotation = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        boolChar = struct.unpack_from(">B", buffer, _pos)[0]
        self.IsRotationClamped = True if boolChar == 1 else False
        _pos += 1
        self.MaxAzimuthSlewRate = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.MaxElevationSlewRate = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.MaxRotationRate = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.ContainedPayloadList = [None] * _arraylen
        if _arraylen > 0:
            self.ContainedPayloadList = struct.unpack_from(">" + repr(_arraylen) + "q", buffer, _pos )
            _pos += 8 * _arraylen
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        PayloadConfiguration.PayloadConfiguration.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "SupportedPointingModes" and len(e.childNodes) > 0 :
                    self.SupportedPointingModes = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.SupportedPointingModes.append( GimbalPointingMode.get_GimbalPointingMode_str(c.childNodes[0].nodeValue) )
                elif e.localName == "MinAzimuth" and len(e.childNodes) > 0 :
                    self.MinAzimuth = float(e.childNodes[0].nodeValue)
                elif e.localName == "MaxAzimuth" and len(e.childNodes) > 0 :
                    self.MaxAzimuth = float(e.childNodes[0].nodeValue)
                elif e.localName == "IsAzimuthClamped" and len(e.childNodes) > 0 :
                    self.IsAzimuthClamped = e.childNodes[0].nodeValue.lower() == 'true'
                elif e.localName == "MinElevation" and len(e.childNodes) > 0 :
                    self.MinElevation = float(e.childNodes[0].nodeValue)
                elif e.localName == "MaxElevation" and len(e.childNodes) > 0 :
                    self.MaxElevation = float(e.childNodes[0].nodeValue)
                elif e.localName == "IsElevationClamped" and len(e.childNodes) > 0 :
                    self.IsElevationClamped = e.childNodes[0].nodeValue.lower() == 'true'
                elif e.localName == "MinRotation" and len(e.childNodes) > 0 :
                    self.MinRotation = float(e.childNodes[0].nodeValue)
                elif e.localName == "MaxRotation" and len(e.childNodes) > 0 :
                    self.MaxRotation = float(e.childNodes[0].nodeValue)
                elif e.localName == "IsRotationClamped" and len(e.childNodes) > 0 :
                    self.IsRotationClamped = e.childNodes[0].nodeValue.lower() == 'true'
                elif e.localName == "MaxAzimuthSlewRate" and len(e.childNodes) > 0 :
                    self.MaxAzimuthSlewRate = float(e.childNodes[0].nodeValue)
                elif e.localName == "MaxElevationSlewRate" and len(e.childNodes) > 0 :
                    self.MaxElevationSlewRate = float(e.childNodes[0].nodeValue)
                elif e.localName == "MaxRotationRate" and len(e.childNodes) > 0 :
                    self.MaxRotationRate = float(e.childNodes[0].nodeValue)
                elif e.localName == "ContainedPayloadList" and len(e.childNodes) > 0 :
                    self.ContainedPayloadList = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.ContainedPayloadList.append( int(c.childNodes[0].nodeValue) )

        return

    def unpackFromDict(self, d, seriesFactory):
        PayloadConfiguration.PayloadConfiguration.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "SupportedPointingModes":
                self.SupportedPointingModes = []
                for c in d[key]:
                    self.SupportedPointingModes.append( c )
            elif key == "MinAzimuth":
                self.MinAzimuth = d[key]
            elif key == "MaxAzimuth":
                self.MaxAzimuth = d[key]
            elif key == "IsAzimuthClamped":
                self.IsAzimuthClamped = d[key]
            elif key == "MinElevation":
                self.MinElevation = d[key]
            elif key == "MaxElevation":
                self.MaxElevation = d[key]
            elif key == "IsElevationClamped":
                self.IsElevationClamped = d[key]
            elif key == "MinRotation":
                self.MinRotation = d[key]
            elif key == "MaxRotation":
                self.MaxRotation = d[key]
            elif key == "IsRotationClamped":
                self.IsRotationClamped = d[key]
            elif key == "MaxAzimuthSlewRate":
                self.MaxAzimuthSlewRate = d[key]
            elif key == "MaxElevationSlewRate":
                self.MaxElevationSlewRate = d[key]
            elif key == "MaxRotationRate":
                self.MaxRotationRate = d[key]
            elif key == "ContainedPayloadList":
                self.ContainedPayloadList = []
                for c in d[key]:
                    self.ContainedPayloadList.append( c )

        return

    def get_SupportedPointingModes(self):
        return self.SupportedPointingModes

    def get_MinAzimuth(self):
        return self.MinAzimuth

    def set_MinAzimuth(self, value):
        self.MinAzimuth = float( value )

    def get_MaxAzimuth(self):
        return self.MaxAzimuth

    def set_MaxAzimuth(self, value):
        self.MaxAzimuth = float( value )

    def get_IsAzimuthClamped(self):
        return self.IsAzimuthClamped

    def set_IsAzimuthClamped(self, value):
        self.IsAzimuthClamped = bool( value )

    def get_MinElevation(self):
        return self.MinElevation

    def set_MinElevation(self, value):
        self.MinElevation = float( value )

    def get_MaxElevation(self):
        return self.MaxElevation

    def set_MaxElevation(self, value):
        self.MaxElevation = float( value )

    def get_IsElevationClamped(self):
        return self.IsElevationClamped

    def set_IsElevationClamped(self, value):
        self.IsElevationClamped = bool( value )

    def get_MinRotation(self):
        return self.MinRotation

    def set_MinRotation(self, value):
        self.MinRotation = float( value )

    def get_MaxRotation(self):
        return self.MaxRotation

    def set_MaxRotation(self, value):
        self.MaxRotation = float( value )

    def get_IsRotationClamped(self):
        return self.IsRotationClamped

    def set_IsRotationClamped(self, value):
        self.IsRotationClamped = bool( value )

    def get_MaxAzimuthSlewRate(self):
        return self.MaxAzimuthSlewRate

    def set_MaxAzimuthSlewRate(self, value):
        self.MaxAzimuthSlewRate = float( value )

    def get_MaxElevationSlewRate(self):
        return self.MaxElevationSlewRate

    def set_MaxElevationSlewRate(self, value):
        self.MaxElevationSlewRate = float( value )

    def get_MaxRotationRate(self):
        return self.MaxRotationRate

    def set_MaxRotationRate(self, value):
        self.MaxRotationRate = float( value )

    def get_ContainedPayloadList(self):
        return self.ContainedPayloadList



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = PayloadConfiguration.PayloadConfiguration.toString(self)
        buf += "From GimbalConfiguration:\n"
        buf +=    "SupportedPointingModes = " + str( self.SupportedPointingModes ) + "\n" 
        buf +=    "MinAzimuth = " + str( self.MinAzimuth ) + "\n" 
        buf +=    "MaxAzimuth = " + str( self.MaxAzimuth ) + "\n" 
        buf +=    "IsAzimuthClamped = " + str( self.IsAzimuthClamped ) + "\n" 
        buf +=    "MinElevation = " + str( self.MinElevation ) + "\n" 
        buf +=    "MaxElevation = " + str( self.MaxElevation ) + "\n" 
        buf +=    "IsElevationClamped = " + str( self.IsElevationClamped ) + "\n" 
        buf +=    "MinRotation = " + str( self.MinRotation ) + "\n" 
        buf +=    "MaxRotation = " + str( self.MaxRotation ) + "\n" 
        buf +=    "IsRotationClamped = " + str( self.IsRotationClamped ) + "\n" 
        buf +=    "MaxAzimuthSlewRate = " + str( self.MaxAzimuthSlewRate ) + "\n" 
        buf +=    "MaxElevationSlewRate = " + str( self.MaxElevationSlewRate ) + "\n" 
        buf +=    "MaxRotationRate = " + str( self.MaxRotationRate ) + "\n" 
        buf +=    "ContainedPayloadList = " + str( self.ContainedPayloadList ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "CMASI") or (len("CMASI") == 0): # this should never happen
        	# Checks for "CMASI" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/GimbalConfiguration")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("CMASI" + "/GimbalConfiguration")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        PayloadConfiguration.PayloadConfiguration.toDictMembers(self, d)
        d['SupportedPointingModes'] = []
        for x in self.SupportedPointingModes:
            d['SupportedPointingModes'].append(x)
        d['MinAzimuth'] = self.MinAzimuth
        d['MaxAzimuth'] = self.MaxAzimuth
        d['IsAzimuthClamped'] = self.IsAzimuthClamped
        d['MinElevation'] = self.MinElevation
        d['MaxElevation'] = self.MaxElevation
        d['IsElevationClamped'] = self.IsElevationClamped
        d['MinRotation'] = self.MinRotation
        d['MaxRotation'] = self.MaxRotation
        d['IsRotationClamped'] = self.IsRotationClamped
        d['MaxAzimuthSlewRate'] = self.MaxAzimuthSlewRate
        d['MaxElevationSlewRate'] = self.MaxElevationSlewRate
        d['MaxRotationRate'] = self.MaxRotationRate
        d['ContainedPayloadList'] = []
        for x in self.ContainedPayloadList:
            d['ContainedPayloadList'].append(x)

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
        str = ws + '<GimbalConfiguration Series="CMASI" >\n';
        #str +=PayloadConfiguration.PayloadConfiguration.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</GimbalConfiguration>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += PayloadConfiguration.PayloadConfiguration.toXMLMembersStr(self, ws)
        buf += ws + "<SupportedPointingModes>\n"
        for x in self.SupportedPointingModes:
            buf += ws + "<GimbalPointingMode>" + GimbalPointingMode.get_GimbalPointingMode_int(x) + "</GimbalPointingMode>\n"
        buf += ws + "</SupportedPointingModes>\n"
        buf += ws + "<MinAzimuth>" + str(self.MinAzimuth) + "</MinAzimuth>\n"
        buf += ws + "<MaxAzimuth>" + str(self.MaxAzimuth) + "</MaxAzimuth>\n"
        buf += ws + "<IsAzimuthClamped>" + ('True' if self.IsAzimuthClamped else 'False') + "</IsAzimuthClamped>\n"
        buf += ws + "<MinElevation>" + str(self.MinElevation) + "</MinElevation>\n"
        buf += ws + "<MaxElevation>" + str(self.MaxElevation) + "</MaxElevation>\n"
        buf += ws + "<IsElevationClamped>" + ('True' if self.IsElevationClamped else 'False') + "</IsElevationClamped>\n"
        buf += ws + "<MinRotation>" + str(self.MinRotation) + "</MinRotation>\n"
        buf += ws + "<MaxRotation>" + str(self.MaxRotation) + "</MaxRotation>\n"
        buf += ws + "<IsRotationClamped>" + ('True' if self.IsRotationClamped else 'False') + "</IsRotationClamped>\n"
        buf += ws + "<MaxAzimuthSlewRate>" + str(self.MaxAzimuthSlewRate) + "</MaxAzimuthSlewRate>\n"
        buf += ws + "<MaxElevationSlewRate>" + str(self.MaxElevationSlewRate) + "</MaxElevationSlewRate>\n"
        buf += ws + "<MaxRotationRate>" + str(self.MaxRotationRate) + "</MaxRotationRate>\n"
        buf += ws + "<ContainedPayloadList>\n"
        for x in self.ContainedPayloadList:
            buf += ws + "<int64>" + str(x) + "</int64>\n"
        buf += ws + "</ContainedPayloadList>\n"

        return buf
        
