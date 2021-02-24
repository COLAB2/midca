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
from midca.domains.tsp.lmcp.py.afrl.cmasi import WavelengthBand
from midca.domains.tsp.lmcp.py.afrl.cmasi import FOVOperationMode


class CameraConfiguration(PayloadConfiguration.PayloadConfiguration):

    def __init__(self):
        PayloadConfiguration.PayloadConfiguration.__init__(self)
        self.LMCP_TYPE = 19
        self.SERIES_NAME = "CMASI"
        self.FULL_LMCP_TYPE_NAME = "afrl.cmasi.CameraConfiguration"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 4849604199710720000
        self.SERIES_VERSION = 3

        #Define message fields
        self.SupportedWavelengthBand = WavelengthBand.WavelengthBand.EO   #WavelengthBand
        self.FieldOfViewMode = FOVOperationMode.FOVOperationMode.Continuous   #FOVOperationMode
        self.MinHorizontalFieldOfView = 0   #real32
        self.MaxHorizontalFieldOfView = 0   #real32
        self.DiscreteHorizontalFieldOfViewList = []   #real32
        self.VideoStreamHorizontalResolution = 0   #uint32
        self.VideoStreamVerticalResolution = 0   #uint32


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(PayloadConfiguration.PayloadConfiguration.pack(self))
        buffer.extend(struct.pack(">i", self.SupportedWavelengthBand))
        buffer.extend(struct.pack(">i", self.FieldOfViewMode))
        buffer.extend(struct.pack(">f", self.MinHorizontalFieldOfView))
        buffer.extend(struct.pack(">f", self.MaxHorizontalFieldOfView))
        buffer.extend(struct.pack(">H", len(self.DiscreteHorizontalFieldOfViewList) ))
        for x in self.DiscreteHorizontalFieldOfViewList:
            buffer.extend(struct.pack(">f", x ))
        buffer.extend(struct.pack(">I", self.VideoStreamHorizontalResolution))
        buffer.extend(struct.pack(">I", self.VideoStreamVerticalResolution))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = PayloadConfiguration.PayloadConfiguration.unpack(self, buffer, _pos)
        self.SupportedWavelengthBand = struct.unpack_from(">i", buffer, _pos)[0]
        _pos += 4
        self.FieldOfViewMode = struct.unpack_from(">i", buffer, _pos)[0]
        _pos += 4
        self.MinHorizontalFieldOfView = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.MaxHorizontalFieldOfView = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.DiscreteHorizontalFieldOfViewList = [None] * _arraylen
        if _arraylen > 0:
            self.DiscreteHorizontalFieldOfViewList = struct.unpack_from(">" + repr(_arraylen) + "f", buffer, _pos )
            _pos += 4 * _arraylen
        self.VideoStreamHorizontalResolution = struct.unpack_from(">I", buffer, _pos)[0]
        _pos += 4
        self.VideoStreamVerticalResolution = struct.unpack_from(">I", buffer, _pos)[0]
        _pos += 4
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        PayloadConfiguration.PayloadConfiguration.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "SupportedWavelengthBand" and len(e.childNodes) > 0 :
                    self.SupportedWavelengthBand = WavelengthBand.get_WavelengthBand_str(e.childNodes[0].nodeValue)
                elif e.localName == "FieldOfViewMode" and len(e.childNodes) > 0 :
                    self.FieldOfViewMode = FOVOperationMode.get_FOVOperationMode_str(e.childNodes[0].nodeValue)
                elif e.localName == "MinHorizontalFieldOfView" and len(e.childNodes) > 0 :
                    self.MinHorizontalFieldOfView = float(e.childNodes[0].nodeValue)
                elif e.localName == "MaxHorizontalFieldOfView" and len(e.childNodes) > 0 :
                    self.MaxHorizontalFieldOfView = float(e.childNodes[0].nodeValue)
                elif e.localName == "DiscreteHorizontalFieldOfViewList" and len(e.childNodes) > 0 :
                    self.DiscreteHorizontalFieldOfViewList = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.DiscreteHorizontalFieldOfViewList.append( float(c.childNodes[0].nodeValue) )
                elif e.localName == "VideoStreamHorizontalResolution" and len(e.childNodes) > 0 :
                    self.VideoStreamHorizontalResolution = int(e.childNodes[0].nodeValue)
                elif e.localName == "VideoStreamVerticalResolution" and len(e.childNodes) > 0 :
                    self.VideoStreamVerticalResolution = int(e.childNodes[0].nodeValue)

        return

    def unpackFromDict(self, d, seriesFactory):
        PayloadConfiguration.PayloadConfiguration.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "SupportedWavelengthBand":
                self.SupportedWavelengthBand = d[key]
            elif key == "FieldOfViewMode":
                self.FieldOfViewMode = d[key]
            elif key == "MinHorizontalFieldOfView":
                self.MinHorizontalFieldOfView = d[key]
            elif key == "MaxHorizontalFieldOfView":
                self.MaxHorizontalFieldOfView = d[key]
            elif key == "DiscreteHorizontalFieldOfViewList":
                self.DiscreteHorizontalFieldOfViewList = []
                for c in d[key]:
                    self.DiscreteHorizontalFieldOfViewList.append( c )
            elif key == "VideoStreamHorizontalResolution":
                self.VideoStreamHorizontalResolution = d[key]
            elif key == "VideoStreamVerticalResolution":
                self.VideoStreamVerticalResolution = d[key]

        return

    def get_SupportedWavelengthBand(self):
        return self.SupportedWavelengthBand

    def set_SupportedWavelengthBand(self, value):
        self.SupportedWavelengthBand = value 

    def get_FieldOfViewMode(self):
        return self.FieldOfViewMode

    def set_FieldOfViewMode(self, value):
        self.FieldOfViewMode = value 

    def get_MinHorizontalFieldOfView(self):
        return self.MinHorizontalFieldOfView

    def set_MinHorizontalFieldOfView(self, value):
        self.MinHorizontalFieldOfView = float( value )

    def get_MaxHorizontalFieldOfView(self):
        return self.MaxHorizontalFieldOfView

    def set_MaxHorizontalFieldOfView(self, value):
        self.MaxHorizontalFieldOfView = float( value )

    def get_DiscreteHorizontalFieldOfViewList(self):
        return self.DiscreteHorizontalFieldOfViewList

    def get_VideoStreamHorizontalResolution(self):
        return self.VideoStreamHorizontalResolution

    def set_VideoStreamHorizontalResolution(self, value):
        self.VideoStreamHorizontalResolution = int( value )

    def get_VideoStreamVerticalResolution(self):
        return self.VideoStreamVerticalResolution

    def set_VideoStreamVerticalResolution(self, value):
        self.VideoStreamVerticalResolution = int( value )



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = PayloadConfiguration.PayloadConfiguration.toString(self)
        buf += "From CameraConfiguration:\n"
        buf +=    "SupportedWavelengthBand = " + str( self.SupportedWavelengthBand ) + "\n" 
        buf +=    "FieldOfViewMode = " + str( self.FieldOfViewMode ) + "\n" 
        buf +=    "MinHorizontalFieldOfView = " + str( self.MinHorizontalFieldOfView ) + "\n" 
        buf +=    "MaxHorizontalFieldOfView = " + str( self.MaxHorizontalFieldOfView ) + "\n" 
        buf +=    "DiscreteHorizontalFieldOfViewList = " + str( self.DiscreteHorizontalFieldOfViewList ) + "\n" 
        buf +=    "VideoStreamHorizontalResolution = " + str( self.VideoStreamHorizontalResolution ) + "\n" 
        buf +=    "VideoStreamVerticalResolution = " + str( self.VideoStreamVerticalResolution ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "CMASI") or (len("CMASI") == 0): # this should never happen
        	# Checks for "CMASI" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/CameraConfiguration")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("CMASI" + "/CameraConfiguration")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        PayloadConfiguration.PayloadConfiguration.toDictMembers(self, d)
        d['SupportedWavelengthBand'] = self.SupportedWavelengthBand
        d['FieldOfViewMode'] = self.FieldOfViewMode
        d['MinHorizontalFieldOfView'] = self.MinHorizontalFieldOfView
        d['MaxHorizontalFieldOfView'] = self.MaxHorizontalFieldOfView
        d['DiscreteHorizontalFieldOfViewList'] = []
        for x in self.DiscreteHorizontalFieldOfViewList:
            d['DiscreteHorizontalFieldOfViewList'].append(x)
        d['VideoStreamHorizontalResolution'] = self.VideoStreamHorizontalResolution
        d['VideoStreamVerticalResolution'] = self.VideoStreamVerticalResolution

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
        str = ws + '<CameraConfiguration Series="CMASI" >\n';
        #str +=PayloadConfiguration.PayloadConfiguration.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</CameraConfiguration>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += PayloadConfiguration.PayloadConfiguration.toXMLMembersStr(self, ws)
        buf += ws + "<SupportedWavelengthBand>" + WavelengthBand.get_WavelengthBand_int(self.SupportedWavelengthBand) + "</SupportedWavelengthBand>\n"
        buf += ws + "<FieldOfViewMode>" + FOVOperationMode.get_FOVOperationMode_int(self.FieldOfViewMode) + "</FieldOfViewMode>\n"
        buf += ws + "<MinHorizontalFieldOfView>" + str(self.MinHorizontalFieldOfView) + "</MinHorizontalFieldOfView>\n"
        buf += ws + "<MaxHorizontalFieldOfView>" + str(self.MaxHorizontalFieldOfView) + "</MaxHorizontalFieldOfView>\n"
        buf += ws + "<DiscreteHorizontalFieldOfViewList>\n"
        for x in self.DiscreteHorizontalFieldOfViewList:
            buf += ws + "<real32>" + str(x) + "</real32>\n"
        buf += ws + "</DiscreteHorizontalFieldOfViewList>\n"
        buf += ws + "<VideoStreamHorizontalResolution>" + str(self.VideoStreamHorizontalResolution) + "</VideoStreamHorizontalResolution>\n"
        buf += ws + "<VideoStreamVerticalResolution>" + str(self.VideoStreamVerticalResolution) + "</VideoStreamVerticalResolution>\n"

        return buf
        
