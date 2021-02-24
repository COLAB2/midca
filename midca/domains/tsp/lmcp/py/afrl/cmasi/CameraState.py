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

from midca.domains.tsp.lmcp.py.afrl.cmasi import GimballedPayloadState
from midca.domains.tsp.lmcp.py.afrl.cmasi import Location3D


class CameraState(GimballedPayloadState.GimballedPayloadState):

    def __init__(self):
        GimballedPayloadState.GimballedPayloadState.__init__(self)
        self.LMCP_TYPE = 21
        self.SERIES_NAME = "CMASI"
        self.FULL_LMCP_TYPE_NAME = "afrl.cmasi.CameraState"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 4849604199710720000
        self.SERIES_VERSION = 3

        #Define message fields
        self.HorizontalFieldOfView = 0   #real32
        self.VerticalFieldOfView = 0   #real32
        self.Footprint = []   #Location3D
        self.Centerpoint = None   #Location3D


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(GimballedPayloadState.GimballedPayloadState.pack(self))
        buffer.extend(struct.pack(">f", self.HorizontalFieldOfView))
        buffer.extend(struct.pack(">f", self.VerticalFieldOfView))
        buffer.extend(struct.pack(">H", len(self.Footprint) ))
        for x in self.Footprint:
           buffer.extend(struct.pack("B", x != None ))
           if x != None:
               buffer.extend(struct.pack(">q", x.SERIES_NAME_ID))
               buffer.extend(struct.pack(">I", x.LMCP_TYPE))
               buffer.extend(struct.pack(">H", x.SERIES_VERSION))
               buffer.extend(x.pack())
        buffer.extend(struct.pack("B", self.Centerpoint != None ))
        if self.Centerpoint != None:
            buffer.extend(struct.pack(">q", self.Centerpoint.SERIES_NAME_ID))
            buffer.extend(struct.pack(">I", self.Centerpoint.LMCP_TYPE))
            buffer.extend(struct.pack(">H", self.Centerpoint.SERIES_VERSION))
            buffer.extend(self.Centerpoint.pack())

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = GimballedPayloadState.GimballedPayloadState.unpack(self, buffer, _pos)
        self.HorizontalFieldOfView = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.VerticalFieldOfView = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.Footprint = [None] * _arraylen
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
                self.Footprint[x] = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
                _pos = self.Footprint[x].unpack(buffer, _pos)
            else:
                self.Footprint[x] = None
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
            self.Centerpoint = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
            _pos = self.Centerpoint.unpack(buffer, _pos)
        else:
            self.Centerpoint = None
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        GimballedPayloadState.GimballedPayloadState.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "HorizontalFieldOfView" and len(e.childNodes) > 0 :
                    self.HorizontalFieldOfView = float(e.childNodes[0].nodeValue)
                elif e.localName == "VerticalFieldOfView" and len(e.childNodes) > 0 :
                    self.VerticalFieldOfView = float(e.childNodes[0].nodeValue)
                elif e.localName == "Footprint" and len(e.childNodes) > 0 :
                    self.Footprint = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            obj = seriesFactory.createObjectByName(c.getAttribute('Series'), c.localName)
                            if obj != None:
                                obj.unpackFromXMLNode(c, seriesFactory)
                                self.Footprint.append(obj)
                elif e.localName == "Centerpoint" and len(e.childNodes) > 0 :
                    for n in e.childNodes:
                        if n.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.Centerpoint = seriesFactory.createObjectByName(n.getAttribute('Series'), n.localName)
                            if self.Centerpoint != None:
                                self.Centerpoint.unpackFromXMLNode(n, seriesFactory)

        return

    def unpackFromDict(self, d, seriesFactory):
        GimballedPayloadState.GimballedPayloadState.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "HorizontalFieldOfView":
                self.HorizontalFieldOfView = d[key]
            elif key == "VerticalFieldOfView":
                self.VerticalFieldOfView = d[key]
            elif key == "Footprint":
                self.Footprint = []
                for c in d[key]:
                    obj = seriesFactory.unpackFromDict(c)
                    if obj != None:
                        self.Footprint.append(obj)
            elif key == "Centerpoint":
                self.Centerpoint = seriesFactory.unpackFromDict(d[key])

        return

    def get_HorizontalFieldOfView(self):
        return self.HorizontalFieldOfView

    def set_HorizontalFieldOfView(self, value):
        self.HorizontalFieldOfView = float( value )

    def get_VerticalFieldOfView(self):
        return self.VerticalFieldOfView

    def set_VerticalFieldOfView(self, value):
        self.VerticalFieldOfView = float( value )

    def get_Footprint(self):
        return self.Footprint

    def get_Centerpoint(self):
        return self.Centerpoint

    def set_Centerpoint(self, value):
        self.Centerpoint = value 



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = GimballedPayloadState.GimballedPayloadState.toString(self)
        buf += "From CameraState:\n"
        buf +=    "HorizontalFieldOfView = " + str( self.HorizontalFieldOfView ) + "\n" 
        buf +=    "VerticalFieldOfView = " + str( self.VerticalFieldOfView ) + "\n" 
        buf +=    "Footprint = " + str( self.Footprint ) + "\n" 
        buf +=    "Centerpoint = " + str( self.Centerpoint ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "CMASI") or (len("CMASI") == 0): # this should never happen
        	# Checks for "CMASI" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/CameraState")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("CMASI" + "/CameraState")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        GimballedPayloadState.GimballedPayloadState.toDictMembers(self, d)
        d['HorizontalFieldOfView'] = self.HorizontalFieldOfView
        d['VerticalFieldOfView'] = self.VerticalFieldOfView
        d['Footprint'] = []
        for x in self.Footprint:
            if x == None:
                d['Footprint'].append(None)
            else:
                d['Footprint'].append(x.toDict())
        if self.Centerpoint == None:
            d['Centerpoint'] = None
        else:
            d['Centerpoint'] = self.Centerpoint.toDict()

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
        str = ws + '<CameraState Series="CMASI" >\n';
        #str +=GimballedPayloadState.GimballedPayloadState.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</CameraState>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += GimballedPayloadState.GimballedPayloadState.toXMLMembersStr(self, ws)
        buf += ws + "<HorizontalFieldOfView>" + str(self.HorizontalFieldOfView) + "</HorizontalFieldOfView>\n"
        buf += ws + "<VerticalFieldOfView>" + str(self.VerticalFieldOfView) + "</VerticalFieldOfView>\n"
        buf += ws + "<Footprint>\n"
        for x in self.Footprint:
            if x == None:
                buf += ws + "    <null/>\n"
            else:
                buf += x.toXMLStr(ws + "    ") 
        buf += ws + "</Footprint>\n"
        if self.Centerpoint != None:
            buf += ws + "<Centerpoint>\n"
            buf += ws + self.Centerpoint.toXMLStr(ws + "    ") 
            buf += ws + "</Centerpoint>\n"

        return buf
        
