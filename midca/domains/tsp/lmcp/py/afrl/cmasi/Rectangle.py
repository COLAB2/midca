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

from midca.domains.tsp.lmcp.py.afrl.cmasi import AbstractGeometry
from midca.domains.tsp.lmcp.py.afrl.cmasi import Location3D


class Rectangle(AbstractGeometry.AbstractGeometry):

    def __init__(self):
        AbstractGeometry.AbstractGeometry.__init__(self)
        self.LMCP_TYPE = 43
        self.SERIES_NAME = "CMASI"
        self.FULL_LMCP_TYPE_NAME = "afrl.cmasi.Rectangle"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 4849604199710720000
        self.SERIES_VERSION = 3

        #Define message fields
        self.CenterPoint = Location3D.Location3D()   #Location3D
        self.Width = 0   #real32
        self.Height = 0   #real32
        self.Rotation = 0   #real32


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(AbstractGeometry.AbstractGeometry.pack(self))
        buffer.extend(struct.pack("B", self.CenterPoint != None ))
        if self.CenterPoint != None:
            buffer.extend(struct.pack(">q", self.CenterPoint.SERIES_NAME_ID))
            buffer.extend(struct.pack(">I", self.CenterPoint.LMCP_TYPE))
            buffer.extend(struct.pack(">H", self.CenterPoint.SERIES_VERSION))
            buffer.extend(self.CenterPoint.pack())
        buffer.extend(struct.pack(">f", self.Width))
        buffer.extend(struct.pack(">f", self.Height))
        buffer.extend(struct.pack(">f", self.Rotation))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = AbstractGeometry.AbstractGeometry.unpack(self, buffer, _pos)
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
            self.CenterPoint = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
            _pos = self.CenterPoint.unpack(buffer, _pos)
        else:
            self.CenterPoint = None
        self.Width = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.Height = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.Rotation = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        AbstractGeometry.AbstractGeometry.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "CenterPoint" and len(e.childNodes) > 0 :
                    for n in e.childNodes:
                        if n.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.CenterPoint = seriesFactory.createObjectByName(n.getAttribute('Series'), n.localName)
                            if self.CenterPoint != None:
                                self.CenterPoint.unpackFromXMLNode(n, seriesFactory)
                elif e.localName == "Width" and len(e.childNodes) > 0 :
                    self.Width = float(e.childNodes[0].nodeValue)
                elif e.localName == "Height" and len(e.childNodes) > 0 :
                    self.Height = float(e.childNodes[0].nodeValue)
                elif e.localName == "Rotation" and len(e.childNodes) > 0 :
                    self.Rotation = float(e.childNodes[0].nodeValue)

        return

    def unpackFromDict(self, d, seriesFactory):
        AbstractGeometry.AbstractGeometry.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "CenterPoint":
                self.CenterPoint = seriesFactory.unpackFromDict(d[key])
            elif key == "Width":
                self.Width = d[key]
            elif key == "Height":
                self.Height = d[key]
            elif key == "Rotation":
                self.Rotation = d[key]

        return

    def get_CenterPoint(self):
        return self.CenterPoint

    def set_CenterPoint(self, value):
        self.CenterPoint = value 

    def get_Width(self):
        return self.Width

    def set_Width(self, value):
        self.Width = float( value )

    def get_Height(self):
        return self.Height

    def set_Height(self, value):
        self.Height = float( value )

    def get_Rotation(self):
        return self.Rotation

    def set_Rotation(self, value):
        self.Rotation = float( value )



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = AbstractGeometry.AbstractGeometry.toString(self)
        buf += "From Rectangle:\n"
        buf +=    "CenterPoint = " + str( self.CenterPoint ) + "\n" 
        buf +=    "Width = " + str( self.Width ) + "\n" 
        buf +=    "Height = " + str( self.Height ) + "\n" 
        buf +=    "Rotation = " + str( self.Rotation ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "CMASI") or (len("CMASI") == 0): # this should never happen
        	# Checks for "CMASI" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/Rectangle")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("CMASI" + "/Rectangle")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        AbstractGeometry.AbstractGeometry.toDictMembers(self, d)
        if self.CenterPoint == None:
            d['CenterPoint'] = None
        else:
            d['CenterPoint'] = self.CenterPoint.toDict()
        d['Width'] = self.Width
        d['Height'] = self.Height
        d['Rotation'] = self.Rotation

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
        str = ws + '<Rectangle Series="CMASI" >\n';
        #str +=AbstractGeometry.AbstractGeometry.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</Rectangle>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += AbstractGeometry.AbstractGeometry.toXMLMembersStr(self, ws)
        if self.CenterPoint != None:
            buf += ws + "<CenterPoint>\n"
            buf += ws + self.CenterPoint.toXMLStr(ws + "    ") 
            buf += ws + "</CenterPoint>\n"
        buf += ws + "<Width>" + str(self.Width) + "</Width>\n"
        buf += ws + "<Height>" + str(self.Height) + "</Height>\n"
        buf += ws + "<Rotation>" + str(self.Rotation) + "</Rotation>\n"

        return buf
        
