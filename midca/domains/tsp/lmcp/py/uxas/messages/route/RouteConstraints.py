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


class RouteConstraints(LMCPObject.LMCPObject):

    def __init__(self):

        self.LMCP_TYPE = 4
        self.SERIES_NAME = "ROUTE"
        self.FULL_LMCP_TYPE_NAME = "uxas.messages.route.RouteConstraints"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 5931053054693474304
        self.SERIES_VERSION = 4

        #Define message fields
        self.RouteID = 0   #int64
        self.StartLocation = Location3D.Location3D()   #Location3D
        self.StartHeading = 0   #real32
        self.UseStartHeading = True   #bool
        self.EndLocation = Location3D.Location3D()   #Location3D
        self.EndHeading = 0   #real32
        self.UseEndHeading = True   #bool


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(LMCPObject.LMCPObject.pack(self))
        buffer.extend(struct.pack(">q", self.RouteID))
        buffer.extend(struct.pack("B", self.StartLocation != None ))
        if self.StartLocation != None:
            buffer.extend(struct.pack(">q", self.StartLocation.SERIES_NAME_ID))
            buffer.extend(struct.pack(">I", self.StartLocation.LMCP_TYPE))
            buffer.extend(struct.pack(">H", self.StartLocation.SERIES_VERSION))
            buffer.extend(self.StartLocation.pack())
        buffer.extend(struct.pack(">f", self.StartHeading))
        boolChar = 1 if self.UseStartHeading == True else 0
        buffer.extend(struct.pack(">B",boolChar))
        buffer.extend(struct.pack("B", self.EndLocation != None ))
        if self.EndLocation != None:
            buffer.extend(struct.pack(">q", self.EndLocation.SERIES_NAME_ID))
            buffer.extend(struct.pack(">I", self.EndLocation.LMCP_TYPE))
            buffer.extend(struct.pack(">H", self.EndLocation.SERIES_VERSION))
            buffer.extend(self.EndLocation.pack())
        buffer.extend(struct.pack(">f", self.EndHeading))
        boolChar = 1 if self.UseEndHeading == True else 0
        buffer.extend(struct.pack(">B",boolChar))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = LMCPObject.LMCPObject.unpack(self, buffer, _pos)
        self.RouteID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
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
            self.StartLocation = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
            _pos = self.StartLocation.unpack(buffer, _pos)
        else:
            self.StartLocation = None
        self.StartHeading = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        boolChar = struct.unpack_from(">B", buffer, _pos)[0]
        self.UseStartHeading = True if boolChar == 1 else False
        _pos += 1
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
            self.EndLocation = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
            _pos = self.EndLocation.unpack(buffer, _pos)
        else:
            self.EndLocation = None
        self.EndHeading = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        boolChar = struct.unpack_from(">B", buffer, _pos)[0]
        self.UseEndHeading = True if boolChar == 1 else False
        _pos += 1
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        LMCPObject.LMCPObject.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "RouteID" and len(e.childNodes) > 0 :
                    self.RouteID = int(e.childNodes[0].nodeValue)
                elif e.localName == "StartLocation" and len(e.childNodes) > 0 :
                    for n in e.childNodes:
                        if n.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.StartLocation = seriesFactory.createObjectByName(n.getAttribute('Series'), n.localName)
                            if self.StartLocation != None:
                                self.StartLocation.unpackFromXMLNode(n, seriesFactory)
                elif e.localName == "StartHeading" and len(e.childNodes) > 0 :
                    self.StartHeading = float(e.childNodes[0].nodeValue)
                elif e.localName == "UseStartHeading" and len(e.childNodes) > 0 :
                    self.UseStartHeading = e.childNodes[0].nodeValue.lower() == 'true'
                elif e.localName == "EndLocation" and len(e.childNodes) > 0 :
                    for n in e.childNodes:
                        if n.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.EndLocation = seriesFactory.createObjectByName(n.getAttribute('Series'), n.localName)
                            if self.EndLocation != None:
                                self.EndLocation.unpackFromXMLNode(n, seriesFactory)
                elif e.localName == "EndHeading" and len(e.childNodes) > 0 :
                    self.EndHeading = float(e.childNodes[0].nodeValue)
                elif e.localName == "UseEndHeading" and len(e.childNodes) > 0 :
                    self.UseEndHeading = e.childNodes[0].nodeValue.lower() == 'true'

        return

    def unpackFromDict(self, d, seriesFactory):
        LMCPObject.LMCPObject.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "RouteID":
                self.RouteID = d[key]
            elif key == "StartLocation":
                self.StartLocation = seriesFactory.unpackFromDict(d[key])
            elif key == "StartHeading":
                self.StartHeading = d[key]
            elif key == "UseStartHeading":
                self.UseStartHeading = d[key]
            elif key == "EndLocation":
                self.EndLocation = seriesFactory.unpackFromDict(d[key])
            elif key == "EndHeading":
                self.EndHeading = d[key]
            elif key == "UseEndHeading":
                self.UseEndHeading = d[key]

        return

    def get_RouteID(self):
        return self.RouteID

    def set_RouteID(self, value):
        self.RouteID = int( value )

    def get_StartLocation(self):
        return self.StartLocation

    def set_StartLocation(self, value):
        self.StartLocation = value 

    def get_StartHeading(self):
        return self.StartHeading

    def set_StartHeading(self, value):
        self.StartHeading = float( value )

    def get_UseStartHeading(self):
        return self.UseStartHeading

    def set_UseStartHeading(self, value):
        self.UseStartHeading = bool( value )

    def get_EndLocation(self):
        return self.EndLocation

    def set_EndLocation(self, value):
        self.EndLocation = value 

    def get_EndHeading(self):
        return self.EndHeading

    def set_EndHeading(self, value):
        self.EndHeading = float( value )

    def get_UseEndHeading(self):
        return self.UseEndHeading

    def set_UseEndHeading(self, value):
        self.UseEndHeading = bool( value )



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = LMCPObject.LMCPObject.toString(self)
        buf += "From RouteConstraints:\n"
        buf +=    "RouteID = " + str( self.RouteID ) + "\n" 
        buf +=    "StartLocation = " + str( self.StartLocation ) + "\n" 
        buf +=    "StartHeading = " + str( self.StartHeading ) + "\n" 
        buf +=    "UseStartHeading = " + str( self.UseStartHeading ) + "\n" 
        buf +=    "EndLocation = " + str( self.EndLocation ) + "\n" 
        buf +=    "EndHeading = " + str( self.EndHeading ) + "\n" 
        buf +=    "UseEndHeading = " + str( self.UseEndHeading ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "ROUTE") or (len("ROUTE") == 0): # this should never happen
        	# Checks for "ROUTE" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/RouteConstraints")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("ROUTE" + "/RouteConstraints")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        LMCPObject.LMCPObject.toDictMembers(self, d)
        d['RouteID'] = self.RouteID
        if self.StartLocation == None:
            d['StartLocation'] = None
        else:
            d['StartLocation'] = self.StartLocation.toDict()
        d['StartHeading'] = self.StartHeading
        d['UseStartHeading'] = self.UseStartHeading
        if self.EndLocation == None:
            d['EndLocation'] = None
        else:
            d['EndLocation'] = self.EndLocation.toDict()
        d['EndHeading'] = self.EndHeading
        d['UseEndHeading'] = self.UseEndHeading

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
        str = ws + '<RouteConstraints Series="ROUTE" >\n';
        #str +=LMCPObject.LMCPObject.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</RouteConstraints>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += LMCPObject.LMCPObject.toXMLMembersStr(self, ws)
        buf += ws + "<RouteID>" + str(self.RouteID) + "</RouteID>\n"
        if self.StartLocation != None:
            buf += ws + "<StartLocation>\n"
            buf += ws + self.StartLocation.toXMLStr(ws + "    ") 
            buf += ws + "</StartLocation>\n"
        buf += ws + "<StartHeading>" + str(self.StartHeading) + "</StartHeading>\n"
        buf += ws + "<UseStartHeading>" + ('True' if self.UseStartHeading else 'False') + "</UseStartHeading>\n"
        if self.EndLocation != None:
            buf += ws + "<EndLocation>\n"
            buf += ws + self.EndLocation.toXMLStr(ws + "    ") 
            buf += ws + "</EndLocation>\n"
        buf += ws + "<EndHeading>" + str(self.EndHeading) + "</EndHeading>\n"
        buf += ws + "<UseEndHeading>" + ('True' if self.UseEndHeading else 'False') + "</UseEndHeading>\n"

        return buf
        
