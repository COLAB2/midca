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
from midca.domains.tsp.lmcp.py.afrl.cmasi import AbstractGeometry


class AbstractZone(LMCPObject.LMCPObject):

    def __init__(self):

        self.LMCP_TYPE = 10
        self.SERIES_NAME = "CMASI"
        self.FULL_LMCP_TYPE_NAME = "afrl.cmasi.AbstractZone"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 4849604199710720000
        self.SERIES_VERSION = 3

        #Define message fields
        self.ZoneID = 0   #int64
        self.MinAltitude = 0   #real32
        self.MinAltitudeType = AltitudeType.AltitudeType.AGL   #AltitudeType
        self.MaxAltitude = 0   #real32
        self.MaxAltitudeType = AltitudeType.AltitudeType.MSL   #AltitudeType
        self.AffectedAircraft = []   #int64
        self.StartTime = 0   #int64
        self.EndTime = 0   #int64
        self.Padding = 0   #real32
        self.Label = ""   #string
        self.Boundary = AbstractGeometry.AbstractGeometry()   #AbstractGeometry


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(LMCPObject.LMCPObject.pack(self))
        buffer.extend(struct.pack(">q", self.ZoneID))
        buffer.extend(struct.pack(">f", self.MinAltitude))
        buffer.extend(struct.pack(">i", self.MinAltitudeType))
        buffer.extend(struct.pack(">f", self.MaxAltitude))
        buffer.extend(struct.pack(">i", self.MaxAltitudeType))
        buffer.extend(struct.pack(">H", len(self.AffectedAircraft) ))
        for x in self.AffectedAircraft:
            buffer.extend(struct.pack(">q", x ))
        buffer.extend(struct.pack(">q", self.StartTime))
        buffer.extend(struct.pack(">q", self.EndTime))
        buffer.extend(struct.pack(">f", self.Padding))
        buffer.extend(struct.pack(">H", len(self.Label) ))
        if len(self.Label) > 0:
            if (sys.version_info > (3, 0)):
                buffer.extend(struct.pack( repr(len(self.Label)) + "s", self.Label.encode('utf-8')))
            else:
                buffer.extend(struct.pack( repr(len(self.Label)) + "s", self.Label))
        buffer.extend(struct.pack("B", self.Boundary != None ))
        if self.Boundary != None:
            buffer.extend(struct.pack(">q", self.Boundary.SERIES_NAME_ID))
            buffer.extend(struct.pack(">I", self.Boundary.LMCP_TYPE))
            buffer.extend(struct.pack(">H", self.Boundary.SERIES_VERSION))
            buffer.extend(self.Boundary.pack())

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = LMCPObject.LMCPObject.unpack(self, buffer, _pos)
        self.ZoneID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.MinAltitude = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.MinAltitudeType = struct.unpack_from(">i", buffer, _pos)[0]
        _pos += 4
        self.MaxAltitude = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.MaxAltitudeType = struct.unpack_from(">i", buffer, _pos)[0]
        _pos += 4
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.AffectedAircraft = [None] * _arraylen
        if _arraylen > 0:
            self.AffectedAircraft = struct.unpack_from(">" + repr(_arraylen) + "q", buffer, _pos )
            _pos += 8 * _arraylen
        self.StartTime = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.EndTime = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.Padding = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        _strlen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        if _strlen > 0:
            if (sys.version_info > (3, 0)):
                self.Label = struct.unpack_from( repr(_strlen) + "s", buffer, _pos )[0].decode('utf-8')
            else:
                 self.Label = struct.unpack_from( repr(_strlen) + "s", buffer, _pos )[0]
            _pos += _strlen
        else:
             self.Label = ""
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
            self.Boundary = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
            _pos = self.Boundary.unpack(buffer, _pos)
        else:
            self.Boundary = None
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        LMCPObject.LMCPObject.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "ZoneID" and len(e.childNodes) > 0 :
                    self.ZoneID = int(e.childNodes[0].nodeValue)
                elif e.localName == "MinAltitude" and len(e.childNodes) > 0 :
                    self.MinAltitude = float(e.childNodes[0].nodeValue)
                elif e.localName == "MinAltitudeType" and len(e.childNodes) > 0 :
                    self.MinAltitudeType = AltitudeType.get_AltitudeType_str(e.childNodes[0].nodeValue)
                elif e.localName == "MaxAltitude" and len(e.childNodes) > 0 :
                    self.MaxAltitude = float(e.childNodes[0].nodeValue)
                elif e.localName == "MaxAltitudeType" and len(e.childNodes) > 0 :
                    self.MaxAltitudeType = AltitudeType.get_AltitudeType_str(e.childNodes[0].nodeValue)
                elif e.localName == "AffectedAircraft" and len(e.childNodes) > 0 :
                    self.AffectedAircraft = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.AffectedAircraft.append( int(c.childNodes[0].nodeValue) )
                elif e.localName == "StartTime" and len(e.childNodes) > 0 :
                    self.StartTime = int(e.childNodes[0].nodeValue)
                elif e.localName == "EndTime" and len(e.childNodes) > 0 :
                    self.EndTime = int(e.childNodes[0].nodeValue)
                elif e.localName == "Padding" and len(e.childNodes) > 0 :
                    self.Padding = float(e.childNodes[0].nodeValue)
                elif e.localName == "Label" :
                    self.Label = str(e.childNodes[0].nodeValue) if len(e.childNodes) > 0 else ""
                elif e.localName == "Boundary" and len(e.childNodes) > 0 :
                    for n in e.childNodes:
                        if n.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.Boundary = seriesFactory.createObjectByName(n.getAttribute('Series'), n.localName)
                            if self.Boundary != None:
                                self.Boundary.unpackFromXMLNode(n, seriesFactory)

        return

    def unpackFromDict(self, d, seriesFactory):
        LMCPObject.LMCPObject.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "ZoneID":
                self.ZoneID = d[key]
            elif key == "MinAltitude":
                self.MinAltitude = d[key]
            elif key == "MinAltitudeType":
                self.MinAltitudeType = d[key]
            elif key == "MaxAltitude":
                self.MaxAltitude = d[key]
            elif key == "MaxAltitudeType":
                self.MaxAltitudeType = d[key]
            elif key == "AffectedAircraft":
                self.AffectedAircraft = []
                for c in d[key]:
                    self.AffectedAircraft.append( c )
            elif key == "StartTime":
                self.StartTime = d[key]
            elif key == "EndTime":
                self.EndTime = d[key]
            elif key == "Padding":
                self.Padding = d[key]
            elif key == "Label":
                self.Label = d[key]
            elif key == "Boundary":
                self.Boundary = seriesFactory.unpackFromDict(d[key])

        return

    def get_ZoneID(self):
        return self.ZoneID

    def set_ZoneID(self, value):
        self.ZoneID = int( value )

    def get_MinAltitude(self):
        return self.MinAltitude

    def set_MinAltitude(self, value):
        self.MinAltitude = float( value )

    def get_MinAltitudeType(self):
        return self.MinAltitudeType

    def set_MinAltitudeType(self, value):
        self.MinAltitudeType = value 

    def get_MaxAltitude(self):
        return self.MaxAltitude

    def set_MaxAltitude(self, value):
        self.MaxAltitude = float( value )

    def get_MaxAltitudeType(self):
        return self.MaxAltitudeType

    def set_MaxAltitudeType(self, value):
        self.MaxAltitudeType = value 

    def get_AffectedAircraft(self):
        return self.AffectedAircraft

    def get_StartTime(self):
        return self.StartTime

    def set_StartTime(self, value):
        self.StartTime = int( value )

    def get_EndTime(self):
        return self.EndTime

    def set_EndTime(self, value):
        self.EndTime = int( value )

    def get_Padding(self):
        return self.Padding

    def set_Padding(self, value):
        self.Padding = float( value )

    def get_Label(self):
        return self.Label

    def set_Label(self, value):
        self.Label = str( value )

    def get_Boundary(self):
        return self.Boundary

    def set_Boundary(self, value):
        self.Boundary = value 



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = LMCPObject.LMCPObject.toString(self)
        buf += "From AbstractZone:\n"
        buf +=    "ZoneID = " + str( self.ZoneID ) + "\n" 
        buf +=    "MinAltitude = " + str( self.MinAltitude ) + "\n" 
        buf +=    "MinAltitudeType = " + str( self.MinAltitudeType ) + "\n" 
        buf +=    "MaxAltitude = " + str( self.MaxAltitude ) + "\n" 
        buf +=    "MaxAltitudeType = " + str( self.MaxAltitudeType ) + "\n" 
        buf +=    "AffectedAircraft = " + str( self.AffectedAircraft ) + "\n" 
        buf +=    "StartTime = " + str( self.StartTime ) + "\n" 
        buf +=    "EndTime = " + str( self.EndTime ) + "\n" 
        buf +=    "Padding = " + str( self.Padding ) + "\n" 
        buf +=    "Label = " + str( self.Label ) + "\n" 
        buf +=    "Boundary = " + str( self.Boundary ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "CMASI") or (len("CMASI") == 0): # this should never happen
        	# Checks for "CMASI" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/AbstractZone")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("CMASI" + "/AbstractZone")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        LMCPObject.LMCPObject.toDictMembers(self, d)
        d['ZoneID'] = self.ZoneID
        d['MinAltitude'] = self.MinAltitude
        d['MinAltitudeType'] = self.MinAltitudeType
        d['MaxAltitude'] = self.MaxAltitude
        d['MaxAltitudeType'] = self.MaxAltitudeType
        d['AffectedAircraft'] = []
        for x in self.AffectedAircraft:
            d['AffectedAircraft'].append(x)
        d['StartTime'] = self.StartTime
        d['EndTime'] = self.EndTime
        d['Padding'] = self.Padding
        d['Label'] = self.Label
        if self.Boundary == None:
            d['Boundary'] = None
        else:
            d['Boundary'] = self.Boundary.toDict()

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
        str = ws + '<AbstractZone Series="CMASI" >\n';
        #str +=LMCPObject.LMCPObject.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</AbstractZone>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += LMCPObject.LMCPObject.toXMLMembersStr(self, ws)
        buf += ws + "<ZoneID>" + str(self.ZoneID) + "</ZoneID>\n"
        buf += ws + "<MinAltitude>" + str(self.MinAltitude) + "</MinAltitude>\n"
        buf += ws + "<MinAltitudeType>" + AltitudeType.get_AltitudeType_int(self.MinAltitudeType) + "</MinAltitudeType>\n"
        buf += ws + "<MaxAltitude>" + str(self.MaxAltitude) + "</MaxAltitude>\n"
        buf += ws + "<MaxAltitudeType>" + AltitudeType.get_AltitudeType_int(self.MaxAltitudeType) + "</MaxAltitudeType>\n"
        buf += ws + "<AffectedAircraft>\n"
        for x in self.AffectedAircraft:
            buf += ws + "<int64>" + str(x) + "</int64>\n"
        buf += ws + "</AffectedAircraft>\n"
        buf += ws + "<StartTime>" + str(self.StartTime) + "</StartTime>\n"
        buf += ws + "<EndTime>" + str(self.EndTime) + "</EndTime>\n"
        buf += ws + "<Padding>" + str(self.Padding) + "</Padding>\n"
        buf += ws + "<Label>" + str(self.Label) + "</Label>\n"
        if self.Boundary != None:
            buf += ws + "<Boundary>\n"
            buf += ws + self.Boundary.toXMLStr(ws + "    ") 
            buf += ws + "</Boundary>\n"

        return buf
        
