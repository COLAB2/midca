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


class FootprintRequest(LMCPObject.LMCPObject):

    def __init__(self):

        self.LMCP_TYPE = 11
        self.SERIES_NAME = "UXTASK"
        self.FULL_LMCP_TYPE_NAME = "uxas.messages.task.FootprintRequest"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 6149757930721443840
        self.SERIES_VERSION = 8

        #Define message fields
        self.FootprintRequestID = 0   #int64
        self.VehicleID = 0   #int64
        self.EligibleWavelengths = []   #WavelengthBand
        self.GroundSampleDistances = []   #real32
        self.AglAltitudes = []   #real32
        self.ElevationAngles = []   #real32


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(LMCPObject.LMCPObject.pack(self))
        buffer.extend(struct.pack(">q", self.FootprintRequestID))
        buffer.extend(struct.pack(">q", self.VehicleID))
        buffer.extend(struct.pack(">H", len(self.EligibleWavelengths) ))
        for x in self.EligibleWavelengths:
            buffer.extend(struct.pack(">i", x ))
        buffer.extend(struct.pack(">H", len(self.GroundSampleDistances) ))
        for x in self.GroundSampleDistances:
            buffer.extend(struct.pack(">f", x ))
        buffer.extend(struct.pack(">H", len(self.AglAltitudes) ))
        for x in self.AglAltitudes:
            buffer.extend(struct.pack(">f", x ))
        buffer.extend(struct.pack(">H", len(self.ElevationAngles) ))
        for x in self.ElevationAngles:
            buffer.extend(struct.pack(">f", x ))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = LMCPObject.LMCPObject.unpack(self, buffer, _pos)
        self.FootprintRequestID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.VehicleID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.EligibleWavelengths = [None] * _arraylen
        if _arraylen > 0:
            self.EligibleWavelengths = struct.unpack_from(">" + repr(_arraylen) + "i", buffer, _pos )
            _pos += 4 * _arraylen
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.GroundSampleDistances = [None] * _arraylen
        if _arraylen > 0:
            self.GroundSampleDistances = struct.unpack_from(">" + repr(_arraylen) + "f", buffer, _pos )
            _pos += 4 * _arraylen
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.AglAltitudes = [None] * _arraylen
        if _arraylen > 0:
            self.AglAltitudes = struct.unpack_from(">" + repr(_arraylen) + "f", buffer, _pos )
            _pos += 4 * _arraylen
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.ElevationAngles = [None] * _arraylen
        if _arraylen > 0:
            self.ElevationAngles = struct.unpack_from(">" + repr(_arraylen) + "f", buffer, _pos )
            _pos += 4 * _arraylen
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        LMCPObject.LMCPObject.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "FootprintRequestID" and len(e.childNodes) > 0 :
                    self.FootprintRequestID = int(e.childNodes[0].nodeValue)
                elif e.localName == "VehicleID" and len(e.childNodes) > 0 :
                    self.VehicleID = int(e.childNodes[0].nodeValue)
                elif e.localName == "EligibleWavelengths" and len(e.childNodes) > 0 :
                    self.EligibleWavelengths = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.EligibleWavelengths.append( WavelengthBand.get_WavelengthBand_str(c.childNodes[0].nodeValue) )
                elif e.localName == "GroundSampleDistances" and len(e.childNodes) > 0 :
                    self.GroundSampleDistances = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.GroundSampleDistances.append( float(c.childNodes[0].nodeValue) )
                elif e.localName == "AglAltitudes" and len(e.childNodes) > 0 :
                    self.AglAltitudes = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.AglAltitudes.append( float(c.childNodes[0].nodeValue) )
                elif e.localName == "ElevationAngles" and len(e.childNodes) > 0 :
                    self.ElevationAngles = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.ElevationAngles.append( float(c.childNodes[0].nodeValue) )

        return

    def unpackFromDict(self, d, seriesFactory):
        LMCPObject.LMCPObject.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "FootprintRequestID":
                self.FootprintRequestID = d[key]
            elif key == "VehicleID":
                self.VehicleID = d[key]
            elif key == "EligibleWavelengths":
                self.EligibleWavelengths = []
                for c in d[key]:
                    self.EligibleWavelengths.append( c )
            elif key == "GroundSampleDistances":
                self.GroundSampleDistances = []
                for c in d[key]:
                    self.GroundSampleDistances.append( c )
            elif key == "AglAltitudes":
                self.AglAltitudes = []
                for c in d[key]:
                    self.AglAltitudes.append( c )
            elif key == "ElevationAngles":
                self.ElevationAngles = []
                for c in d[key]:
                    self.ElevationAngles.append( c )

        return

    def get_FootprintRequestID(self):
        return self.FootprintRequestID

    def set_FootprintRequestID(self, value):
        self.FootprintRequestID = int( value )

    def get_VehicleID(self):
        return self.VehicleID

    def set_VehicleID(self, value):
        self.VehicleID = int( value )

    def get_EligibleWavelengths(self):
        return self.EligibleWavelengths

    def get_GroundSampleDistances(self):
        return self.GroundSampleDistances

    def get_AglAltitudes(self):
        return self.AglAltitudes

    def get_ElevationAngles(self):
        return self.ElevationAngles



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = LMCPObject.LMCPObject.toString(self)
        buf += "From FootprintRequest:\n"
        buf +=    "FootprintRequestID = " + str( self.FootprintRequestID ) + "\n" 
        buf +=    "VehicleID = " + str( self.VehicleID ) + "\n" 
        buf +=    "EligibleWavelengths = " + str( self.EligibleWavelengths ) + "\n" 
        buf +=    "GroundSampleDistances = " + str( self.GroundSampleDistances ) + "\n" 
        buf +=    "AglAltitudes = " + str( self.AglAltitudes ) + "\n" 
        buf +=    "ElevationAngles = " + str( self.ElevationAngles ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "UXTASK") or (len("UXTASK") == 0): # this should never happen
        	# Checks for "UXTASK" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/FootprintRequest")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("UXTASK" + "/FootprintRequest")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        LMCPObject.LMCPObject.toDictMembers(self, d)
        d['FootprintRequestID'] = self.FootprintRequestID
        d['VehicleID'] = self.VehicleID
        d['EligibleWavelengths'] = []
        for x in self.EligibleWavelengths:
            d['EligibleWavelengths'].append(x)
        d['GroundSampleDistances'] = []
        for x in self.GroundSampleDistances:
            d['GroundSampleDistances'].append(x)
        d['AglAltitudes'] = []
        for x in self.AglAltitudes:
            d['AglAltitudes'].append(x)
        d['ElevationAngles'] = []
        for x in self.ElevationAngles:
            d['ElevationAngles'].append(x)

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
        str = ws + '<FootprintRequest Series="UXTASK" >\n';
        #str +=LMCPObject.LMCPObject.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</FootprintRequest>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += LMCPObject.LMCPObject.toXMLMembersStr(self, ws)
        buf += ws + "<FootprintRequestID>" + str(self.FootprintRequestID) + "</FootprintRequestID>\n"
        buf += ws + "<VehicleID>" + str(self.VehicleID) + "</VehicleID>\n"
        buf += ws + "<EligibleWavelengths>\n"
        for x in self.EligibleWavelengths:
            buf += ws + "<WavelengthBand>" + WavelengthBand.get_WavelengthBand_int(x) + "</WavelengthBand>\n"
        buf += ws + "</EligibleWavelengths>\n"
        buf += ws + "<GroundSampleDistances>\n"
        for x in self.GroundSampleDistances:
            buf += ws + "<real32>" + str(x) + "</real32>\n"
        buf += ws + "</GroundSampleDistances>\n"
        buf += ws + "<AglAltitudes>\n"
        for x in self.AglAltitudes:
            buf += ws + "<real32>" + str(x) + "</real32>\n"
        buf += ws + "</AglAltitudes>\n"
        buf += ws + "<ElevationAngles>\n"
        for x in self.ElevationAngles:
            buf += ws + "<real32>" + str(x) + "</real32>\n"
        buf += ws + "</ElevationAngles>\n"

        return buf
        
