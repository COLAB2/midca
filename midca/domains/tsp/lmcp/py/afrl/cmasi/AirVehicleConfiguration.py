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

from midca.domains.tsp.lmcp.py.afrl.cmasi import EntityConfiguration
from midca.domains.tsp.lmcp.py.afrl.cmasi import FlightProfile
from midca.domains.tsp.lmcp.py.afrl.cmasi import LoiterType
from midca.domains.tsp.lmcp.py.afrl.cmasi import TurnType
from midca.domains.tsp.lmcp.py.afrl.cmasi import AltitudeType


class AirVehicleConfiguration(EntityConfiguration.EntityConfiguration):

    def __init__(self):
        EntityConfiguration.EntityConfiguration.__init__(self)
        self.LMCP_TYPE = 13
        self.SERIES_NAME = "CMASI"
        self.FULL_LMCP_TYPE_NAME = "afrl.cmasi.AirVehicleConfiguration"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 4849604199710720000
        self.SERIES_VERSION = 3

        #Define message fields
        self.MinimumSpeed = 0   #real32
        self.MaximumSpeed = 0   #real32
        self.NominalFlightProfile = FlightProfile.FlightProfile()   #FlightProfile
        self.AlternateFlightProfiles = []   #FlightProfile
        self.AvailableLoiterTypes = []   #LoiterType
        self.AvailableTurnTypes = []   #TurnType
        self.MinimumAltitude = 0   #real32
        self.MinAltitudeType = AltitudeType.AltitudeType.AGL   #AltitudeType
        self.MaximumAltitude = 1000000   #real32
        self.MaxAltitudeType = AltitudeType.AltitudeType.MSL   #AltitudeType


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(EntityConfiguration.EntityConfiguration.pack(self))
        buffer.extend(struct.pack(">f", self.MinimumSpeed))
        buffer.extend(struct.pack(">f", self.MaximumSpeed))
        buffer.extend(struct.pack("B", self.NominalFlightProfile != None ))
        if self.NominalFlightProfile != None:
            buffer.extend(struct.pack(">q", self.NominalFlightProfile.SERIES_NAME_ID))
            buffer.extend(struct.pack(">I", self.NominalFlightProfile.LMCP_TYPE))
            buffer.extend(struct.pack(">H", self.NominalFlightProfile.SERIES_VERSION))
            buffer.extend(self.NominalFlightProfile.pack())
        buffer.extend(struct.pack(">H", len(self.AlternateFlightProfiles) ))
        for x in self.AlternateFlightProfiles:
           buffer.extend(struct.pack("B", x != None ))
           if x != None:
               buffer.extend(struct.pack(">q", x.SERIES_NAME_ID))
               buffer.extend(struct.pack(">I", x.LMCP_TYPE))
               buffer.extend(struct.pack(">H", x.SERIES_VERSION))
               buffer.extend(x.pack())
        buffer.extend(struct.pack(">H", len(self.AvailableLoiterTypes) ))
        for x in self.AvailableLoiterTypes:
            buffer.extend(struct.pack(">i", x ))
        buffer.extend(struct.pack(">H", len(self.AvailableTurnTypes) ))
        for x in self.AvailableTurnTypes:
            buffer.extend(struct.pack(">i", x ))
        buffer.extend(struct.pack(">f", self.MinimumAltitude))
        buffer.extend(struct.pack(">i", self.MinAltitudeType))
        buffer.extend(struct.pack(">f", self.MaximumAltitude))
        buffer.extend(struct.pack(">i", self.MaxAltitudeType))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = EntityConfiguration.EntityConfiguration.unpack(self, buffer, _pos)
        self.MinimumSpeed = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.MaximumSpeed = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
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
            self.NominalFlightProfile = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
            _pos = self.NominalFlightProfile.unpack(buffer, _pos)
        else:
            self.NominalFlightProfile = None
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.AlternateFlightProfiles = [None] * _arraylen
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
                self.AlternateFlightProfiles[x] = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
                _pos = self.AlternateFlightProfiles[x].unpack(buffer, _pos)
            else:
                self.AlternateFlightProfiles[x] = None
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.AvailableLoiterTypes = [None] * _arraylen
        if _arraylen > 0:
            self.AvailableLoiterTypes = struct.unpack_from(">" + repr(_arraylen) + "i", buffer, _pos )
            _pos += 4 * _arraylen
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.AvailableTurnTypes = [None] * _arraylen
        if _arraylen > 0:
            self.AvailableTurnTypes = struct.unpack_from(">" + repr(_arraylen) + "i", buffer, _pos )
            _pos += 4 * _arraylen
        self.MinimumAltitude = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.MinAltitudeType = struct.unpack_from(">i", buffer, _pos)[0]
        _pos += 4
        self.MaximumAltitude = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.MaxAltitudeType = struct.unpack_from(">i", buffer, _pos)[0]
        _pos += 4
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        EntityConfiguration.EntityConfiguration.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "MinimumSpeed" and len(e.childNodes) > 0 :
                    self.MinimumSpeed = float(e.childNodes[0].nodeValue)
                elif e.localName == "MaximumSpeed" and len(e.childNodes) > 0 :
                    self.MaximumSpeed = float(e.childNodes[0].nodeValue)
                elif e.localName == "NominalFlightProfile" and len(e.childNodes) > 0 :
                    for n in e.childNodes:
                        if n.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.NominalFlightProfile = seriesFactory.createObjectByName(n.getAttribute('Series'), n.localName)
                            if self.NominalFlightProfile != None:
                                self.NominalFlightProfile.unpackFromXMLNode(n, seriesFactory)
                elif e.localName == "AlternateFlightProfiles" and len(e.childNodes) > 0 :
                    self.AlternateFlightProfiles = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            obj = seriesFactory.createObjectByName(c.getAttribute('Series'), c.localName)
                            if obj != None:
                                obj.unpackFromXMLNode(c, seriesFactory)
                                self.AlternateFlightProfiles.append(obj)
                elif e.localName == "AvailableLoiterTypes" and len(e.childNodes) > 0 :
                    self.AvailableLoiterTypes = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.AvailableLoiterTypes.append( LoiterType.get_LoiterType_str(c.childNodes[0].nodeValue) )
                elif e.localName == "AvailableTurnTypes" and len(e.childNodes) > 0 :
                    self.AvailableTurnTypes = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.AvailableTurnTypes.append( TurnType.get_TurnType_str(c.childNodes[0].nodeValue) )
                elif e.localName == "MinimumAltitude" and len(e.childNodes) > 0 :
                    self.MinimumAltitude = float(e.childNodes[0].nodeValue)
                elif e.localName == "MinAltitudeType" and len(e.childNodes) > 0 :
                    self.MinAltitudeType = AltitudeType.get_AltitudeType_str(e.childNodes[0].nodeValue)
                elif e.localName == "MaximumAltitude" and len(e.childNodes) > 0 :
                    self.MaximumAltitude = float(e.childNodes[0].nodeValue)
                elif e.localName == "MaxAltitudeType" and len(e.childNodes) > 0 :
                    self.MaxAltitudeType = AltitudeType.get_AltitudeType_str(e.childNodes[0].nodeValue)

        return

    def unpackFromDict(self, d, seriesFactory):
        EntityConfiguration.EntityConfiguration.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "MinimumSpeed":
                self.MinimumSpeed = d[key]
            elif key == "MaximumSpeed":
                self.MaximumSpeed = d[key]
            elif key == "NominalFlightProfile":
                self.NominalFlightProfile = seriesFactory.unpackFromDict(d[key])
            elif key == "AlternateFlightProfiles":
                self.AlternateFlightProfiles = []
                for c in d[key]:
                    obj = seriesFactory.unpackFromDict(c)
                    if obj != None:
                        self.AlternateFlightProfiles.append(obj)
            elif key == "AvailableLoiterTypes":
                self.AvailableLoiterTypes = []
                for c in d[key]:
                    self.AvailableLoiterTypes.append( c )
            elif key == "AvailableTurnTypes":
                self.AvailableTurnTypes = []
                for c in d[key]:
                    self.AvailableTurnTypes.append( c )
            elif key == "MinimumAltitude":
                self.MinimumAltitude = d[key]
            elif key == "MinAltitudeType":
                self.MinAltitudeType = d[key]
            elif key == "MaximumAltitude":
                self.MaximumAltitude = d[key]
            elif key == "MaxAltitudeType":
                self.MaxAltitudeType = d[key]

        return

    def get_MinimumSpeed(self):
        return self.MinimumSpeed

    def set_MinimumSpeed(self, value):
        self.MinimumSpeed = float( value )

    def get_MaximumSpeed(self):
        return self.MaximumSpeed

    def set_MaximumSpeed(self, value):
        self.MaximumSpeed = float( value )

    def get_NominalFlightProfile(self):
        return self.NominalFlightProfile

    def set_NominalFlightProfile(self, value):
        self.NominalFlightProfile = value 

    def get_AlternateFlightProfiles(self):
        return self.AlternateFlightProfiles

    def get_AvailableLoiterTypes(self):
        return self.AvailableLoiterTypes

    def get_AvailableTurnTypes(self):
        return self.AvailableTurnTypes

    def get_MinimumAltitude(self):
        return self.MinimumAltitude

    def set_MinimumAltitude(self, value):
        self.MinimumAltitude = float( value )

    def get_MinAltitudeType(self):
        return self.MinAltitudeType

    def set_MinAltitudeType(self, value):
        self.MinAltitudeType = value 

    def get_MaximumAltitude(self):
        return self.MaximumAltitude

    def set_MaximumAltitude(self, value):
        self.MaximumAltitude = float( value )

    def get_MaxAltitudeType(self):
        return self.MaxAltitudeType

    def set_MaxAltitudeType(self, value):
        self.MaxAltitudeType = value 



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = EntityConfiguration.EntityConfiguration.toString(self)
        buf += "From AirVehicleConfiguration:\n"
        buf +=    "MinimumSpeed = " + str( self.MinimumSpeed ) + "\n" 
        buf +=    "MaximumSpeed = " + str( self.MaximumSpeed ) + "\n" 
        buf +=    "NominalFlightProfile = " + str( self.NominalFlightProfile ) + "\n" 
        buf +=    "AlternateFlightProfiles = " + str( self.AlternateFlightProfiles ) + "\n" 
        buf +=    "AvailableLoiterTypes = " + str( self.AvailableLoiterTypes ) + "\n" 
        buf +=    "AvailableTurnTypes = " + str( self.AvailableTurnTypes ) + "\n" 
        buf +=    "MinimumAltitude = " + str( self.MinimumAltitude ) + "\n" 
        buf +=    "MinAltitudeType = " + str( self.MinAltitudeType ) + "\n" 
        buf +=    "MaximumAltitude = " + str( self.MaximumAltitude ) + "\n" 
        buf +=    "MaxAltitudeType = " + str( self.MaxAltitudeType ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "CMASI") or (len("CMASI") == 0): # this should never happen
        	# Checks for "CMASI" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/AirVehicleConfiguration")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("CMASI" + "/AirVehicleConfiguration")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        EntityConfiguration.EntityConfiguration.toDictMembers(self, d)
        d['MinimumSpeed'] = self.MinimumSpeed
        d['MaximumSpeed'] = self.MaximumSpeed
        if self.NominalFlightProfile == None:
            d['NominalFlightProfile'] = None
        else:
            d['NominalFlightProfile'] = self.NominalFlightProfile.toDict()
        d['AlternateFlightProfiles'] = []
        for x in self.AlternateFlightProfiles:
            if x == None:
                d['AlternateFlightProfiles'].append(None)
            else:
                d['AlternateFlightProfiles'].append(x.toDict())
        d['AvailableLoiterTypes'] = []
        for x in self.AvailableLoiterTypes:
            d['AvailableLoiterTypes'].append(x)
        d['AvailableTurnTypes'] = []
        for x in self.AvailableTurnTypes:
            d['AvailableTurnTypes'].append(x)
        d['MinimumAltitude'] = self.MinimumAltitude
        d['MinAltitudeType'] = self.MinAltitudeType
        d['MaximumAltitude'] = self.MaximumAltitude
        d['MaxAltitudeType'] = self.MaxAltitudeType

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
        str = ws + '<AirVehicleConfiguration Series="CMASI" >\n';
        #str +=EntityConfiguration.EntityConfiguration.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</AirVehicleConfiguration>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += EntityConfiguration.EntityConfiguration.toXMLMembersStr(self, ws)
        buf += ws + "<MinimumSpeed>" + str(self.MinimumSpeed) + "</MinimumSpeed>\n"
        buf += ws + "<MaximumSpeed>" + str(self.MaximumSpeed) + "</MaximumSpeed>\n"
        if self.NominalFlightProfile != None:
            buf += ws + "<NominalFlightProfile>\n"
            buf += ws + self.NominalFlightProfile.toXMLStr(ws + "    ") 
            buf += ws + "</NominalFlightProfile>\n"
        buf += ws + "<AlternateFlightProfiles>\n"
        for x in self.AlternateFlightProfiles:
            if x == None:
                buf += ws + "    <null/>\n"
            else:
                buf += x.toXMLStr(ws + "    ") 
        buf += ws + "</AlternateFlightProfiles>\n"
        buf += ws + "<AvailableLoiterTypes>\n"
        for x in self.AvailableLoiterTypes:
            buf += ws + "<LoiterType>" + LoiterType.get_LoiterType_int(x) + "</LoiterType>\n"
        buf += ws + "</AvailableLoiterTypes>\n"
        buf += ws + "<AvailableTurnTypes>\n"
        for x in self.AvailableTurnTypes:
            buf += ws + "<TurnType>" + TurnType.get_TurnType_int(x) + "</TurnType>\n"
        buf += ws + "</AvailableTurnTypes>\n"
        buf += ws + "<MinimumAltitude>" + str(self.MinimumAltitude) + "</MinimumAltitude>\n"
        buf += ws + "<MinAltitudeType>" + AltitudeType.get_AltitudeType_int(self.MinAltitudeType) + "</MinAltitudeType>\n"
        buf += ws + "<MaximumAltitude>" + str(self.MaximumAltitude) + "</MaximumAltitude>\n"
        buf += ws + "<MaxAltitudeType>" + AltitudeType.get_AltitudeType_int(self.MaxAltitudeType) + "</MaxAltitudeType>\n"

        return buf
        
