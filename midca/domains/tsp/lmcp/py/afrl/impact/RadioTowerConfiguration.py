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
from midca.domains.tsp.lmcp.py.afrl.cmasi import Location3D


class RadioTowerConfiguration(EntityConfiguration.EntityConfiguration):

    def __init__(self):
        EntityConfiguration.EntityConfiguration.__init__(self)
        self.LMCP_TYPE = 3
        self.SERIES_NAME = "IMPACT"
        self.FULL_LMCP_TYPE_NAME = "afrl.impact.RadioTowerConfiguration"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 5281966179208134656
        self.SERIES_VERSION = 14

        #Define message fields
        self.Position = Location3D.Location3D()   #Location3D
        self.Range = 1500.0   #real32
        self.Enabled = True   #bool


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(EntityConfiguration.EntityConfiguration.pack(self))
        buffer.extend(struct.pack("B", self.Position != None ))
        if self.Position != None:
            buffer.extend(struct.pack(">q", self.Position.SERIES_NAME_ID))
            buffer.extend(struct.pack(">I", self.Position.LMCP_TYPE))
            buffer.extend(struct.pack(">H", self.Position.SERIES_VERSION))
            buffer.extend(self.Position.pack())
        buffer.extend(struct.pack(">f", self.Range))
        boolChar = 1 if self.Enabled == True else 0
        buffer.extend(struct.pack(">B",boolChar))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = EntityConfiguration.EntityConfiguration.unpack(self, buffer, _pos)
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
            self.Position = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
            _pos = self.Position.unpack(buffer, _pos)
        else:
            self.Position = None
        self.Range = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        boolChar = struct.unpack_from(">B", buffer, _pos)[0]
        self.Enabled = True if boolChar == 1 else False
        _pos += 1
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        EntityConfiguration.EntityConfiguration.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "Position" and len(e.childNodes) > 0 :
                    for n in e.childNodes:
                        if n.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.Position = seriesFactory.createObjectByName(n.getAttribute('Series'), n.localName)
                            if self.Position != None:
                                self.Position.unpackFromXMLNode(n, seriesFactory)
                elif e.localName == "Range" and len(e.childNodes) > 0 :
                    self.Range = float(e.childNodes[0].nodeValue)
                elif e.localName == "Enabled" and len(e.childNodes) > 0 :
                    self.Enabled = e.childNodes[0].nodeValue.lower() == 'true'

        return

    def unpackFromDict(self, d, seriesFactory):
        EntityConfiguration.EntityConfiguration.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "Position":
                self.Position = seriesFactory.unpackFromDict(d[key])
            elif key == "Range":
                self.Range = d[key]
            elif key == "Enabled":
                self.Enabled = d[key]

        return

    def get_Position(self):
        return self.Position

    def set_Position(self, value):
        self.Position = value 

    def get_Range(self):
        return self.Range

    def set_Range(self, value):
        self.Range = float( value )

    def get_Enabled(self):
        return self.Enabled

    def set_Enabled(self, value):
        self.Enabled = bool( value )



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = EntityConfiguration.EntityConfiguration.toString(self)
        buf += "From RadioTowerConfiguration:\n"
        buf +=    "Position = " + str( self.Position ) + "\n" 
        buf +=    "Range = " + str( self.Range ) + "\n" 
        buf +=    "Enabled = " + str( self.Enabled ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "IMPACT") or (len("IMPACT") == 0): # this should never happen
        	# Checks for "IMPACT" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/RadioTowerConfiguration")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("IMPACT" + "/RadioTowerConfiguration")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        EntityConfiguration.EntityConfiguration.toDictMembers(self, d)
        if self.Position == None:
            d['Position'] = None
        else:
            d['Position'] = self.Position.toDict()
        d['Range'] = self.Range
        d['Enabled'] = self.Enabled

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
        str = ws + '<RadioTowerConfiguration Series="IMPACT" >\n';
        #str +=EntityConfiguration.EntityConfiguration.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</RadioTowerConfiguration>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += EntityConfiguration.EntityConfiguration.toXMLMembersStr(self, ws)
        if self.Position != None:
            buf += ws + "<Position>\n"
            buf += ws + self.Position.toXMLStr(ws + "    ") 
            buf += ws + "</Position>\n"
        buf += ws + "<Range>" + str(self.Range) + "</Range>\n"
        buf += ws + "<Enabled>" + ('True' if self.Enabled else 'False') + "</Enabled>\n"

        return buf
        
