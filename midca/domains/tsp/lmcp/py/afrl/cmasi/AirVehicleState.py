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

from midca.domains.tsp.lmcp.py.afrl.cmasi import EntityState


class AirVehicleState(EntityState.EntityState):

    def __init__(self):
        EntityState.EntityState.__init__(self)
        self.LMCP_TYPE = 15
        self.SERIES_NAME = "CMASI"
        self.FULL_LMCP_TYPE_NAME = "afrl.cmasi.AirVehicleState"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 4849604199710720000
        self.SERIES_VERSION = 3

        #Define message fields
        self.Airspeed = 0   #real32
        self.VerticalSpeed = 0   #real32
        self.WindSpeed = 0   #real32
        self.WindDirection = 0   #real32


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(EntityState.EntityState.pack(self))
        buffer.extend(struct.pack(">f", self.Airspeed))
        buffer.extend(struct.pack(">f", self.VerticalSpeed))
        buffer.extend(struct.pack(">f", self.WindSpeed))
        buffer.extend(struct.pack(">f", self.WindDirection))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = EntityState.EntityState.unpack(self, buffer, _pos)
        self.Airspeed = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.VerticalSpeed = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.WindSpeed = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.WindDirection = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        EntityState.EntityState.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "Airspeed" and len(e.childNodes) > 0 :
                    self.Airspeed = float(e.childNodes[0].nodeValue)
                elif e.localName == "VerticalSpeed" and len(e.childNodes) > 0 :
                    self.VerticalSpeed = float(e.childNodes[0].nodeValue)
                elif e.localName == "WindSpeed" and len(e.childNodes) > 0 :
                    self.WindSpeed = float(e.childNodes[0].nodeValue)
                elif e.localName == "WindDirection" and len(e.childNodes) > 0 :
                    self.WindDirection = float(e.childNodes[0].nodeValue)

        return

    def unpackFromDict(self, d, seriesFactory):
        EntityState.EntityState.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "Airspeed":
                self.Airspeed = d[key]
            elif key == "VerticalSpeed":
                self.VerticalSpeed = d[key]
            elif key == "WindSpeed":
                self.WindSpeed = d[key]
            elif key == "WindDirection":
                self.WindDirection = d[key]

        return

    def get_Airspeed(self):
        return self.Airspeed

    def set_Airspeed(self, value):
        self.Airspeed = float( value )

    def get_VerticalSpeed(self):
        return self.VerticalSpeed

    def set_VerticalSpeed(self, value):
        self.VerticalSpeed = float( value )

    def get_WindSpeed(self):
        return self.WindSpeed

    def set_WindSpeed(self, value):
        self.WindSpeed = float( value )

    def get_WindDirection(self):
        return self.WindDirection

    def set_WindDirection(self, value):
        self.WindDirection = float( value )



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = EntityState.EntityState.toString(self)
        buf += "From AirVehicleState:\n"
        buf +=    "Airspeed = " + str( self.Airspeed ) + "\n" 
        buf +=    "VerticalSpeed = " + str( self.VerticalSpeed ) + "\n" 
        buf +=    "WindSpeed = " + str( self.WindSpeed ) + "\n" 
        buf +=    "WindDirection = " + str( self.WindDirection ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "CMASI") or (len("CMASI") == 0): # this should never happen
        	# Checks for "CMASI" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/AirVehicleState")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("CMASI" + "/AirVehicleState")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        EntityState.EntityState.toDictMembers(self, d)
        d['Airspeed'] = self.Airspeed
        d['VerticalSpeed'] = self.VerticalSpeed
        d['WindSpeed'] = self.WindSpeed
        d['WindDirection'] = self.WindDirection

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
        str = ws + '<AirVehicleState Series="CMASI" >\n';
        #str +=EntityState.EntityState.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</AirVehicleState>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += EntityState.EntityState.toXMLMembersStr(self, ws)
        buf += ws + "<Airspeed>" + str(self.Airspeed) + "</Airspeed>\n"
        buf += ws + "<VerticalSpeed>" + str(self.VerticalSpeed) + "</VerticalSpeed>\n"
        buf += ws + "<WindSpeed>" + str(self.WindSpeed) + "</WindSpeed>\n"
        buf += ws + "<WindDirection>" + str(self.WindDirection) + "</WindDirection>\n"

        return buf
        
