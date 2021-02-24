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


class GroundVehicleConfiguration(EntityConfiguration.EntityConfiguration):

    def __init__(self):
        EntityConfiguration.EntityConfiguration.__init__(self)
        self.LMCP_TYPE = 1
        self.SERIES_NAME = "VEHICLES"
        self.FULL_LMCP_TYPE_NAME = "afrl.vehicles.GroundVehicleConfiguration"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 6216454340153722195
        self.SERIES_VERSION = 1

        #Define message fields
        self.RoadGraphID = 0   #int64
        self.MinimumSpeed = 0   #real32
        self.MaximumSpeed = 0   #real32
        self.EnergyRate = 0   #real32


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(EntityConfiguration.EntityConfiguration.pack(self))
        buffer.extend(struct.pack(">q", self.RoadGraphID))
        buffer.extend(struct.pack(">f", self.MinimumSpeed))
        buffer.extend(struct.pack(">f", self.MaximumSpeed))
        buffer.extend(struct.pack(">f", self.EnergyRate))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = EntityConfiguration.EntityConfiguration.unpack(self, buffer, _pos)
        self.RoadGraphID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.MinimumSpeed = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.MaximumSpeed = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.EnergyRate = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        EntityConfiguration.EntityConfiguration.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "RoadGraphID" and len(e.childNodes) > 0 :
                    self.RoadGraphID = int(e.childNodes[0].nodeValue)
                elif e.localName == "MinimumSpeed" and len(e.childNodes) > 0 :
                    self.MinimumSpeed = float(e.childNodes[0].nodeValue)
                elif e.localName == "MaximumSpeed" and len(e.childNodes) > 0 :
                    self.MaximumSpeed = float(e.childNodes[0].nodeValue)
                elif e.localName == "EnergyRate" and len(e.childNodes) > 0 :
                    self.EnergyRate = float(e.childNodes[0].nodeValue)

        return

    def unpackFromDict(self, d, seriesFactory):
        EntityConfiguration.EntityConfiguration.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "RoadGraphID":
                self.RoadGraphID = d[key]
            elif key == "MinimumSpeed":
                self.MinimumSpeed = d[key]
            elif key == "MaximumSpeed":
                self.MaximumSpeed = d[key]
            elif key == "EnergyRate":
                self.EnergyRate = d[key]

        return

    def get_RoadGraphID(self):
        return self.RoadGraphID

    def set_RoadGraphID(self, value):
        self.RoadGraphID = int( value )

    def get_MinimumSpeed(self):
        return self.MinimumSpeed

    def set_MinimumSpeed(self, value):
        self.MinimumSpeed = float( value )

    def get_MaximumSpeed(self):
        return self.MaximumSpeed

    def set_MaximumSpeed(self, value):
        self.MaximumSpeed = float( value )

    def get_EnergyRate(self):
        return self.EnergyRate

    def set_EnergyRate(self, value):
        self.EnergyRate = float( value )



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = EntityConfiguration.EntityConfiguration.toString(self)
        buf += "From GroundVehicleConfiguration:\n"
        buf +=    "RoadGraphID = " + str( self.RoadGraphID ) + "\n" 
        buf +=    "MinimumSpeed = " + str( self.MinimumSpeed ) + "\n" 
        buf +=    "MaximumSpeed = " + str( self.MaximumSpeed ) + "\n" 
        buf +=    "EnergyRate = " + str( self.EnergyRate ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "VEHICLES") or (len("VEHICLES") == 0): # this should never happen
        	# Checks for "VEHICLES" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/GroundVehicleConfiguration")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("VEHICLES" + "/GroundVehicleConfiguration")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        EntityConfiguration.EntityConfiguration.toDictMembers(self, d)
        d['RoadGraphID'] = self.RoadGraphID
        d['MinimumSpeed'] = self.MinimumSpeed
        d['MaximumSpeed'] = self.MaximumSpeed
        d['EnergyRate'] = self.EnergyRate

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
        str = ws + '<GroundVehicleConfiguration Series="VEHICLES" >\n';
        #str +=EntityConfiguration.EntityConfiguration.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</GroundVehicleConfiguration>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += EntityConfiguration.EntityConfiguration.toXMLMembersStr(self, ws)
        buf += ws + "<RoadGraphID>" + str(self.RoadGraphID) + "</RoadGraphID>\n"
        buf += ws + "<MinimumSpeed>" + str(self.MinimumSpeed) + "</MinimumSpeed>\n"
        buf += ws + "<MaximumSpeed>" + str(self.MaximumSpeed) + "</MaximumSpeed>\n"
        buf += ws + "<EnergyRate>" + str(self.EnergyRate) + "</EnergyRate>\n"

        return buf
        
