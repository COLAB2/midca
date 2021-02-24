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


class SurfaceVehicleState(EntityState.EntityState):

    def __init__(self):
        EntityState.EntityState.__init__(self)
        self.LMCP_TYPE = 4
        self.SERIES_NAME = "VEHICLES"
        self.FULL_LMCP_TYPE_NAME = "afrl.vehicles.SurfaceVehicleState"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 6216454340153722195
        self.SERIES_VERSION = 1

        #Define message fields
        self.BankAngle = 0   #real32
        self.Speed = 0   #real32


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(EntityState.EntityState.pack(self))
        buffer.extend(struct.pack(">f", self.BankAngle))
        buffer.extend(struct.pack(">f", self.Speed))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = EntityState.EntityState.unpack(self, buffer, _pos)
        self.BankAngle = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.Speed = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        EntityState.EntityState.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "BankAngle" and len(e.childNodes) > 0 :
                    self.BankAngle = float(e.childNodes[0].nodeValue)
                elif e.localName == "Speed" and len(e.childNodes) > 0 :
                    self.Speed = float(e.childNodes[0].nodeValue)

        return

    def unpackFromDict(self, d, seriesFactory):
        EntityState.EntityState.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "BankAngle":
                self.BankAngle = d[key]
            elif key == "Speed":
                self.Speed = d[key]

        return

    def get_BankAngle(self):
        return self.BankAngle

    def set_BankAngle(self, value):
        self.BankAngle = float( value )

    def get_Speed(self):
        return self.Speed

    def set_Speed(self, value):
        self.Speed = float( value )



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = EntityState.EntityState.toString(self)
        buf += "From SurfaceVehicleState:\n"
        buf +=    "BankAngle = " + str( self.BankAngle ) + "\n" 
        buf +=    "Speed = " + str( self.Speed ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "VEHICLES") or (len("VEHICLES") == 0): # this should never happen
        	# Checks for "VEHICLES" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/SurfaceVehicleState")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("VEHICLES" + "/SurfaceVehicleState")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        EntityState.EntityState.toDictMembers(self, d)
        d['BankAngle'] = self.BankAngle
        d['Speed'] = self.Speed

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
        str = ws + '<SurfaceVehicleState Series="VEHICLES" >\n';
        #str +=EntityState.EntityState.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</SurfaceVehicleState>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += EntityState.EntityState.toXMLMembersStr(self, ws)
        buf += ws + "<BankAngle>" + str(self.BankAngle) + "</BankAngle>\n"
        buf += ws + "<Speed>" + str(self.Speed) + "</Speed>\n"

        return buf
        
