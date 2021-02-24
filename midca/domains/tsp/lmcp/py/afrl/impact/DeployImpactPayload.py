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

from midca.domains.tsp.lmcp.py.afrl.cmasi import VehicleAction
from midca.domains.tsp.lmcp.py.afrl.impact import ImpactPayloadType


class DeployImpactPayload(VehicleAction.VehicleAction):

    def __init__(self):
        VehicleAction.VehicleAction.__init__(self)
        self.LMCP_TYPE = 7
        self.SERIES_NAME = "IMPACT"
        self.FULL_LMCP_TYPE_NAME = "afrl.impact.DeployImpactPayload"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 5281966179208134656
        self.SERIES_VERSION = 14

        #Define message fields
        self.VehicleID = 0   #int64
        self.DeployedPayload = ImpactPayloadType.ImpactPayloadType.Unknown   #ImpactPayloadType
        self.TargetEntityID = 0   #int64


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(VehicleAction.VehicleAction.pack(self))
        buffer.extend(struct.pack(">q", self.VehicleID))
        buffer.extend(struct.pack(">i", self.DeployedPayload))
        buffer.extend(struct.pack(">q", self.TargetEntityID))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = VehicleAction.VehicleAction.unpack(self, buffer, _pos)
        self.VehicleID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.DeployedPayload = struct.unpack_from(">i", buffer, _pos)[0]
        _pos += 4
        self.TargetEntityID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        VehicleAction.VehicleAction.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "VehicleID" and len(e.childNodes) > 0 :
                    self.VehicleID = int(e.childNodes[0].nodeValue)
                elif e.localName == "DeployedPayload" and len(e.childNodes) > 0 :
                    self.DeployedPayload = ImpactPayloadType.get_ImpactPayloadType_str(e.childNodes[0].nodeValue)
                elif e.localName == "TargetEntityID" and len(e.childNodes) > 0 :
                    self.TargetEntityID = int(e.childNodes[0].nodeValue)

        return

    def unpackFromDict(self, d, seriesFactory):
        VehicleAction.VehicleAction.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "VehicleID":
                self.VehicleID = d[key]
            elif key == "DeployedPayload":
                self.DeployedPayload = d[key]
            elif key == "TargetEntityID":
                self.TargetEntityID = d[key]

        return

    def get_VehicleID(self):
        return self.VehicleID

    def set_VehicleID(self, value):
        self.VehicleID = int( value )

    def get_DeployedPayload(self):
        return self.DeployedPayload

    def set_DeployedPayload(self, value):
        self.DeployedPayload = value 

    def get_TargetEntityID(self):
        return self.TargetEntityID

    def set_TargetEntityID(self, value):
        self.TargetEntityID = int( value )



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = VehicleAction.VehicleAction.toString(self)
        buf += "From DeployImpactPayload:\n"
        buf +=    "VehicleID = " + str( self.VehicleID ) + "\n" 
        buf +=    "DeployedPayload = " + str( self.DeployedPayload ) + "\n" 
        buf +=    "TargetEntityID = " + str( self.TargetEntityID ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "IMPACT") or (len("IMPACT") == 0): # this should never happen
        	# Checks for "IMPACT" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/DeployImpactPayload")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("IMPACT" + "/DeployImpactPayload")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        VehicleAction.VehicleAction.toDictMembers(self, d)
        d['VehicleID'] = self.VehicleID
        d['DeployedPayload'] = self.DeployedPayload
        d['TargetEntityID'] = self.TargetEntityID

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
        str = ws + '<DeployImpactPayload Series="IMPACT" >\n';
        #str +=VehicleAction.VehicleAction.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</DeployImpactPayload>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += VehicleAction.VehicleAction.toXMLMembersStr(self, ws)
        buf += ws + "<VehicleID>" + str(self.VehicleID) + "</VehicleID>\n"
        buf += ws + "<DeployedPayload>" + ImpactPayloadType.get_ImpactPayloadType_int(self.DeployedPayload) + "</DeployedPayload>\n"
        buf += ws + "<TargetEntityID>" + str(self.TargetEntityID) + "</TargetEntityID>\n"

        return buf
        
