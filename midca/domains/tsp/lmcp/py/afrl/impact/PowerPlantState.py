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

from midca.domains.tsp.lmcp.py.afrl.cmasi import PayloadState
from midca.domains.tsp.lmcp.py.afrl.impact import PowerPlant


class PowerPlantState(PayloadState.PayloadState):

    def __init__(self):
        PayloadState.PayloadState.__init__(self)
        self.LMCP_TYPE = 8
        self.SERIES_NAME = "IMPACT"
        self.FULL_LMCP_TYPE_NAME = "afrl.impact.PowerPlantState"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 5281966179208134656
        self.SERIES_VERSION = 14

        #Define message fields
        self.ActivePowerPlant = PowerPlant.PowerPlant.Gasoline   #PowerPlant


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(PayloadState.PayloadState.pack(self))
        buffer.extend(struct.pack(">i", self.ActivePowerPlant))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = PayloadState.PayloadState.unpack(self, buffer, _pos)
        self.ActivePowerPlant = struct.unpack_from(">i", buffer, _pos)[0]
        _pos += 4
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        PayloadState.PayloadState.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "ActivePowerPlant" and len(e.childNodes) > 0 :
                    self.ActivePowerPlant = PowerPlant.get_PowerPlant_str(e.childNodes[0].nodeValue)

        return

    def unpackFromDict(self, d, seriesFactory):
        PayloadState.PayloadState.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "ActivePowerPlant":
                self.ActivePowerPlant = d[key]

        return

    def get_ActivePowerPlant(self):
        return self.ActivePowerPlant

    def set_ActivePowerPlant(self, value):
        self.ActivePowerPlant = value 



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = PayloadState.PayloadState.toString(self)
        buf += "From PowerPlantState:\n"
        buf +=    "ActivePowerPlant = " + str( self.ActivePowerPlant ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "IMPACT") or (len("IMPACT") == 0): # this should never happen
        	# Checks for "IMPACT" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/PowerPlantState")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("IMPACT" + "/PowerPlantState")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        PayloadState.PayloadState.toDictMembers(self, d)
        d['ActivePowerPlant'] = self.ActivePowerPlant

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
        str = ws + '<PowerPlantState Series="IMPACT" >\n';
        #str +=PayloadState.PayloadState.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</PowerPlantState>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += PayloadState.PayloadState.toXMLMembersStr(self, ws)
        buf += ws + "<ActivePowerPlant>" + PowerPlant.get_PowerPlant_int(self.ActivePowerPlant) + "</ActivePowerPlant>\n"

        return buf
        
