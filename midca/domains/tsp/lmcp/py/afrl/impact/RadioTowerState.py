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


class RadioTowerState(EntityState.EntityState):

    def __init__(self):
        EntityState.EntityState.__init__(self)
        self.LMCP_TYPE = 5
        self.SERIES_NAME = "IMPACT"
        self.FULL_LMCP_TYPE_NAME = "afrl.impact.RadioTowerState"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 5281966179208134656
        self.SERIES_VERSION = 14

        #Define message fields
        self.Enabled = True   #bool


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(EntityState.EntityState.pack(self))
        boolChar = 1 if self.Enabled == True else 0
        buffer.extend(struct.pack(">B",boolChar))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = EntityState.EntityState.unpack(self, buffer, _pos)
        boolChar = struct.unpack_from(">B", buffer, _pos)[0]
        self.Enabled = True if boolChar == 1 else False
        _pos += 1
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        EntityState.EntityState.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "Enabled" and len(e.childNodes) > 0 :
                    self.Enabled = e.childNodes[0].nodeValue.lower() == 'true'

        return

    def unpackFromDict(self, d, seriesFactory):
        EntityState.EntityState.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "Enabled":
                self.Enabled = d[key]

        return

    def get_Enabled(self):
        return self.Enabled

    def set_Enabled(self, value):
        self.Enabled = bool( value )



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = EntityState.EntityState.toString(self)
        buf += "From RadioTowerState:\n"
        buf +=    "Enabled = " + str( self.Enabled ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "IMPACT") or (len("IMPACT") == 0): # this should never happen
        	# Checks for "IMPACT" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/RadioTowerState")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("IMPACT" + "/RadioTowerState")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        EntityState.EntityState.toDictMembers(self, d)
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
        str = ws + '<RadioTowerState Series="IMPACT" >\n';
        #str +=EntityState.EntityState.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</RadioTowerState>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += EntityState.EntityState.toXMLMembersStr(self, ws)
        buf += ws + "<Enabled>" + ('True' if self.Enabled else 'False') + "</Enabled>\n"

        return buf
        
