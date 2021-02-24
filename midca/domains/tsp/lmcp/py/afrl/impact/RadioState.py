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


class RadioState(PayloadState.PayloadState):

    def __init__(self):
        PayloadState.PayloadState.__init__(self)
        self.LMCP_TYPE = 4
        self.SERIES_NAME = "IMPACT"
        self.FULL_LMCP_TYPE_NAME = "afrl.impact.RadioState"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 5281966179208134656
        self.SERIES_VERSION = 14

        #Define message fields
        self.Enabled = True   #bool
        self.InRange = bool(0)   #bool


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(PayloadState.PayloadState.pack(self))
        boolChar = 1 if self.Enabled == True else 0
        buffer.extend(struct.pack(">B",boolChar))
        boolChar = 1 if self.InRange == True else 0
        buffer.extend(struct.pack(">B",boolChar))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = PayloadState.PayloadState.unpack(self, buffer, _pos)
        boolChar = struct.unpack_from(">B", buffer, _pos)[0]
        self.Enabled = True if boolChar == 1 else False
        _pos += 1
        boolChar = struct.unpack_from(">B", buffer, _pos)[0]
        self.InRange = True if boolChar == 1 else False
        _pos += 1
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        PayloadState.PayloadState.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "Enabled" and len(e.childNodes) > 0 :
                    self.Enabled = e.childNodes[0].nodeValue.lower() == 'true'
                elif e.localName == "InRange" and len(e.childNodes) > 0 :
                    self.InRange = e.childNodes[0].nodeValue.lower() == 'true'

        return

    def unpackFromDict(self, d, seriesFactory):
        PayloadState.PayloadState.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "Enabled":
                self.Enabled = d[key]
            elif key == "InRange":
                self.InRange = d[key]

        return

    def get_Enabled(self):
        return self.Enabled

    def set_Enabled(self, value):
        self.Enabled = bool( value )

    def get_InRange(self):
        return self.InRange

    def set_InRange(self, value):
        self.InRange = bool( value )



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = PayloadState.PayloadState.toString(self)
        buf += "From RadioState:\n"
        buf +=    "Enabled = " + str( self.Enabled ) + "\n" 
        buf +=    "InRange = " + str( self.InRange ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "IMPACT") or (len("IMPACT") == 0): # this should never happen
        	# Checks for "IMPACT" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/RadioState")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("IMPACT" + "/RadioState")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        PayloadState.PayloadState.toDictMembers(self, d)
        d['Enabled'] = self.Enabled
        d['InRange'] = self.InRange

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
        str = ws + '<RadioState Series="IMPACT" >\n';
        #str +=PayloadState.PayloadState.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</RadioState>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += PayloadState.PayloadState.toXMLMembersStr(self, ws)
        buf += ws + "<Enabled>" + ('True' if self.Enabled else 'False') + "</Enabled>\n"
        buf += ws + "<InRange>" + ('True' if self.InRange else 'False') + "</InRange>\n"

        return buf
        
