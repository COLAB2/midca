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

from midca.domains.tsp.lmcp.py.afrl.cmasi import PayloadConfiguration
from midca.domains.tsp.lmcp.py.afrl.impact import ImpactPayloadType


class ImpactPayloadConfiguration(PayloadConfiguration.PayloadConfiguration):

    def __init__(self):
        PayloadConfiguration.PayloadConfiguration.__init__(self)
        self.LMCP_TYPE = 6
        self.SERIES_NAME = "IMPACT"
        self.FULL_LMCP_TYPE_NAME = "afrl.impact.ImpactPayloadConfiguration"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 5281966179208134656
        self.SERIES_VERSION = 14

        #Define message fields
        self.AvailablePayloads = []   #ImpactPayloadType


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(PayloadConfiguration.PayloadConfiguration.pack(self))
        buffer.extend(struct.pack(">H", len(self.AvailablePayloads) ))
        for x in self.AvailablePayloads:
            buffer.extend(struct.pack(">i", x ))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = PayloadConfiguration.PayloadConfiguration.unpack(self, buffer, _pos)
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.AvailablePayloads = [None] * _arraylen
        if _arraylen > 0:
            self.AvailablePayloads = struct.unpack_from(">" + repr(_arraylen) + "i", buffer, _pos )
            _pos += 4 * _arraylen
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        PayloadConfiguration.PayloadConfiguration.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "AvailablePayloads" and len(e.childNodes) > 0 :
                    self.AvailablePayloads = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.AvailablePayloads.append( ImpactPayloadType.get_ImpactPayloadType_str(c.childNodes[0].nodeValue) )

        return

    def unpackFromDict(self, d, seriesFactory):
        PayloadConfiguration.PayloadConfiguration.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "AvailablePayloads":
                self.AvailablePayloads = []
                for c in d[key]:
                    self.AvailablePayloads.append( c )

        return

    def get_AvailablePayloads(self):
        return self.AvailablePayloads



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = PayloadConfiguration.PayloadConfiguration.toString(self)
        buf += "From ImpactPayloadConfiguration:\n"
        buf +=    "AvailablePayloads = " + str( self.AvailablePayloads ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "IMPACT") or (len("IMPACT") == 0): # this should never happen
        	# Checks for "IMPACT" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/ImpactPayloadConfiguration")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("IMPACT" + "/ImpactPayloadConfiguration")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        PayloadConfiguration.PayloadConfiguration.toDictMembers(self, d)
        d['AvailablePayloads'] = []
        for x in self.AvailablePayloads:
            d['AvailablePayloads'].append(x)

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
        str = ws + '<ImpactPayloadConfiguration Series="IMPACT" >\n';
        #str +=PayloadConfiguration.PayloadConfiguration.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</ImpactPayloadConfiguration>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += PayloadConfiguration.PayloadConfiguration.toXMLMembersStr(self, ws)
        buf += ws + "<AvailablePayloads>\n"
        for x in self.AvailablePayloads:
            buf += ws + "<ImpactPayloadType>" + ImpactPayloadType.get_ImpactPayloadType_int(x) + "</ImpactPayloadType>\n"
        buf += ws + "</AvailablePayloads>\n"

        return buf
        
