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


class VideoStreamConfiguration(PayloadConfiguration.PayloadConfiguration):

    def __init__(self):
        PayloadConfiguration.PayloadConfiguration.__init__(self)
        self.LMCP_TYPE = 49
        self.SERIES_NAME = "CMASI"
        self.FULL_LMCP_TYPE_NAME = "afrl.cmasi.VideoStreamConfiguration"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 4849604199710720000
        self.SERIES_VERSION = 3

        #Define message fields
        self.AvailableSensorList = []   #int64


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(PayloadConfiguration.PayloadConfiguration.pack(self))
        buffer.extend(struct.pack(">H", len(self.AvailableSensorList) ))
        for x in self.AvailableSensorList:
            buffer.extend(struct.pack(">q", x ))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = PayloadConfiguration.PayloadConfiguration.unpack(self, buffer, _pos)
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.AvailableSensorList = [None] * _arraylen
        if _arraylen > 0:
            self.AvailableSensorList = struct.unpack_from(">" + repr(_arraylen) + "q", buffer, _pos )
            _pos += 8 * _arraylen
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        PayloadConfiguration.PayloadConfiguration.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "AvailableSensorList" and len(e.childNodes) > 0 :
                    self.AvailableSensorList = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.AvailableSensorList.append( int(c.childNodes[0].nodeValue) )

        return

    def unpackFromDict(self, d, seriesFactory):
        PayloadConfiguration.PayloadConfiguration.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "AvailableSensorList":
                self.AvailableSensorList = []
                for c in d[key]:
                    self.AvailableSensorList.append( c )

        return

    def get_AvailableSensorList(self):
        return self.AvailableSensorList



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = PayloadConfiguration.PayloadConfiguration.toString(self)
        buf += "From VideoStreamConfiguration:\n"
        buf +=    "AvailableSensorList = " + str( self.AvailableSensorList ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "CMASI") or (len("CMASI") == 0): # this should never happen
        	# Checks for "CMASI" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/VideoStreamConfiguration")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("CMASI" + "/VideoStreamConfiguration")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        PayloadConfiguration.PayloadConfiguration.toDictMembers(self, d)
        d['AvailableSensorList'] = []
        for x in self.AvailableSensorList:
            d['AvailableSensorList'].append(x)

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
        str = ws + '<VideoStreamConfiguration Series="CMASI" >\n';
        #str +=PayloadConfiguration.PayloadConfiguration.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</VideoStreamConfiguration>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += PayloadConfiguration.PayloadConfiguration.toXMLMembersStr(self, ws)
        buf += ws + "<AvailableSensorList>\n"
        for x in self.AvailableSensorList:
            buf += ws + "<int64>" + str(x) + "</int64>\n"
        buf += ws + "</AvailableSensorList>\n"

        return buf
        
