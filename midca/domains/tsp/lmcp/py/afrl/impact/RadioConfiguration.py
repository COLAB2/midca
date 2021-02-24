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
from midca.domains.tsp.lmcp.py.afrl.cmasi import Location3D


class RadioConfiguration(PayloadConfiguration.PayloadConfiguration):

    def __init__(self):
        PayloadConfiguration.PayloadConfiguration.__init__(self)
        self.LMCP_TYPE = 2
        self.SERIES_NAME = "IMPACT"
        self.FULL_LMCP_TYPE_NAME = "afrl.impact.RadioConfiguration"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 5281966179208134656
        self.SERIES_VERSION = 14

        #Define message fields
        self.Range = 1500.0   #real32
        self.RallyPoint = None   #Location3D
        self.Timeout = 120000   #int64


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(PayloadConfiguration.PayloadConfiguration.pack(self))
        buffer.extend(struct.pack(">f", self.Range))
        buffer.extend(struct.pack("B", self.RallyPoint != None ))
        if self.RallyPoint != None:
            buffer.extend(struct.pack(">q", self.RallyPoint.SERIES_NAME_ID))
            buffer.extend(struct.pack(">I", self.RallyPoint.LMCP_TYPE))
            buffer.extend(struct.pack(">H", self.RallyPoint.SERIES_VERSION))
            buffer.extend(self.RallyPoint.pack())
        buffer.extend(struct.pack(">q", self.Timeout))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = PayloadConfiguration.PayloadConfiguration.unpack(self, buffer, _pos)
        self.Range = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
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
            self.RallyPoint = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
            _pos = self.RallyPoint.unpack(buffer, _pos)
        else:
            self.RallyPoint = None
        self.Timeout = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        PayloadConfiguration.PayloadConfiguration.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "Range" and len(e.childNodes) > 0 :
                    self.Range = float(e.childNodes[0].nodeValue)
                elif e.localName == "RallyPoint" and len(e.childNodes) > 0 :
                    for n in e.childNodes:
                        if n.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.RallyPoint = seriesFactory.createObjectByName(n.getAttribute('Series'), n.localName)
                            if self.RallyPoint != None:
                                self.RallyPoint.unpackFromXMLNode(n, seriesFactory)
                elif e.localName == "Timeout" and len(e.childNodes) > 0 :
                    self.Timeout = int(e.childNodes[0].nodeValue)

        return

    def unpackFromDict(self, d, seriesFactory):
        PayloadConfiguration.PayloadConfiguration.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "Range":
                self.Range = d[key]
            elif key == "RallyPoint":
                self.RallyPoint = seriesFactory.unpackFromDict(d[key])
            elif key == "Timeout":
                self.Timeout = d[key]

        return

    def get_Range(self):
        return self.Range

    def set_Range(self, value):
        self.Range = float( value )

    def get_RallyPoint(self):
        return self.RallyPoint

    def set_RallyPoint(self, value):
        self.RallyPoint = value 

    def get_Timeout(self):
        return self.Timeout

    def set_Timeout(self, value):
        self.Timeout = int( value )



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = PayloadConfiguration.PayloadConfiguration.toString(self)
        buf += "From RadioConfiguration:\n"
        buf +=    "Range = " + str( self.Range ) + "\n" 
        buf +=    "RallyPoint = " + str( self.RallyPoint ) + "\n" 
        buf +=    "Timeout = " + str( self.Timeout ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "IMPACT") or (len("IMPACT") == 0): # this should never happen
        	# Checks for "IMPACT" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/RadioConfiguration")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("IMPACT" + "/RadioConfiguration")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        PayloadConfiguration.PayloadConfiguration.toDictMembers(self, d)
        d['Range'] = self.Range
        if self.RallyPoint == None:
            d['RallyPoint'] = None
        else:
            d['RallyPoint'] = self.RallyPoint.toDict()
        d['Timeout'] = self.Timeout

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
        str = ws + '<RadioConfiguration Series="IMPACT" >\n';
        #str +=PayloadConfiguration.PayloadConfiguration.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</RadioConfiguration>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += PayloadConfiguration.PayloadConfiguration.toXMLMembersStr(self, ws)
        buf += ws + "<Range>" + str(self.Range) + "</Range>\n"
        if self.RallyPoint != None:
            buf += ws + "<RallyPoint>\n"
            buf += ws + self.RallyPoint.toXMLStr(ws + "    ") 
            buf += ws + "</RallyPoint>\n"
        buf += ws + "<Timeout>" + str(self.Timeout) + "</Timeout>\n"

        return buf
        
