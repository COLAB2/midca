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

from midca.domains.tsp.lmcp.py.afrl.cmasi import PayloadAction
from midca.domains.tsp.lmcp.py.afrl.cmasi import Location3D


class GimbalStareAction(PayloadAction.PayloadAction):

    def __init__(self):
        PayloadAction.PayloadAction.__init__(self)
        self.LMCP_TYPE = 26
        self.SERIES_NAME = "CMASI"
        self.FULL_LMCP_TYPE_NAME = "afrl.cmasi.GimbalStareAction"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 4849604199710720000
        self.SERIES_VERSION = 3

        #Define message fields
        self.Starepoint = Location3D.Location3D()   #Location3D
        self.Duration = 0   #int64


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(PayloadAction.PayloadAction.pack(self))
        buffer.extend(struct.pack("B", self.Starepoint != None ))
        if self.Starepoint != None:
            buffer.extend(struct.pack(">q", self.Starepoint.SERIES_NAME_ID))
            buffer.extend(struct.pack(">I", self.Starepoint.LMCP_TYPE))
            buffer.extend(struct.pack(">H", self.Starepoint.SERIES_VERSION))
            buffer.extend(self.Starepoint.pack())
        buffer.extend(struct.pack(">q", self.Duration))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = PayloadAction.PayloadAction.unpack(self, buffer, _pos)
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
            self.Starepoint = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
            _pos = self.Starepoint.unpack(buffer, _pos)
        else:
            self.Starepoint = None
        self.Duration = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        PayloadAction.PayloadAction.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "Starepoint" and len(e.childNodes) > 0 :
                    for n in e.childNodes:
                        if n.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.Starepoint = seriesFactory.createObjectByName(n.getAttribute('Series'), n.localName)
                            if self.Starepoint != None:
                                self.Starepoint.unpackFromXMLNode(n, seriesFactory)
                elif e.localName == "Duration" and len(e.childNodes) > 0 :
                    self.Duration = int(e.childNodes[0].nodeValue)

        return

    def unpackFromDict(self, d, seriesFactory):
        PayloadAction.PayloadAction.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "Starepoint":
                self.Starepoint = seriesFactory.unpackFromDict(d[key])
            elif key == "Duration":
                self.Duration = d[key]

        return

    def get_Starepoint(self):
        return self.Starepoint

    def set_Starepoint(self, value):
        self.Starepoint = value 

    def get_Duration(self):
        return self.Duration

    def set_Duration(self, value):
        self.Duration = int( value )



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = PayloadAction.PayloadAction.toString(self)
        buf += "From GimbalStareAction:\n"
        buf +=    "Starepoint = " + str( self.Starepoint ) + "\n" 
        buf +=    "Duration = " + str( self.Duration ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "CMASI") or (len("CMASI") == 0): # this should never happen
        	# Checks for "CMASI" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/GimbalStareAction")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("CMASI" + "/GimbalStareAction")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        PayloadAction.PayloadAction.toDictMembers(self, d)
        if self.Starepoint == None:
            d['Starepoint'] = None
        else:
            d['Starepoint'] = self.Starepoint.toDict()
        d['Duration'] = self.Duration

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
        str = ws + '<GimbalStareAction Series="CMASI" >\n';
        #str +=PayloadAction.PayloadAction.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</GimbalStareAction>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += PayloadAction.PayloadAction.toXMLMembersStr(self, ws)
        if self.Starepoint != None:
            buf += ws + "<Starepoint>\n"
            buf += ws + self.Starepoint.toXMLStr(ws + "    ") 
            buf += ws + "</Starepoint>\n"
        buf += ws + "<Duration>" + str(self.Duration) + "</Duration>\n"

        return buf
        
