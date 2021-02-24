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

from midca.domains.tsp.lmcp.py.afrl.cmasi import Task
from midca.domains.tsp.lmcp.py.afrl.cmasi import WavelengthBand


class SearchTask(Task.Task):

    def __init__(self):
        Task.Task.__init__(self)
        self.LMCP_TYPE = 9
        self.SERIES_NAME = "CMASI"
        self.FULL_LMCP_TYPE_NAME = "afrl.cmasi.SearchTask"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 4849604199710720000
        self.SERIES_VERSION = 3

        #Define message fields
        self.DesiredWavelengthBands = []   #WavelengthBand
        self.DwellTime = 0   #int64
        self.GroundSampleDistance = 0   #real32


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(Task.Task.pack(self))
        buffer.extend(struct.pack(">H", len(self.DesiredWavelengthBands) ))
        for x in self.DesiredWavelengthBands:
            buffer.extend(struct.pack(">i", x ))
        buffer.extend(struct.pack(">q", self.DwellTime))
        buffer.extend(struct.pack(">f", self.GroundSampleDistance))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = Task.Task.unpack(self, buffer, _pos)
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.DesiredWavelengthBands = [None] * _arraylen
        if _arraylen > 0:
            self.DesiredWavelengthBands = struct.unpack_from(">" + repr(_arraylen) + "i", buffer, _pos )
            _pos += 4 * _arraylen
        self.DwellTime = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.GroundSampleDistance = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        Task.Task.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "DesiredWavelengthBands" and len(e.childNodes) > 0 :
                    self.DesiredWavelengthBands = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.DesiredWavelengthBands.append( WavelengthBand.get_WavelengthBand_str(c.childNodes[0].nodeValue) )
                elif e.localName == "DwellTime" and len(e.childNodes) > 0 :
                    self.DwellTime = int(e.childNodes[0].nodeValue)
                elif e.localName == "GroundSampleDistance" and len(e.childNodes) > 0 :
                    self.GroundSampleDistance = float(e.childNodes[0].nodeValue)

        return

    def unpackFromDict(self, d, seriesFactory):
        Task.Task.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "DesiredWavelengthBands":
                self.DesiredWavelengthBands = []
                for c in d[key]:
                    self.DesiredWavelengthBands.append( c )
            elif key == "DwellTime":
                self.DwellTime = d[key]
            elif key == "GroundSampleDistance":
                self.GroundSampleDistance = d[key]

        return

    def get_DesiredWavelengthBands(self):
        return self.DesiredWavelengthBands

    def get_DwellTime(self):
        return self.DwellTime

    def set_DwellTime(self, value):
        self.DwellTime = int( value )

    def get_GroundSampleDistance(self):
        return self.GroundSampleDistance

    def set_GroundSampleDistance(self, value):
        self.GroundSampleDistance = float( value )



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = Task.Task.toString(self)
        buf += "From SearchTask:\n"
        buf +=    "DesiredWavelengthBands = " + str( self.DesiredWavelengthBands ) + "\n" 
        buf +=    "DwellTime = " + str( self.DwellTime ) + "\n" 
        buf +=    "GroundSampleDistance = " + str( self.GroundSampleDistance ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "CMASI") or (len("CMASI") == 0): # this should never happen
        	# Checks for "CMASI" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/SearchTask")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("CMASI" + "/SearchTask")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        Task.Task.toDictMembers(self, d)
        d['DesiredWavelengthBands'] = []
        for x in self.DesiredWavelengthBands:
            d['DesiredWavelengthBands'].append(x)
        d['DwellTime'] = self.DwellTime
        d['GroundSampleDistance'] = self.GroundSampleDistance

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
        str = ws + '<SearchTask Series="CMASI" >\n';
        #str +=Task.Task.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</SearchTask>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += Task.Task.toXMLMembersStr(self, ws)
        buf += ws + "<DesiredWavelengthBands>\n"
        for x in self.DesiredWavelengthBands:
            buf += ws + "<WavelengthBand>" + WavelengthBand.get_WavelengthBand_int(x) + "</WavelengthBand>\n"
        buf += ws + "</DesiredWavelengthBands>\n"
        buf += ws + "<DwellTime>" + str(self.DwellTime) + "</DwellTime>\n"
        buf += ws + "<GroundSampleDistance>" + str(self.GroundSampleDistance) + "</GroundSampleDistance>\n"

        return buf
        
