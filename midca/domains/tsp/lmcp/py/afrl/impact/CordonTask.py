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
from midca.domains.tsp.lmcp.py.afrl.cmasi import Location3D


class CordonTask(Task.Task):

    def __init__(self):
        Task.Task.__init__(self)
        self.LMCP_TYPE = 29
        self.SERIES_NAME = "IMPACT"
        self.FULL_LMCP_TYPE_NAME = "afrl.impact.CordonTask"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 5281966179208134656
        self.SERIES_VERSION = 14

        #Define message fields
        self.CordonLocation = Location3D.Location3D()   #Location3D
        self.StandoffDistance = 100   #real32


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(Task.Task.pack(self))
        buffer.extend(struct.pack("B", self.CordonLocation != None ))
        if self.CordonLocation != None:
            buffer.extend(struct.pack(">q", self.CordonLocation.SERIES_NAME_ID))
            buffer.extend(struct.pack(">I", self.CordonLocation.LMCP_TYPE))
            buffer.extend(struct.pack(">H", self.CordonLocation.SERIES_VERSION))
            buffer.extend(self.CordonLocation.pack())
        buffer.extend(struct.pack(">f", self.StandoffDistance))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = Task.Task.unpack(self, buffer, _pos)
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
            self.CordonLocation = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
            _pos = self.CordonLocation.unpack(buffer, _pos)
        else:
            self.CordonLocation = None
        self.StandoffDistance = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        Task.Task.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "CordonLocation" and len(e.childNodes) > 0 :
                    for n in e.childNodes:
                        if n.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.CordonLocation = seriesFactory.createObjectByName(n.getAttribute('Series'), n.localName)
                            if self.CordonLocation != None:
                                self.CordonLocation.unpackFromXMLNode(n, seriesFactory)
                elif e.localName == "StandoffDistance" and len(e.childNodes) > 0 :
                    self.StandoffDistance = float(e.childNodes[0].nodeValue)

        return

    def unpackFromDict(self, d, seriesFactory):
        Task.Task.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "CordonLocation":
                self.CordonLocation = seriesFactory.unpackFromDict(d[key])
            elif key == "StandoffDistance":
                self.StandoffDistance = d[key]

        return

    def get_CordonLocation(self):
        return self.CordonLocation

    def set_CordonLocation(self, value):
        self.CordonLocation = value 

    def get_StandoffDistance(self):
        return self.StandoffDistance

    def set_StandoffDistance(self, value):
        self.StandoffDistance = float( value )



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = Task.Task.toString(self)
        buf += "From CordonTask:\n"
        buf +=    "CordonLocation = " + str( self.CordonLocation ) + "\n" 
        buf +=    "StandoffDistance = " + str( self.StandoffDistance ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "IMPACT") or (len("IMPACT") == 0): # this should never happen
        	# Checks for "IMPACT" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/CordonTask")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("IMPACT" + "/CordonTask")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        Task.Task.toDictMembers(self, d)
        if self.CordonLocation == None:
            d['CordonLocation'] = None
        else:
            d['CordonLocation'] = self.CordonLocation.toDict()
        d['StandoffDistance'] = self.StandoffDistance

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
        str = ws + '<CordonTask Series="IMPACT" >\n';
        #str +=Task.Task.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</CordonTask>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += Task.Task.toXMLMembersStr(self, ws)
        if self.CordonLocation != None:
            buf += ws + "<CordonLocation>\n"
            buf += ws + self.CordonLocation.toXMLStr(ws + "    ") 
            buf += ws + "</CordonLocation>\n"
        buf += ws + "<StandoffDistance>" + str(self.StandoffDistance) + "</StandoffDistance>\n"

        return buf
        
