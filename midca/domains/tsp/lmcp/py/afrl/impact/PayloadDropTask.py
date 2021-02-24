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


class PayloadDropTask(Task.Task):

    def __init__(self):
        Task.Task.__init__(self)
        self.LMCP_TYPE = 35
        self.SERIES_NAME = "IMPACT"
        self.FULL_LMCP_TYPE_NAME = "afrl.impact.PayloadDropTask"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 5281966179208134656
        self.SERIES_VERSION = 14

        #Define message fields
        self.DropLocation = Location3D.Location3D()   #Location3D
        self.BDALocation = Location3D.Location3D()   #Location3D


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(Task.Task.pack(self))
        buffer.extend(struct.pack("B", self.DropLocation != None ))
        if self.DropLocation != None:
            buffer.extend(struct.pack(">q", self.DropLocation.SERIES_NAME_ID))
            buffer.extend(struct.pack(">I", self.DropLocation.LMCP_TYPE))
            buffer.extend(struct.pack(">H", self.DropLocation.SERIES_VERSION))
            buffer.extend(self.DropLocation.pack())
        buffer.extend(struct.pack("B", self.BDALocation != None ))
        if self.BDALocation != None:
            buffer.extend(struct.pack(">q", self.BDALocation.SERIES_NAME_ID))
            buffer.extend(struct.pack(">I", self.BDALocation.LMCP_TYPE))
            buffer.extend(struct.pack(">H", self.BDALocation.SERIES_VERSION))
            buffer.extend(self.BDALocation.pack())

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
            self.DropLocation = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
            _pos = self.DropLocation.unpack(buffer, _pos)
        else:
            self.DropLocation = None
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
            self.BDALocation = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
            _pos = self.BDALocation.unpack(buffer, _pos)
        else:
            self.BDALocation = None
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        Task.Task.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "DropLocation" and len(e.childNodes) > 0 :
                    for n in e.childNodes:
                        if n.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.DropLocation = seriesFactory.createObjectByName(n.getAttribute('Series'), n.localName)
                            if self.DropLocation != None:
                                self.DropLocation.unpackFromXMLNode(n, seriesFactory)
                elif e.localName == "BDALocation" and len(e.childNodes) > 0 :
                    for n in e.childNodes:
                        if n.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.BDALocation = seriesFactory.createObjectByName(n.getAttribute('Series'), n.localName)
                            if self.BDALocation != None:
                                self.BDALocation.unpackFromXMLNode(n, seriesFactory)

        return

    def unpackFromDict(self, d, seriesFactory):
        Task.Task.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "DropLocation":
                self.DropLocation = seriesFactory.unpackFromDict(d[key])
            elif key == "BDALocation":
                self.BDALocation = seriesFactory.unpackFromDict(d[key])

        return

    def get_DropLocation(self):
        return self.DropLocation

    def set_DropLocation(self, value):
        self.DropLocation = value 

    def get_BDALocation(self):
        return self.BDALocation

    def set_BDALocation(self, value):
        self.BDALocation = value 



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = Task.Task.toString(self)
        buf += "From PayloadDropTask:\n"
        buf +=    "DropLocation = " + str( self.DropLocation ) + "\n" 
        buf +=    "BDALocation = " + str( self.BDALocation ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "IMPACT") or (len("IMPACT") == 0): # this should never happen
        	# Checks for "IMPACT" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/PayloadDropTask")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("IMPACT" + "/PayloadDropTask")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        Task.Task.toDictMembers(self, d)
        if self.DropLocation == None:
            d['DropLocation'] = None
        else:
            d['DropLocation'] = self.DropLocation.toDict()
        if self.BDALocation == None:
            d['BDALocation'] = None
        else:
            d['BDALocation'] = self.BDALocation.toDict()

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
        str = ws + '<PayloadDropTask Series="IMPACT" >\n';
        #str +=Task.Task.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</PayloadDropTask>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += Task.Task.toXMLMembersStr(self, ws)
        if self.DropLocation != None:
            buf += ws + "<DropLocation>\n"
            buf += ws + self.DropLocation.toXMLStr(ws + "    ") 
            buf += ws + "</DropLocation>\n"
        if self.BDALocation != None:
            buf += ws + "<BDALocation>\n"
            buf += ws + self.BDALocation.toXMLStr(ws + "    ") 
            buf += ws + "</BDALocation>\n"

        return buf
        
