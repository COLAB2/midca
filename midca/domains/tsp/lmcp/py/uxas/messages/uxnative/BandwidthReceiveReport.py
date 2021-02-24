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

from midca.domains.tsp.lmcp.py.uxas.messages.uxnative import EntityLocation


class BandwidthReceiveReport(LMCPObject.LMCPObject):

    def __init__(self):

        self.LMCP_TYPE = 9
        self.SERIES_NAME = "UXNATIVE"
        self.FULL_LMCP_TYPE_NAME = "uxas.messages.uxnative.BandwidthReceiveReport"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 6149751333668345413
        self.SERIES_VERSION = 9

        #Define message fields
        self.EntitySender = EntityLocation.EntityLocation()   #EntityLocation
        self.EntityReceiver = EntityLocation.EntityLocation()   #EntityLocation
        self.TransferPayloadSize = 0   #uint32


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(LMCPObject.LMCPObject.pack(self))
        buffer.extend(struct.pack("B", self.EntitySender != None ))
        if self.EntitySender != None:
            buffer.extend(struct.pack(">q", self.EntitySender.SERIES_NAME_ID))
            buffer.extend(struct.pack(">I", self.EntitySender.LMCP_TYPE))
            buffer.extend(struct.pack(">H", self.EntitySender.SERIES_VERSION))
            buffer.extend(self.EntitySender.pack())
        buffer.extend(struct.pack("B", self.EntityReceiver != None ))
        if self.EntityReceiver != None:
            buffer.extend(struct.pack(">q", self.EntityReceiver.SERIES_NAME_ID))
            buffer.extend(struct.pack(">I", self.EntityReceiver.LMCP_TYPE))
            buffer.extend(struct.pack(">H", self.EntityReceiver.SERIES_VERSION))
            buffer.extend(self.EntityReceiver.pack())
        buffer.extend(struct.pack(">I", self.TransferPayloadSize))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = LMCPObject.LMCPObject.unpack(self, buffer, _pos)
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
            self.EntitySender = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
            _pos = self.EntitySender.unpack(buffer, _pos)
        else:
            self.EntitySender = None
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
            self.EntityReceiver = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
            _pos = self.EntityReceiver.unpack(buffer, _pos)
        else:
            self.EntityReceiver = None
        self.TransferPayloadSize = struct.unpack_from(">I", buffer, _pos)[0]
        _pos += 4
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        LMCPObject.LMCPObject.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "EntitySender" and len(e.childNodes) > 0 :
                    for n in e.childNodes:
                        if n.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.EntitySender = seriesFactory.createObjectByName(n.getAttribute('Series'), n.localName)
                            if self.EntitySender != None:
                                self.EntitySender.unpackFromXMLNode(n, seriesFactory)
                elif e.localName == "EntityReceiver" and len(e.childNodes) > 0 :
                    for n in e.childNodes:
                        if n.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.EntityReceiver = seriesFactory.createObjectByName(n.getAttribute('Series'), n.localName)
                            if self.EntityReceiver != None:
                                self.EntityReceiver.unpackFromXMLNode(n, seriesFactory)
                elif e.localName == "TransferPayloadSize" and len(e.childNodes) > 0 :
                    self.TransferPayloadSize = int(e.childNodes[0].nodeValue)

        return

    def unpackFromDict(self, d, seriesFactory):
        LMCPObject.LMCPObject.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "EntitySender":
                self.EntitySender = seriesFactory.unpackFromDict(d[key])
            elif key == "EntityReceiver":
                self.EntityReceiver = seriesFactory.unpackFromDict(d[key])
            elif key == "TransferPayloadSize":
                self.TransferPayloadSize = d[key]

        return

    def get_EntitySender(self):
        return self.EntitySender

    def set_EntitySender(self, value):
        self.EntitySender = value 

    def get_EntityReceiver(self):
        return self.EntityReceiver

    def set_EntityReceiver(self, value):
        self.EntityReceiver = value 

    def get_TransferPayloadSize(self):
        return self.TransferPayloadSize

    def set_TransferPayloadSize(self, value):
        self.TransferPayloadSize = int( value )



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = LMCPObject.LMCPObject.toString(self)
        buf += "From BandwidthReceiveReport:\n"
        buf +=    "EntitySender = " + str( self.EntitySender ) + "\n" 
        buf +=    "EntityReceiver = " + str( self.EntityReceiver ) + "\n" 
        buf +=    "TransferPayloadSize = " + str( self.TransferPayloadSize ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "UXNATIVE") or (len("UXNATIVE") == 0): # this should never happen
        	# Checks for "UXNATIVE" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/BandwidthReceiveReport")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("UXNATIVE" + "/BandwidthReceiveReport")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        LMCPObject.LMCPObject.toDictMembers(self, d)
        if self.EntitySender == None:
            d['EntitySender'] = None
        else:
            d['EntitySender'] = self.EntitySender.toDict()
        if self.EntityReceiver == None:
            d['EntityReceiver'] = None
        else:
            d['EntityReceiver'] = self.EntityReceiver.toDict()
        d['TransferPayloadSize'] = self.TransferPayloadSize

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
        str = ws + '<BandwidthReceiveReport Series="UXNATIVE" >\n';
        #str +=LMCPObject.LMCPObject.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</BandwidthReceiveReport>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += LMCPObject.LMCPObject.toXMLMembersStr(self, ws)
        if self.EntitySender != None:
            buf += ws + "<EntitySender>\n"
            buf += ws + self.EntitySender.toXMLStr(ws + "    ") 
            buf += ws + "</EntitySender>\n"
        if self.EntityReceiver != None:
            buf += ws + "<EntityReceiver>\n"
            buf += ws + self.EntityReceiver.toXMLStr(ws + "    ") 
            buf += ws + "</EntityReceiver>\n"
        buf += ws + "<TransferPayloadSize>" + str(self.TransferPayloadSize) + "</TransferPayloadSize>\n"

        return buf
        
