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


class BandwidthTest(EntityLocation.EntityLocation):

    def __init__(self):
        EntityLocation.EntityLocation.__init__(self)
        self.LMCP_TYPE = 8
        self.SERIES_NAME = "UXNATIVE"
        self.FULL_LMCP_TYPE_NAME = "uxas.messages.uxnative.BandwidthTest"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 6149751333668345413
        self.SERIES_VERSION = 9

        #Define message fields
        self.MessageID = 0   #int64
        self.Payload = ""   #string


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(EntityLocation.EntityLocation.pack(self))
        buffer.extend(struct.pack(">q", self.MessageID))
        buffer.extend(struct.pack(">H", len(self.Payload) ))
        if len(self.Payload) > 0:
            if (sys.version_info > (3, 0)):
                buffer.extend(struct.pack( repr(len(self.Payload)) + "s", self.Payload.encode('utf-8')))
            else:
                buffer.extend(struct.pack( repr(len(self.Payload)) + "s", self.Payload))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = EntityLocation.EntityLocation.unpack(self, buffer, _pos)
        self.MessageID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        _strlen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        if _strlen > 0:
            if (sys.version_info > (3, 0)):
                self.Payload = struct.unpack_from( repr(_strlen) + "s", buffer, _pos )[0].decode('utf-8')
            else:
                 self.Payload = struct.unpack_from( repr(_strlen) + "s", buffer, _pos )[0]
            _pos += _strlen
        else:
             self.Payload = ""
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        EntityLocation.EntityLocation.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "MessageID" and len(e.childNodes) > 0 :
                    self.MessageID = int(e.childNodes[0].nodeValue)
                elif e.localName == "Payload" :
                    self.Payload = str(e.childNodes[0].nodeValue) if len(e.childNodes) > 0 else ""

        return

    def unpackFromDict(self, d, seriesFactory):
        EntityLocation.EntityLocation.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "MessageID":
                self.MessageID = d[key]
            elif key == "Payload":
                self.Payload = d[key]

        return

    def get_MessageID(self):
        return self.MessageID

    def set_MessageID(self, value):
        self.MessageID = int( value )

    def get_Payload(self):
        return self.Payload

    def set_Payload(self, value):
        self.Payload = str( value )



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = EntityLocation.EntityLocation.toString(self)
        buf += "From BandwidthTest:\n"
        buf +=    "MessageID = " + str( self.MessageID ) + "\n" 
        buf +=    "Payload = " + str( self.Payload ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "UXNATIVE") or (len("UXNATIVE") == 0): # this should never happen
        	# Checks for "UXNATIVE" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/BandwidthTest")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("UXNATIVE" + "/BandwidthTest")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        EntityLocation.EntityLocation.toDictMembers(self, d)
        d['MessageID'] = self.MessageID
        d['Payload'] = self.Payload

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
        str = ws + '<BandwidthTest Series="UXNATIVE" >\n';
        #str +=EntityLocation.EntityLocation.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</BandwidthTest>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += EntityLocation.EntityLocation.toXMLMembersStr(self, ws)
        buf += ws + "<MessageID>" + str(self.MessageID) + "</MessageID>\n"
        buf += ws + "<Payload>" + str(self.Payload) + "</Payload>\n"

        return buf
        
