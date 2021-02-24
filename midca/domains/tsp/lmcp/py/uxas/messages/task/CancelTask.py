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



class CancelTask(LMCPObject.LMCPObject):

    def __init__(self):

        self.LMCP_TYPE = 29
        self.SERIES_NAME = "UXTASK"
        self.FULL_LMCP_TYPE_NAME = "uxas.messages.task.CancelTask"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 6149757930721443840
        self.SERIES_VERSION = 8

        #Define message fields
        self.Vehicles = []   #int64
        self.CanceledTasks = []   #int64


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(LMCPObject.LMCPObject.pack(self))
        buffer.extend(struct.pack(">H", len(self.Vehicles) ))
        for x in self.Vehicles:
            buffer.extend(struct.pack(">q", x ))
        buffer.extend(struct.pack(">H", len(self.CanceledTasks) ))
        for x in self.CanceledTasks:
            buffer.extend(struct.pack(">q", x ))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = LMCPObject.LMCPObject.unpack(self, buffer, _pos)
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.Vehicles = [None] * _arraylen
        if _arraylen > 0:
            self.Vehicles = struct.unpack_from(">" + repr(_arraylen) + "q", buffer, _pos )
            _pos += 8 * _arraylen
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.CanceledTasks = [None] * _arraylen
        if _arraylen > 0:
            self.CanceledTasks = struct.unpack_from(">" + repr(_arraylen) + "q", buffer, _pos )
            _pos += 8 * _arraylen
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        LMCPObject.LMCPObject.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "Vehicles" and len(e.childNodes) > 0 :
                    self.Vehicles = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.Vehicles.append( int(c.childNodes[0].nodeValue) )
                elif e.localName == "CanceledTasks" and len(e.childNodes) > 0 :
                    self.CanceledTasks = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.CanceledTasks.append( int(c.childNodes[0].nodeValue) )

        return

    def unpackFromDict(self, d, seriesFactory):
        LMCPObject.LMCPObject.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "Vehicles":
                self.Vehicles = []
                for c in d[key]:
                    self.Vehicles.append( c )
            elif key == "CanceledTasks":
                self.CanceledTasks = []
                for c in d[key]:
                    self.CanceledTasks.append( c )

        return

    def get_Vehicles(self):
        return self.Vehicles

    def get_CanceledTasks(self):
        return self.CanceledTasks



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = LMCPObject.LMCPObject.toString(self)
        buf += "From CancelTask:\n"
        buf +=    "Vehicles = " + str( self.Vehicles ) + "\n" 
        buf +=    "CanceledTasks = " + str( self.CanceledTasks ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "UXTASK") or (len("UXTASK") == 0): # this should never happen
        	# Checks for "UXTASK" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/CancelTask")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("UXTASK" + "/CancelTask")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        LMCPObject.LMCPObject.toDictMembers(self, d)
        d['Vehicles'] = []
        for x in self.Vehicles:
            d['Vehicles'].append(x)
        d['CanceledTasks'] = []
        for x in self.CanceledTasks:
            d['CanceledTasks'].append(x)

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
        str = ws + '<CancelTask Series="UXTASK" >\n';
        #str +=LMCPObject.LMCPObject.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</CancelTask>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += LMCPObject.LMCPObject.toXMLMembersStr(self, ws)
        buf += ws + "<Vehicles>\n"
        for x in self.Vehicles:
            buf += ws + "<int64>" + str(x) + "</int64>\n"
        buf += ws + "</Vehicles>\n"
        buf += ws + "<CanceledTasks>\n"
        for x in self.CanceledTasks:
            buf += ws + "<int64>" + str(x) + "</int64>\n"
        buf += ws + "</CanceledTasks>\n"

        return buf
        
