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



class AutopilotKeepAlive(LMCPObject.LMCPObject):

    def __init__(self):

        self.LMCP_TYPE = 12
        self.SERIES_NAME = "UXNATIVE"
        self.FULL_LMCP_TYPE_NAME = "uxas.messages.uxnative.AutopilotKeepAlive"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 6149751333668345413
        self.SERIES_VERSION = 9

        #Define message fields
        self.AutopilotEnabled = True   #bool
        self.SpeedAuthorized = True   #bool
        self.GimbalEnabled = True   #bool
        self.TimeSent = 0   #int64
        self.SpeedOverride = -1.0   #real32
        self.AltOverride = -1.0   #real32


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(LMCPObject.LMCPObject.pack(self))
        boolChar = 1 if self.AutopilotEnabled == True else 0
        buffer.extend(struct.pack(">B",boolChar))
        boolChar = 1 if self.SpeedAuthorized == True else 0
        buffer.extend(struct.pack(">B",boolChar))
        boolChar = 1 if self.GimbalEnabled == True else 0
        buffer.extend(struct.pack(">B",boolChar))
        buffer.extend(struct.pack(">q", self.TimeSent))
        buffer.extend(struct.pack(">f", self.SpeedOverride))
        buffer.extend(struct.pack(">f", self.AltOverride))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = LMCPObject.LMCPObject.unpack(self, buffer, _pos)
        boolChar = struct.unpack_from(">B", buffer, _pos)[0]
        self.AutopilotEnabled = True if boolChar == 1 else False
        _pos += 1
        boolChar = struct.unpack_from(">B", buffer, _pos)[0]
        self.SpeedAuthorized = True if boolChar == 1 else False
        _pos += 1
        boolChar = struct.unpack_from(">B", buffer, _pos)[0]
        self.GimbalEnabled = True if boolChar == 1 else False
        _pos += 1
        self.TimeSent = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.SpeedOverride = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.AltOverride = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        LMCPObject.LMCPObject.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "AutopilotEnabled" and len(e.childNodes) > 0 :
                    self.AutopilotEnabled = e.childNodes[0].nodeValue.lower() == 'true'
                elif e.localName == "SpeedAuthorized" and len(e.childNodes) > 0 :
                    self.SpeedAuthorized = e.childNodes[0].nodeValue.lower() == 'true'
                elif e.localName == "GimbalEnabled" and len(e.childNodes) > 0 :
                    self.GimbalEnabled = e.childNodes[0].nodeValue.lower() == 'true'
                elif e.localName == "TimeSent" and len(e.childNodes) > 0 :
                    self.TimeSent = int(e.childNodes[0].nodeValue)
                elif e.localName == "SpeedOverride" and len(e.childNodes) > 0 :
                    self.SpeedOverride = float(e.childNodes[0].nodeValue)
                elif e.localName == "AltOverride" and len(e.childNodes) > 0 :
                    self.AltOverride = float(e.childNodes[0].nodeValue)

        return

    def unpackFromDict(self, d, seriesFactory):
        LMCPObject.LMCPObject.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "AutopilotEnabled":
                self.AutopilotEnabled = d[key]
            elif key == "SpeedAuthorized":
                self.SpeedAuthorized = d[key]
            elif key == "GimbalEnabled":
                self.GimbalEnabled = d[key]
            elif key == "TimeSent":
                self.TimeSent = d[key]
            elif key == "SpeedOverride":
                self.SpeedOverride = d[key]
            elif key == "AltOverride":
                self.AltOverride = d[key]

        return

    def get_AutopilotEnabled(self):
        return self.AutopilotEnabled

    def set_AutopilotEnabled(self, value):
        self.AutopilotEnabled = bool( value )

    def get_SpeedAuthorized(self):
        return self.SpeedAuthorized

    def set_SpeedAuthorized(self, value):
        self.SpeedAuthorized = bool( value )

    def get_GimbalEnabled(self):
        return self.GimbalEnabled

    def set_GimbalEnabled(self, value):
        self.GimbalEnabled = bool( value )

    def get_TimeSent(self):
        return self.TimeSent

    def set_TimeSent(self, value):
        self.TimeSent = int( value )

    def get_SpeedOverride(self):
        return self.SpeedOverride

    def set_SpeedOverride(self, value):
        self.SpeedOverride = float( value )

    def get_AltOverride(self):
        return self.AltOverride

    def set_AltOverride(self, value):
        self.AltOverride = float( value )



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = LMCPObject.LMCPObject.toString(self)
        buf += "From AutopilotKeepAlive:\n"
        buf +=    "AutopilotEnabled = " + str( self.AutopilotEnabled ) + "\n" 
        buf +=    "SpeedAuthorized = " + str( self.SpeedAuthorized ) + "\n" 
        buf +=    "GimbalEnabled = " + str( self.GimbalEnabled ) + "\n" 
        buf +=    "TimeSent = " + str( self.TimeSent ) + "\n" 
        buf +=    "SpeedOverride = " + str( self.SpeedOverride ) + "\n" 
        buf +=    "AltOverride = " + str( self.AltOverride ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "UXNATIVE") or (len("UXNATIVE") == 0): # this should never happen
        	# Checks for "UXNATIVE" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/AutopilotKeepAlive")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("UXNATIVE" + "/AutopilotKeepAlive")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        LMCPObject.LMCPObject.toDictMembers(self, d)
        d['AutopilotEnabled'] = self.AutopilotEnabled
        d['SpeedAuthorized'] = self.SpeedAuthorized
        d['GimbalEnabled'] = self.GimbalEnabled
        d['TimeSent'] = self.TimeSent
        d['SpeedOverride'] = self.SpeedOverride
        d['AltOverride'] = self.AltOverride

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
        str = ws + '<AutopilotKeepAlive Series="UXNATIVE" >\n';
        #str +=LMCPObject.LMCPObject.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</AutopilotKeepAlive>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += LMCPObject.LMCPObject.toXMLMembersStr(self, ws)
        buf += ws + "<AutopilotEnabled>" + ('True' if self.AutopilotEnabled else 'False') + "</AutopilotEnabled>\n"
        buf += ws + "<SpeedAuthorized>" + ('True' if self.SpeedAuthorized else 'False') + "</SpeedAuthorized>\n"
        buf += ws + "<GimbalEnabled>" + ('True' if self.GimbalEnabled else 'False') + "</GimbalEnabled>\n"
        buf += ws + "<TimeSent>" + str(self.TimeSent) + "</TimeSent>\n"
        buf += ws + "<SpeedOverride>" + str(self.SpeedOverride) + "</SpeedOverride>\n"
        buf += ws + "<AltOverride>" + str(self.AltOverride) + "</AltOverride>\n"

        return buf
        
