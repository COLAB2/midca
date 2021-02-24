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



class OnboardStatusReport(LMCPObject.LMCPObject):

    def __init__(self):

        self.LMCP_TYPE = 13
        self.SERIES_NAME = "UXNATIVE"
        self.FULL_LMCP_TYPE_NAME = "uxas.messages.uxnative.OnboardStatusReport"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 6149751333668345413
        self.SERIES_VERSION = 9

        #Define message fields
        self.VehicleID = 0   #int64
        self.ConnectedEntities = []   #int64
        self.CurrentTaskList = []   #int64
        self.ValidState = False   #bool
        self.ValidAuthorization = False   #bool
        self.SpeedAuthorization = False   #bool
        self.GimbalAuthorization = False   #bool
        self.VehicleTime = 0   #int64


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(LMCPObject.LMCPObject.pack(self))
        buffer.extend(struct.pack(">q", self.VehicleID))
        buffer.extend(struct.pack(">H", len(self.ConnectedEntities) ))
        for x in self.ConnectedEntities:
            buffer.extend(struct.pack(">q", x ))
        buffer.extend(struct.pack(">H", len(self.CurrentTaskList) ))
        for x in self.CurrentTaskList:
            buffer.extend(struct.pack(">q", x ))
        boolChar = 1 if self.ValidState == True else 0
        buffer.extend(struct.pack(">B",boolChar))
        boolChar = 1 if self.ValidAuthorization == True else 0
        buffer.extend(struct.pack(">B",boolChar))
        boolChar = 1 if self.SpeedAuthorization == True else 0
        buffer.extend(struct.pack(">B",boolChar))
        boolChar = 1 if self.GimbalAuthorization == True else 0
        buffer.extend(struct.pack(">B",boolChar))
        buffer.extend(struct.pack(">q", self.VehicleTime))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = LMCPObject.LMCPObject.unpack(self, buffer, _pos)
        self.VehicleID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.ConnectedEntities = [None] * _arraylen
        if _arraylen > 0:
            self.ConnectedEntities = struct.unpack_from(">" + repr(_arraylen) + "q", buffer, _pos )
            _pos += 8 * _arraylen
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.CurrentTaskList = [None] * _arraylen
        if _arraylen > 0:
            self.CurrentTaskList = struct.unpack_from(">" + repr(_arraylen) + "q", buffer, _pos )
            _pos += 8 * _arraylen
        boolChar = struct.unpack_from(">B", buffer, _pos)[0]
        self.ValidState = True if boolChar == 1 else False
        _pos += 1
        boolChar = struct.unpack_from(">B", buffer, _pos)[0]
        self.ValidAuthorization = True if boolChar == 1 else False
        _pos += 1
        boolChar = struct.unpack_from(">B", buffer, _pos)[0]
        self.SpeedAuthorization = True if boolChar == 1 else False
        _pos += 1
        boolChar = struct.unpack_from(">B", buffer, _pos)[0]
        self.GimbalAuthorization = True if boolChar == 1 else False
        _pos += 1
        self.VehicleTime = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        LMCPObject.LMCPObject.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "VehicleID" and len(e.childNodes) > 0 :
                    self.VehicleID = int(e.childNodes[0].nodeValue)
                elif e.localName == "ConnectedEntities" and len(e.childNodes) > 0 :
                    self.ConnectedEntities = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.ConnectedEntities.append( int(c.childNodes[0].nodeValue) )
                elif e.localName == "CurrentTaskList" and len(e.childNodes) > 0 :
                    self.CurrentTaskList = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.CurrentTaskList.append( int(c.childNodes[0].nodeValue) )
                elif e.localName == "ValidState" and len(e.childNodes) > 0 :
                    self.ValidState = e.childNodes[0].nodeValue.lower() == 'true'
                elif e.localName == "ValidAuthorization" and len(e.childNodes) > 0 :
                    self.ValidAuthorization = e.childNodes[0].nodeValue.lower() == 'true'
                elif e.localName == "SpeedAuthorization" and len(e.childNodes) > 0 :
                    self.SpeedAuthorization = e.childNodes[0].nodeValue.lower() == 'true'
                elif e.localName == "GimbalAuthorization" and len(e.childNodes) > 0 :
                    self.GimbalAuthorization = e.childNodes[0].nodeValue.lower() == 'true'
                elif e.localName == "VehicleTime" and len(e.childNodes) > 0 :
                    self.VehicleTime = int(e.childNodes[0].nodeValue)

        return

    def unpackFromDict(self, d, seriesFactory):
        LMCPObject.LMCPObject.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "VehicleID":
                self.VehicleID = d[key]
            elif key == "ConnectedEntities":
                self.ConnectedEntities = []
                for c in d[key]:
                    self.ConnectedEntities.append( c )
            elif key == "CurrentTaskList":
                self.CurrentTaskList = []
                for c in d[key]:
                    self.CurrentTaskList.append( c )
            elif key == "ValidState":
                self.ValidState = d[key]
            elif key == "ValidAuthorization":
                self.ValidAuthorization = d[key]
            elif key == "SpeedAuthorization":
                self.SpeedAuthorization = d[key]
            elif key == "GimbalAuthorization":
                self.GimbalAuthorization = d[key]
            elif key == "VehicleTime":
                self.VehicleTime = d[key]

        return

    def get_VehicleID(self):
        return self.VehicleID

    def set_VehicleID(self, value):
        self.VehicleID = int( value )

    def get_ConnectedEntities(self):
        return self.ConnectedEntities

    def get_CurrentTaskList(self):
        return self.CurrentTaskList

    def get_ValidState(self):
        return self.ValidState

    def set_ValidState(self, value):
        self.ValidState = bool( value )

    def get_ValidAuthorization(self):
        return self.ValidAuthorization

    def set_ValidAuthorization(self, value):
        self.ValidAuthorization = bool( value )

    def get_SpeedAuthorization(self):
        return self.SpeedAuthorization

    def set_SpeedAuthorization(self, value):
        self.SpeedAuthorization = bool( value )

    def get_GimbalAuthorization(self):
        return self.GimbalAuthorization

    def set_GimbalAuthorization(self, value):
        self.GimbalAuthorization = bool( value )

    def get_VehicleTime(self):
        return self.VehicleTime

    def set_VehicleTime(self, value):
        self.VehicleTime = int( value )



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = LMCPObject.LMCPObject.toString(self)
        buf += "From OnboardStatusReport:\n"
        buf +=    "VehicleID = " + str( self.VehicleID ) + "\n" 
        buf +=    "ConnectedEntities = " + str( self.ConnectedEntities ) + "\n" 
        buf +=    "CurrentTaskList = " + str( self.CurrentTaskList ) + "\n" 
        buf +=    "ValidState = " + str( self.ValidState ) + "\n" 
        buf +=    "ValidAuthorization = " + str( self.ValidAuthorization ) + "\n" 
        buf +=    "SpeedAuthorization = " + str( self.SpeedAuthorization ) + "\n" 
        buf +=    "GimbalAuthorization = " + str( self.GimbalAuthorization ) + "\n" 
        buf +=    "VehicleTime = " + str( self.VehicleTime ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "UXNATIVE") or (len("UXNATIVE") == 0): # this should never happen
        	# Checks for "UXNATIVE" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/OnboardStatusReport")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("UXNATIVE" + "/OnboardStatusReport")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        LMCPObject.LMCPObject.toDictMembers(self, d)
        d['VehicleID'] = self.VehicleID
        d['ConnectedEntities'] = []
        for x in self.ConnectedEntities:
            d['ConnectedEntities'].append(x)
        d['CurrentTaskList'] = []
        for x in self.CurrentTaskList:
            d['CurrentTaskList'].append(x)
        d['ValidState'] = self.ValidState
        d['ValidAuthorization'] = self.ValidAuthorization
        d['SpeedAuthorization'] = self.SpeedAuthorization
        d['GimbalAuthorization'] = self.GimbalAuthorization
        d['VehicleTime'] = self.VehicleTime

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
        str = ws + '<OnboardStatusReport Series="UXNATIVE" >\n';
        #str +=LMCPObject.LMCPObject.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</OnboardStatusReport>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += LMCPObject.LMCPObject.toXMLMembersStr(self, ws)
        buf += ws + "<VehicleID>" + str(self.VehicleID) + "</VehicleID>\n"
        buf += ws + "<ConnectedEntities>\n"
        for x in self.ConnectedEntities:
            buf += ws + "<int64>" + str(x) + "</int64>\n"
        buf += ws + "</ConnectedEntities>\n"
        buf += ws + "<CurrentTaskList>\n"
        for x in self.CurrentTaskList:
            buf += ws + "<int64>" + str(x) + "</int64>\n"
        buf += ws + "</CurrentTaskList>\n"
        buf += ws + "<ValidState>" + ('True' if self.ValidState else 'False') + "</ValidState>\n"
        buf += ws + "<ValidAuthorization>" + ('True' if self.ValidAuthorization else 'False') + "</ValidAuthorization>\n"
        buf += ws + "<SpeedAuthorization>" + ('True' if self.SpeedAuthorization else 'False') + "</SpeedAuthorization>\n"
        buf += ws + "<GimbalAuthorization>" + ('True' if self.GimbalAuthorization else 'False') + "</GimbalAuthorization>\n"
        buf += ws + "<VehicleTime>" + str(self.VehicleTime) + "</VehicleTime>\n"

        return buf
        
