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

from midca.domains.tsp.lmcp.py.afrl.cmasi import Location3D
from midca.domains.tsp.lmcp.py.afrl.impact import AreaActionOptions


class LineOfInterest(LMCPObject.LMCPObject):

    def __init__(self):

        self.LMCP_TYPE = 20
        self.SERIES_NAME = "IMPACT"
        self.FULL_LMCP_TYPE_NAME = "afrl.impact.LineOfInterest"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 5281966179208134656
        self.SERIES_VERSION = 14

        #Define message fields
        self.LineID = 0   #int64
        self.Line = []   #Location3D
        self.LineAction = AreaActionOptions.AreaActionOptions.Created   #AreaActionOptions
        self.LineLabel = ""   #string
        self.BackgroundBehaviorLine = False   #bool


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(LMCPObject.LMCPObject.pack(self))
        buffer.extend(struct.pack(">q", self.LineID))
        buffer.extend(struct.pack(">H", len(self.Line) ))
        for x in self.Line:
           buffer.extend(struct.pack("B", x != None ))
           if x != None:
               buffer.extend(struct.pack(">q", x.SERIES_NAME_ID))
               buffer.extend(struct.pack(">I", x.LMCP_TYPE))
               buffer.extend(struct.pack(">H", x.SERIES_VERSION))
               buffer.extend(x.pack())
        buffer.extend(struct.pack(">i", self.LineAction))
        buffer.extend(struct.pack(">H", len(self.LineLabel) ))
        if len(self.LineLabel) > 0:
            if (sys.version_info > (3, 0)):
                buffer.extend(struct.pack( repr(len(self.LineLabel)) + "s", self.LineLabel.encode('utf-8')))
            else:
                buffer.extend(struct.pack( repr(len(self.LineLabel)) + "s", self.LineLabel))
        boolChar = 1 if self.BackgroundBehaviorLine == True else 0
        buffer.extend(struct.pack(">B",boolChar))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = LMCPObject.LMCPObject.unpack(self, buffer, _pos)
        self.LineID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.Line = [None] * _arraylen
        for x in range(_arraylen):
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
                self.Line[x] = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
                _pos = self.Line[x].unpack(buffer, _pos)
            else:
                self.Line[x] = None
        self.LineAction = struct.unpack_from(">i", buffer, _pos)[0]
        _pos += 4
        _strlen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        if _strlen > 0:
            if (sys.version_info > (3, 0)):
                self.LineLabel = struct.unpack_from( repr(_strlen) + "s", buffer, _pos )[0].decode('utf-8')
            else:
                 self.LineLabel = struct.unpack_from( repr(_strlen) + "s", buffer, _pos )[0]
            _pos += _strlen
        else:
             self.LineLabel = ""
        boolChar = struct.unpack_from(">B", buffer, _pos)[0]
        self.BackgroundBehaviorLine = True if boolChar == 1 else False
        _pos += 1
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        LMCPObject.LMCPObject.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "LineID" and len(e.childNodes) > 0 :
                    self.LineID = int(e.childNodes[0].nodeValue)
                elif e.localName == "Line" and len(e.childNodes) > 0 :
                    self.Line = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            obj = seriesFactory.createObjectByName(c.getAttribute('Series'), c.localName)
                            if obj != None:
                                obj.unpackFromXMLNode(c, seriesFactory)
                                self.Line.append(obj)
                elif e.localName == "LineAction" and len(e.childNodes) > 0 :
                    self.LineAction = AreaActionOptions.get_AreaActionOptions_str(e.childNodes[0].nodeValue)
                elif e.localName == "LineLabel" :
                    self.LineLabel = str(e.childNodes[0].nodeValue) if len(e.childNodes) > 0 else ""
                elif e.localName == "BackgroundBehaviorLine" and len(e.childNodes) > 0 :
                    self.BackgroundBehaviorLine = e.childNodes[0].nodeValue.lower() == 'true'

        return

    def unpackFromDict(self, d, seriesFactory):
        LMCPObject.LMCPObject.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "LineID":
                self.LineID = d[key]
            elif key == "Line":
                self.Line = []
                for c in d[key]:
                    obj = seriesFactory.unpackFromDict(c)
                    if obj != None:
                        self.Line.append(obj)
            elif key == "LineAction":
                self.LineAction = d[key]
            elif key == "LineLabel":
                self.LineLabel = d[key]
            elif key == "BackgroundBehaviorLine":
                self.BackgroundBehaviorLine = d[key]

        return

    def get_LineID(self):
        return self.LineID

    def set_LineID(self, value):
        self.LineID = int( value )

    def get_Line(self):
        return self.Line

    def get_LineAction(self):
        return self.LineAction

    def set_LineAction(self, value):
        self.LineAction = value 

    def get_LineLabel(self):
        return self.LineLabel

    def set_LineLabel(self, value):
        self.LineLabel = str( value )

    def get_BackgroundBehaviorLine(self):
        return self.BackgroundBehaviorLine

    def set_BackgroundBehaviorLine(self, value):
        self.BackgroundBehaviorLine = bool( value )



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = LMCPObject.LMCPObject.toString(self)
        buf += "From LineOfInterest:\n"
        buf +=    "LineID = " + str( self.LineID ) + "\n" 
        buf +=    "Line = " + str( self.Line ) + "\n" 
        buf +=    "LineAction = " + str( self.LineAction ) + "\n" 
        buf +=    "LineLabel = " + str( self.LineLabel ) + "\n" 
        buf +=    "BackgroundBehaviorLine = " + str( self.BackgroundBehaviorLine ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "IMPACT") or (len("IMPACT") == 0): # this should never happen
        	# Checks for "IMPACT" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/LineOfInterest")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("IMPACT" + "/LineOfInterest")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        LMCPObject.LMCPObject.toDictMembers(self, d)
        d['LineID'] = self.LineID
        d['Line'] = []
        for x in self.Line:
            if x == None:
                d['Line'].append(None)
            else:
                d['Line'].append(x.toDict())
        d['LineAction'] = self.LineAction
        d['LineLabel'] = self.LineLabel
        d['BackgroundBehaviorLine'] = self.BackgroundBehaviorLine

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
        str = ws + '<LineOfInterest Series="IMPACT" >\n';
        #str +=LMCPObject.LMCPObject.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</LineOfInterest>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += LMCPObject.LMCPObject.toXMLMembersStr(self, ws)
        buf += ws + "<LineID>" + str(self.LineID) + "</LineID>\n"
        buf += ws + "<Line>\n"
        for x in self.Line:
            if x == None:
                buf += ws + "    <null/>\n"
            else:
                buf += x.toXMLStr(ws + "    ") 
        buf += ws + "</Line>\n"
        buf += ws + "<LineAction>" + AreaActionOptions.get_AreaActionOptions_int(self.LineAction) + "</LineAction>\n"
        buf += ws + "<LineLabel>" + str(self.LineLabel) + "</LineLabel>\n"
        buf += ws + "<BackgroundBehaviorLine>" + ('True' if self.BackgroundBehaviorLine else 'False') + "</BackgroundBehaviorLine>\n"

        return buf
        
