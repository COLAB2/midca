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

from midca.domains.tsp.lmcp.py.afrl.cmasi import AbstractGeometry
from midca.domains.tsp.lmcp.py.afrl.impact import AreaActionOptions


class AreaOfInterest(LMCPObject.LMCPObject):

    def __init__(self):

        self.LMCP_TYPE = 21
        self.SERIES_NAME = "IMPACT"
        self.FULL_LMCP_TYPE_NAME = "afrl.impact.AreaOfInterest"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 5281966179208134656
        self.SERIES_VERSION = 14

        #Define message fields
        self.AreaID = 0   #int64
        self.Area = AbstractGeometry.AbstractGeometry()   #AbstractGeometry
        self.AreaAction = AreaActionOptions.AreaActionOptions.Created   #AreaActionOptions
        self.AreaLabel = ""   #string
        self.BackgroundBehaviorArea = False   #bool


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(LMCPObject.LMCPObject.pack(self))
        buffer.extend(struct.pack(">q", self.AreaID))
        buffer.extend(struct.pack("B", self.Area != None ))
        if self.Area != None:
            buffer.extend(struct.pack(">q", self.Area.SERIES_NAME_ID))
            buffer.extend(struct.pack(">I", self.Area.LMCP_TYPE))
            buffer.extend(struct.pack(">H", self.Area.SERIES_VERSION))
            buffer.extend(self.Area.pack())
        buffer.extend(struct.pack(">i", self.AreaAction))
        buffer.extend(struct.pack(">H", len(self.AreaLabel) ))
        if len(self.AreaLabel) > 0:
            if (sys.version_info > (3, 0)):
                buffer.extend(struct.pack( repr(len(self.AreaLabel)) + "s", self.AreaLabel.encode('utf-8')))
            else:
                buffer.extend(struct.pack( repr(len(self.AreaLabel)) + "s", self.AreaLabel))
        boolChar = 1 if self.BackgroundBehaviorArea == True else 0
        buffer.extend(struct.pack(">B",boolChar))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = LMCPObject.LMCPObject.unpack(self, buffer, _pos)
        self.AreaID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
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
            self.Area = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
            _pos = self.Area.unpack(buffer, _pos)
        else:
            self.Area = None
        self.AreaAction = struct.unpack_from(">i", buffer, _pos)[0]
        _pos += 4
        _strlen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        if _strlen > 0:
            if (sys.version_info > (3, 0)):
                self.AreaLabel = struct.unpack_from( repr(_strlen) + "s", buffer, _pos )[0].decode('utf-8')
            else:
                 self.AreaLabel = struct.unpack_from( repr(_strlen) + "s", buffer, _pos )[0]
            _pos += _strlen
        else:
             self.AreaLabel = ""
        boolChar = struct.unpack_from(">B", buffer, _pos)[0]
        self.BackgroundBehaviorArea = True if boolChar == 1 else False
        _pos += 1
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        LMCPObject.LMCPObject.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "AreaID" and len(e.childNodes) > 0 :
                    self.AreaID = int(e.childNodes[0].nodeValue)
                elif e.localName == "Area" and len(e.childNodes) > 0 :
                    for n in e.childNodes:
                        if n.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.Area = seriesFactory.createObjectByName(n.getAttribute('Series'), n.localName)
                            if self.Area != None:
                                self.Area.unpackFromXMLNode(n, seriesFactory)
                elif e.localName == "AreaAction" and len(e.childNodes) > 0 :
                    self.AreaAction = AreaActionOptions.get_AreaActionOptions_str(e.childNodes[0].nodeValue)
                elif e.localName == "AreaLabel" :
                    self.AreaLabel = str(e.childNodes[0].nodeValue) if len(e.childNodes) > 0 else ""
                elif e.localName == "BackgroundBehaviorArea" and len(e.childNodes) > 0 :
                    self.BackgroundBehaviorArea = e.childNodes[0].nodeValue.lower() == 'true'

        return

    def unpackFromDict(self, d, seriesFactory):
        LMCPObject.LMCPObject.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "AreaID":
                self.AreaID = d[key]
            elif key == "Area":
                self.Area = seriesFactory.unpackFromDict(d[key])
            elif key == "AreaAction":
                self.AreaAction = d[key]
            elif key == "AreaLabel":
                self.AreaLabel = d[key]
            elif key == "BackgroundBehaviorArea":
                self.BackgroundBehaviorArea = d[key]

        return

    def get_AreaID(self):
        return self.AreaID

    def set_AreaID(self, value):
        self.AreaID = int( value )

    def get_Area(self):
        return self.Area

    def set_Area(self, value):
        self.Area = value 

    def get_AreaAction(self):
        return self.AreaAction

    def set_AreaAction(self, value):
        self.AreaAction = value 

    def get_AreaLabel(self):
        return self.AreaLabel

    def set_AreaLabel(self, value):
        self.AreaLabel = str( value )

    def get_BackgroundBehaviorArea(self):
        return self.BackgroundBehaviorArea

    def set_BackgroundBehaviorArea(self, value):
        self.BackgroundBehaviorArea = bool( value )



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = LMCPObject.LMCPObject.toString(self)
        buf += "From AreaOfInterest:\n"
        buf +=    "AreaID = " + str( self.AreaID ) + "\n" 
        buf +=    "Area = " + str( self.Area ) + "\n" 
        buf +=    "AreaAction = " + str( self.AreaAction ) + "\n" 
        buf +=    "AreaLabel = " + str( self.AreaLabel ) + "\n" 
        buf +=    "BackgroundBehaviorArea = " + str( self.BackgroundBehaviorArea ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "IMPACT") or (len("IMPACT") == 0): # this should never happen
        	# Checks for "IMPACT" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/AreaOfInterest")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("IMPACT" + "/AreaOfInterest")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        LMCPObject.LMCPObject.toDictMembers(self, d)
        d['AreaID'] = self.AreaID
        if self.Area == None:
            d['Area'] = None
        else:
            d['Area'] = self.Area.toDict()
        d['AreaAction'] = self.AreaAction
        d['AreaLabel'] = self.AreaLabel
        d['BackgroundBehaviorArea'] = self.BackgroundBehaviorArea

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
        str = ws + '<AreaOfInterest Series="IMPACT" >\n';
        #str +=LMCPObject.LMCPObject.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</AreaOfInterest>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += LMCPObject.LMCPObject.toXMLMembersStr(self, ws)
        buf += ws + "<AreaID>" + str(self.AreaID) + "</AreaID>\n"
        if self.Area != None:
            buf += ws + "<Area>\n"
            buf += ws + self.Area.toXMLStr(ws + "    ") 
            buf += ws + "</Area>\n"
        buf += ws + "<AreaAction>" + AreaActionOptions.get_AreaActionOptions_int(self.AreaAction) + "</AreaAction>\n"
        buf += ws + "<AreaLabel>" + str(self.AreaLabel) + "</AreaLabel>\n"
        buf += ws + "<BackgroundBehaviorArea>" + ('True' if self.BackgroundBehaviorArea else 'False') + "</BackgroundBehaviorArea>\n"

        return buf
        
