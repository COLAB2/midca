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

from midca.domains.tsp.lmcp.py.afrl.cmasi import AltitudeType
from midca.domains.tsp.lmcp.py.afrl.cmasi import PayloadConfiguration
from midca.domains.tsp.lmcp.py.afrl.cmasi import KeyValuePair


class EntityConfiguration(LMCPObject.LMCPObject):

    def __init__(self):

        self.LMCP_TYPE = 11
        self.SERIES_NAME = "CMASI"
        self.FULL_LMCP_TYPE_NAME = "afrl.cmasi.EntityConfiguration"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 4849604199710720000
        self.SERIES_VERSION = 3

        #Define message fields
        self.ID = 0   #int64
        self.Affiliation = "Unknown"   #string
        self.EntityType = ""   #string
        self.Label = ""   #string
        self.NominalSpeed = 0   #real32
        self.NominalAltitude = 0.0   #real32
        self.NominalAltitudeType = AltitudeType.AltitudeType.AGL   #AltitudeType
        self.PayloadConfigurationList = []   #PayloadConfiguration
        self.Info = []   #KeyValuePair


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(LMCPObject.LMCPObject.pack(self))
        buffer.extend(struct.pack(">q", self.ID))
        buffer.extend(struct.pack(">H", len(self.Affiliation) ))
        if len(self.Affiliation) > 0:
            if (sys.version_info > (3, 0)):
                buffer.extend(struct.pack( repr(len(self.Affiliation)) + "s", self.Affiliation.encode('utf-8')))
            else:
                buffer.extend(struct.pack( repr(len(self.Affiliation)) + "s", self.Affiliation))
        buffer.extend(struct.pack(">H", len(self.EntityType) ))
        if len(self.EntityType) > 0:
            if (sys.version_info > (3, 0)):
                buffer.extend(struct.pack( repr(len(self.EntityType)) + "s", self.EntityType.encode('utf-8')))
            else:
                buffer.extend(struct.pack( repr(len(self.EntityType)) + "s", self.EntityType))
        buffer.extend(struct.pack(">H", len(self.Label) ))
        if len(self.Label) > 0:
            if (sys.version_info > (3, 0)):
                buffer.extend(struct.pack( repr(len(self.Label)) + "s", self.Label.encode('utf-8')))
            else:
                buffer.extend(struct.pack( repr(len(self.Label)) + "s", self.Label))
        buffer.extend(struct.pack(">f", self.NominalSpeed))
        buffer.extend(struct.pack(">f", self.NominalAltitude))
        buffer.extend(struct.pack(">i", self.NominalAltitudeType))
        buffer.extend(struct.pack(">H", len(self.PayloadConfigurationList) ))
        for x in self.PayloadConfigurationList:
           buffer.extend(struct.pack("B", x != None ))
           if x != None:
               buffer.extend(struct.pack(">q", x.SERIES_NAME_ID))
               buffer.extend(struct.pack(">I", x.LMCP_TYPE))
               buffer.extend(struct.pack(">H", x.SERIES_VERSION))
               buffer.extend(x.pack())
        buffer.extend(struct.pack(">H", len(self.Info) ))
        for x in self.Info:
           buffer.extend(struct.pack("B", x != None ))
           if x != None:
               buffer.extend(struct.pack(">q", x.SERIES_NAME_ID))
               buffer.extend(struct.pack(">I", x.LMCP_TYPE))
               buffer.extend(struct.pack(">H", x.SERIES_VERSION))
               buffer.extend(x.pack())

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = LMCPObject.LMCPObject.unpack(self, buffer, _pos)
        self.ID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        _strlen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        if _strlen > 0:
            if (sys.version_info > (3, 0)):
                self.Affiliation = struct.unpack_from( repr(_strlen) + "s", buffer, _pos )[0].decode('utf-8')
            else:
                 self.Affiliation = struct.unpack_from( repr(_strlen) + "s", buffer, _pos )[0]
            _pos += _strlen
        else:
             self.Affiliation = ""
        _strlen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        if _strlen > 0:
            if (sys.version_info > (3, 0)):
                self.EntityType = struct.unpack_from( repr(_strlen) + "s", buffer, _pos )[0].decode('utf-8')
            else:
                 self.EntityType = struct.unpack_from( repr(_strlen) + "s", buffer, _pos )[0]
            _pos += _strlen
        else:
             self.EntityType = ""
        _strlen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        if _strlen > 0:
            if (sys.version_info > (3, 0)):
                self.Label = struct.unpack_from( repr(_strlen) + "s", buffer, _pos )[0].decode('utf-8')
            else:
                 self.Label = struct.unpack_from( repr(_strlen) + "s", buffer, _pos )[0]
            _pos += _strlen
        else:
             self.Label = ""
        self.NominalSpeed = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.NominalAltitude = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.NominalAltitudeType = struct.unpack_from(">i", buffer, _pos)[0]
        _pos += 4
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.PayloadConfigurationList = [None] * _arraylen
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
                self.PayloadConfigurationList[x] = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
                _pos = self.PayloadConfigurationList[x].unpack(buffer, _pos)
            else:
                self.PayloadConfigurationList[x] = None
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.Info = [None] * _arraylen
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
                self.Info[x] = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
                _pos = self.Info[x].unpack(buffer, _pos)
            else:
                self.Info[x] = None
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        LMCPObject.LMCPObject.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "ID" and len(e.childNodes) > 0 :
                    self.ID = int(e.childNodes[0].nodeValue)
                elif e.localName == "Affiliation" :
                    self.Affiliation = str(e.childNodes[0].nodeValue) if len(e.childNodes) > 0 else ""
                elif e.localName == "EntityType" :
                    self.EntityType = str(e.childNodes[0].nodeValue) if len(e.childNodes) > 0 else ""
                elif e.localName == "Label" :
                    self.Label = str(e.childNodes[0].nodeValue) if len(e.childNodes) > 0 else ""
                elif e.localName == "NominalSpeed" and len(e.childNodes) > 0 :
                    self.NominalSpeed = float(e.childNodes[0].nodeValue)
                elif e.localName == "NominalAltitude" and len(e.childNodes) > 0 :
                    self.NominalAltitude = float(e.childNodes[0].nodeValue)
                elif e.localName == "NominalAltitudeType" and len(e.childNodes) > 0 :
                    self.NominalAltitudeType = AltitudeType.get_AltitudeType_str(e.childNodes[0].nodeValue)
                elif e.localName == "PayloadConfigurationList" and len(e.childNodes) > 0 :
                    self.PayloadConfigurationList = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            obj = seriesFactory.createObjectByName(c.getAttribute('Series'), c.localName)
                            if obj != None:
                                obj.unpackFromXMLNode(c, seriesFactory)
                                self.PayloadConfigurationList.append(obj)
                elif e.localName == "Info" and len(e.childNodes) > 0 :
                    self.Info = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            obj = seriesFactory.createObjectByName(c.getAttribute('Series'), c.localName)
                            if obj != None:
                                obj.unpackFromXMLNode(c, seriesFactory)
                                self.Info.append(obj)

        return

    def unpackFromDict(self, d, seriesFactory):
        LMCPObject.LMCPObject.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "ID":
                self.ID = d[key]
            elif key == "Affiliation":
                self.Affiliation = d[key]
            elif key == "EntityType":
                self.EntityType = d[key]
            elif key == "Label":
                self.Label = d[key]
            elif key == "NominalSpeed":
                self.NominalSpeed = d[key]
            elif key == "NominalAltitude":
                self.NominalAltitude = d[key]
            elif key == "NominalAltitudeType":
                self.NominalAltitudeType = d[key]
            elif key == "PayloadConfigurationList":
                self.PayloadConfigurationList = []
                for c in d[key]:
                    obj = seriesFactory.unpackFromDict(c)
                    if obj != None:
                        self.PayloadConfigurationList.append(obj)
            elif key == "Info":
                self.Info = []
                for c in d[key]:
                    obj = seriesFactory.unpackFromDict(c)
                    if obj != None:
                        self.Info.append(obj)

        return

    def get_ID(self):
        return self.ID

    def set_ID(self, value):
        self.ID = int( value )

    def get_Affiliation(self):
        return self.Affiliation

    def set_Affiliation(self, value):
        self.Affiliation = str( value )

    def get_EntityType(self):
        return self.EntityType

    def set_EntityType(self, value):
        self.EntityType = str( value )

    def get_Label(self):
        return self.Label

    def set_Label(self, value):
        self.Label = str( value )

    def get_NominalSpeed(self):
        return self.NominalSpeed

    def set_NominalSpeed(self, value):
        self.NominalSpeed = float( value )

    def get_NominalAltitude(self):
        return self.NominalAltitude

    def set_NominalAltitude(self, value):
        self.NominalAltitude = float( value )

    def get_NominalAltitudeType(self):
        return self.NominalAltitudeType

    def set_NominalAltitudeType(self, value):
        self.NominalAltitudeType = value 

    def get_PayloadConfigurationList(self):
        return self.PayloadConfigurationList

    def get_Info(self):
        return self.Info



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = LMCPObject.LMCPObject.toString(self)
        buf += "From EntityConfiguration:\n"
        buf +=    "ID = " + str( self.ID ) + "\n" 
        buf +=    "Affiliation = " + str( self.Affiliation ) + "\n" 
        buf +=    "EntityType = " + str( self.EntityType ) + "\n" 
        buf +=    "Label = " + str( self.Label ) + "\n" 
        buf +=    "NominalSpeed = " + str( self.NominalSpeed ) + "\n" 
        buf +=    "NominalAltitude = " + str( self.NominalAltitude ) + "\n" 
        buf +=    "NominalAltitudeType = " + str( self.NominalAltitudeType ) + "\n" 
        buf +=    "PayloadConfigurationList = " + str( self.PayloadConfigurationList ) + "\n" 
        buf +=    "Info = " + str( self.Info ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "CMASI") or (len("CMASI") == 0): # this should never happen
        	# Checks for "CMASI" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/EntityConfiguration")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("CMASI" + "/EntityConfiguration")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        LMCPObject.LMCPObject.toDictMembers(self, d)
        d['ID'] = self.ID
        d['Affiliation'] = self.Affiliation
        d['EntityType'] = self.EntityType
        d['Label'] = self.Label
        d['NominalSpeed'] = self.NominalSpeed
        d['NominalAltitude'] = self.NominalAltitude
        d['NominalAltitudeType'] = self.NominalAltitudeType
        d['PayloadConfigurationList'] = []
        for x in self.PayloadConfigurationList:
            if x == None:
                d['PayloadConfigurationList'].append(None)
            else:
                d['PayloadConfigurationList'].append(x.toDict())
        d['Info'] = []
        for x in self.Info:
            if x == None:
                d['Info'].append(None)
            else:
                d['Info'].append(x.toDict())

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
        str = ws + '<EntityConfiguration Series="CMASI" >\n';
        #str +=LMCPObject.LMCPObject.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</EntityConfiguration>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += LMCPObject.LMCPObject.toXMLMembersStr(self, ws)
        buf += ws + "<ID>" + str(self.ID) + "</ID>\n"
        buf += ws + "<Affiliation>" + str(self.Affiliation) + "</Affiliation>\n"
        buf += ws + "<EntityType>" + str(self.EntityType) + "</EntityType>\n"
        buf += ws + "<Label>" + str(self.Label) + "</Label>\n"
        buf += ws + "<NominalSpeed>" + str(self.NominalSpeed) + "</NominalSpeed>\n"
        buf += ws + "<NominalAltitude>" + str(self.NominalAltitude) + "</NominalAltitude>\n"
        buf += ws + "<NominalAltitudeType>" + AltitudeType.get_AltitudeType_int(self.NominalAltitudeType) + "</NominalAltitudeType>\n"
        buf += ws + "<PayloadConfigurationList>\n"
        for x in self.PayloadConfigurationList:
            if x == None:
                buf += ws + "    <null/>\n"
            else:
                buf += x.toXMLStr(ws + "    ") 
        buf += ws + "</PayloadConfigurationList>\n"
        buf += ws + "<Info>\n"
        for x in self.Info:
            if x == None:
                buf += ws + "    <null/>\n"
            else:
                buf += x.toXMLStr(ws + "    ") 
        buf += ws + "</Info>\n"

        return buf
        
