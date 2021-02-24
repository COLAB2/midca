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

from midca.domains.tsp.lmcp.py.afrl.cmasi import KeyValuePair


class Task(LMCPObject.LMCPObject):

    def __init__(self):

        self.LMCP_TYPE = 8
        self.SERIES_NAME = "CMASI"
        self.FULL_LMCP_TYPE_NAME = "afrl.cmasi.Task"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 4849604199710720000
        self.SERIES_VERSION = 3

        #Define message fields
        self.TaskID = 0   #int64
        self.Label = ""   #string
        self.EligibleEntities = []   #int64
        self.RevisitRate = 0   #real32
        self.Parameters = []   #KeyValuePair
        self.Priority = 0   #byte
        self.Required = True   #bool


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(LMCPObject.LMCPObject.pack(self))
        buffer.extend(struct.pack(">q", self.TaskID))
        buffer.extend(struct.pack(">H", len(self.Label) ))
        if len(self.Label) > 0:
            if (sys.version_info > (3, 0)):
                buffer.extend(struct.pack( repr(len(self.Label)) + "s", self.Label.encode('utf-8')))
            else:
                buffer.extend(struct.pack( repr(len(self.Label)) + "s", self.Label))
        buffer.extend(struct.pack(">H", len(self.EligibleEntities) ))
        for x in self.EligibleEntities:
            buffer.extend(struct.pack(">q", x ))
        buffer.extend(struct.pack(">f", self.RevisitRate))
        buffer.extend(struct.pack(">H", len(self.Parameters) ))
        for x in self.Parameters:
           buffer.extend(struct.pack("B", x != None ))
           if x != None:
               buffer.extend(struct.pack(">q", x.SERIES_NAME_ID))
               buffer.extend(struct.pack(">I", x.LMCP_TYPE))
               buffer.extend(struct.pack(">H", x.SERIES_VERSION))
               buffer.extend(x.pack())
        buffer.extend(struct.pack(">B", self.Priority))
        boolChar = 1 if self.Required == True else 0
        buffer.extend(struct.pack(">B",boolChar))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = LMCPObject.LMCPObject.unpack(self, buffer, _pos)
        self.TaskID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
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
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.EligibleEntities = [None] * _arraylen
        if _arraylen > 0:
            self.EligibleEntities = struct.unpack_from(">" + repr(_arraylen) + "q", buffer, _pos )
            _pos += 8 * _arraylen
        self.RevisitRate = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.Parameters = [None] * _arraylen
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
                self.Parameters[x] = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
                _pos = self.Parameters[x].unpack(buffer, _pos)
            else:
                self.Parameters[x] = None
        self.Priority = struct.unpack_from(">B", buffer, _pos)[0]
        _pos += 1
        boolChar = struct.unpack_from(">B", buffer, _pos)[0]
        self.Required = True if boolChar == 1 else False
        _pos += 1
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        LMCPObject.LMCPObject.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "TaskID" and len(e.childNodes) > 0 :
                    self.TaskID = int(e.childNodes[0].nodeValue)
                elif e.localName == "Label" :
                    self.Label = str(e.childNodes[0].nodeValue) if len(e.childNodes) > 0 else ""
                elif e.localName == "EligibleEntities" and len(e.childNodes) > 0 :
                    self.EligibleEntities = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.EligibleEntities.append( int(c.childNodes[0].nodeValue) )
                elif e.localName == "RevisitRate" and len(e.childNodes) > 0 :
                    self.RevisitRate = float(e.childNodes[0].nodeValue)
                elif e.localName == "Parameters" and len(e.childNodes) > 0 :
                    self.Parameters = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            obj = seriesFactory.createObjectByName(c.getAttribute('Series'), c.localName)
                            if obj != None:
                                obj.unpackFromXMLNode(c, seriesFactory)
                                self.Parameters.append(obj)
                elif e.localName == "Priority" and len(e.childNodes) > 0 :
                    self.Priority = int(e.childNodes[0].nodeValue)
                elif e.localName == "Required" and len(e.childNodes) > 0 :
                    self.Required = e.childNodes[0].nodeValue.lower() == 'true'

        return

    def unpackFromDict(self, d, seriesFactory):
        LMCPObject.LMCPObject.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "TaskID":
                self.TaskID = d[key]
            elif key == "Label":
                self.Label = d[key]
            elif key == "EligibleEntities":
                self.EligibleEntities = []
                for c in d[key]:
                    self.EligibleEntities.append( c )
            elif key == "RevisitRate":
                self.RevisitRate = d[key]
            elif key == "Parameters":
                self.Parameters = []
                for c in d[key]:
                    obj = seriesFactory.unpackFromDict(c)
                    if obj != None:
                        self.Parameters.append(obj)
            elif key == "Priority":
                self.Priority = d[key]
            elif key == "Required":
                self.Required = d[key]

        return

    def get_TaskID(self):
        return self.TaskID

    def set_TaskID(self, value):
        self.TaskID = int( value )

    def get_Label(self):
        return self.Label

    def set_Label(self, value):
        self.Label = str( value )

    def get_EligibleEntities(self):
        return self.EligibleEntities

    def get_RevisitRate(self):
        return self.RevisitRate

    def set_RevisitRate(self, value):
        self.RevisitRate = float( value )

    def get_Parameters(self):
        return self.Parameters

    def get_Priority(self):
        return self.Priority

    def set_Priority(self, value):
        self.Priority = int( value )

    def get_Required(self):
        return self.Required

    def set_Required(self, value):
        self.Required = bool( value )



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = LMCPObject.LMCPObject.toString(self)
        buf += "From Task:\n"
        buf +=    "TaskID = " + str( self.TaskID ) + "\n" 
        buf +=    "Label = " + str( self.Label ) + "\n" 
        buf +=    "EligibleEntities = " + str( self.EligibleEntities ) + "\n" 
        buf +=    "RevisitRate = " + str( self.RevisitRate ) + "\n" 
        buf +=    "Parameters = " + str( self.Parameters ) + "\n" 
        buf +=    "Priority = " + str( self.Priority ) + "\n" 
        buf +=    "Required = " + str( self.Required ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "CMASI") or (len("CMASI") == 0): # this should never happen
        	# Checks for "CMASI" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/Task")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("CMASI" + "/Task")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        LMCPObject.LMCPObject.toDictMembers(self, d)
        d['TaskID'] = self.TaskID
        d['Label'] = self.Label
        d['EligibleEntities'] = []
        for x in self.EligibleEntities:
            d['EligibleEntities'].append(x)
        d['RevisitRate'] = self.RevisitRate
        d['Parameters'] = []
        for x in self.Parameters:
            if x == None:
                d['Parameters'].append(None)
            else:
                d['Parameters'].append(x.toDict())
        d['Priority'] = self.Priority
        d['Required'] = self.Required

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
        str = ws + '<Task Series="CMASI" >\n';
        #str +=LMCPObject.LMCPObject.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</Task>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += LMCPObject.LMCPObject.toXMLMembersStr(self, ws)
        buf += ws + "<TaskID>" + str(self.TaskID) + "</TaskID>\n"
        buf += ws + "<Label>" + str(self.Label) + "</Label>\n"
        buf += ws + "<EligibleEntities>\n"
        for x in self.EligibleEntities:
            buf += ws + "<int64>" + str(x) + "</int64>\n"
        buf += ws + "</EligibleEntities>\n"
        buf += ws + "<RevisitRate>" + str(self.RevisitRate) + "</RevisitRate>\n"
        buf += ws + "<Parameters>\n"
        for x in self.Parameters:
            if x == None:
                buf += ws + "    <null/>\n"
            else:
                buf += x.toXMLStr(ws + "    ") 
        buf += ws + "</Parameters>\n"
        buf += ws + "<Priority>" + str(self.Priority) + "</Priority>\n"
        buf += ws + "<Required>" + ('True' if self.Required else 'False') + "</Required>\n"

        return buf
        
