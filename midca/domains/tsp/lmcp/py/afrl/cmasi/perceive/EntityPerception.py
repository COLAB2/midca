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


class EntityPerception(LMCPObject.LMCPObject):

    def __init__(self):

        self.LMCP_TYPE = 1
        self.SERIES_NAME = "PERCEIVE"
        self.FULL_LMCP_TYPE_NAME = "afrl.cmasi.perceive.EntityPerception"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 5784119745305990725
        self.SERIES_VERSION = 1

        #Define message fields
        self.PerceivedEntityID = 0   #uint32
        self.PerceiverID = 0   #uint32
        self.PerceiverPayloads = []   #uint32
        self.Velocity = [0]*3   #real32
        self.VelocityError = [0]*3   #real32
        self.VelocityValid = False   #bool
        self.Attitude = [0]*3   #real32
        self.AttitudeError = [0]*3   #real32
        self.AttitudeValid = False   #bool
        self.Location = Location3D.Location3D()   #Location3D
        self.LocationError = [0]*3   #real32
        self.TimeLastSeen = 0   #int64


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(LMCPObject.LMCPObject.pack(self))
        buffer.extend(struct.pack(">I", self.PerceivedEntityID))
        buffer.extend(struct.pack(">I", self.PerceiverID))
        buffer.extend(struct.pack(">H", len(self.PerceiverPayloads) ))
        for x in self.PerceiverPayloads:
            buffer.extend(struct.pack(">I", x ))
        for x in self.Velocity:
            buffer.extend(struct.pack(">f", x ))
        for x in self.VelocityError:
            buffer.extend(struct.pack(">f", x ))
        boolChar = 1 if self.VelocityValid == True else 0
        buffer.extend(struct.pack(">B",boolChar))
        for x in self.Attitude:
            buffer.extend(struct.pack(">f", x ))
        for x in self.AttitudeError:
            buffer.extend(struct.pack(">f", x ))
        boolChar = 1 if self.AttitudeValid == True else 0
        buffer.extend(struct.pack(">B",boolChar))
        buffer.extend(struct.pack("B", self.Location != None ))
        if self.Location != None:
            buffer.extend(struct.pack(">q", self.Location.SERIES_NAME_ID))
            buffer.extend(struct.pack(">I", self.Location.LMCP_TYPE))
            buffer.extend(struct.pack(">H", self.Location.SERIES_VERSION))
            buffer.extend(self.Location.pack())
        for x in self.LocationError:
            buffer.extend(struct.pack(">f", x ))
        buffer.extend(struct.pack(">q", self.TimeLastSeen))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = LMCPObject.LMCPObject.unpack(self, buffer, _pos)
        self.PerceivedEntityID = struct.unpack_from(">I", buffer, _pos)[0]
        _pos += 4
        self.PerceiverID = struct.unpack_from(">I", buffer, _pos)[0]
        _pos += 4
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.PerceiverPayloads = [None] * _arraylen
        if _arraylen > 0:
            self.PerceiverPayloads = struct.unpack_from(">" + repr(_arraylen) + "I", buffer, _pos )
            _pos += 4 * _arraylen
        self.Velocity = [None] * 3
        _arraylen = 3
        if _arraylen > 0:
            self.Velocity = struct.unpack_from(">" + repr(_arraylen) + "f", buffer, _pos )
            _pos += 4 * _arraylen
        self.VelocityError = [None] * 3
        _arraylen = 3
        if _arraylen > 0:
            self.VelocityError = struct.unpack_from(">" + repr(_arraylen) + "f", buffer, _pos )
            _pos += 4 * _arraylen
        boolChar = struct.unpack_from(">B", buffer, _pos)[0]
        self.VelocityValid = True if boolChar == 1 else False
        _pos += 1
        self.Attitude = [None] * 3
        _arraylen = 3
        if _arraylen > 0:
            self.Attitude = struct.unpack_from(">" + repr(_arraylen) + "f", buffer, _pos )
            _pos += 4 * _arraylen
        self.AttitudeError = [None] * 3
        _arraylen = 3
        if _arraylen > 0:
            self.AttitudeError = struct.unpack_from(">" + repr(_arraylen) + "f", buffer, _pos )
            _pos += 4 * _arraylen
        boolChar = struct.unpack_from(">B", buffer, _pos)[0]
        self.AttitudeValid = True if boolChar == 1 else False
        _pos += 1
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
            self.Location = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
            _pos = self.Location.unpack(buffer, _pos)
        else:
            self.Location = None
        self.LocationError = [None] * 3
        _arraylen = 3
        if _arraylen > 0:
            self.LocationError = struct.unpack_from(">" + repr(_arraylen) + "f", buffer, _pos )
            _pos += 4 * _arraylen
        self.TimeLastSeen = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        LMCPObject.LMCPObject.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "PerceivedEntityID" and len(e.childNodes) > 0 :
                    self.PerceivedEntityID = int(e.childNodes[0].nodeValue)
                elif e.localName == "PerceiverID" and len(e.childNodes) > 0 :
                    self.PerceiverID = int(e.childNodes[0].nodeValue)
                elif e.localName == "PerceiverPayloads" and len(e.childNodes) > 0 :
                    self.PerceiverPayloads = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.PerceiverPayloads.append( int(c.childNodes[0].nodeValue) )
                elif e.localName == "Velocity" and len(e.childNodes) > 0 :
                    self.Velocity = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.Velocity.append( float(c.childNodes[0].nodeValue) )
                elif e.localName == "VelocityError" and len(e.childNodes) > 0 :
                    self.VelocityError = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.VelocityError.append( float(c.childNodes[0].nodeValue) )
                elif e.localName == "VelocityValid" and len(e.childNodes) > 0 :
                    self.VelocityValid = e.childNodes[0].nodeValue.lower() == 'true'
                elif e.localName == "Attitude" and len(e.childNodes) > 0 :
                    self.Attitude = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.Attitude.append( float(c.childNodes[0].nodeValue) )
                elif e.localName == "AttitudeError" and len(e.childNodes) > 0 :
                    self.AttitudeError = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.AttitudeError.append( float(c.childNodes[0].nodeValue) )
                elif e.localName == "AttitudeValid" and len(e.childNodes) > 0 :
                    self.AttitudeValid = e.childNodes[0].nodeValue.lower() == 'true'
                elif e.localName == "Location" and len(e.childNodes) > 0 :
                    for n in e.childNodes:
                        if n.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.Location = seriesFactory.createObjectByName(n.getAttribute('Series'), n.localName)
                            if self.Location != None:
                                self.Location.unpackFromXMLNode(n, seriesFactory)
                elif e.localName == "LocationError" and len(e.childNodes) > 0 :
                    self.LocationError = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.LocationError.append( float(c.childNodes[0].nodeValue) )
                elif e.localName == "TimeLastSeen" and len(e.childNodes) > 0 :
                    self.TimeLastSeen = int(e.childNodes[0].nodeValue)

        return

    def unpackFromDict(self, d, seriesFactory):
        LMCPObject.LMCPObject.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "PerceivedEntityID":
                self.PerceivedEntityID = d[key]
            elif key == "PerceiverID":
                self.PerceiverID = d[key]
            elif key == "PerceiverPayloads":
                self.PerceiverPayloads = []
                for c in d[key]:
                    self.PerceiverPayloads.append( c )
            elif key == "Velocity":
                self.Velocity = []
                for c in d[key]:
                    self.Velocity.append( c )
            elif key == "VelocityError":
                self.VelocityError = []
                for c in d[key]:
                    self.VelocityError.append( c )
            elif key == "VelocityValid":
                self.VelocityValid = d[key]
            elif key == "Attitude":
                self.Attitude = []
                for c in d[key]:
                    self.Attitude.append( c )
            elif key == "AttitudeError":
                self.AttitudeError = []
                for c in d[key]:
                    self.AttitudeError.append( c )
            elif key == "AttitudeValid":
                self.AttitudeValid = d[key]
            elif key == "Location":
                self.Location = seriesFactory.unpackFromDict(d[key])
            elif key == "LocationError":
                self.LocationError = []
                for c in d[key]:
                    self.LocationError.append( c )
            elif key == "TimeLastSeen":
                self.TimeLastSeen = d[key]

        return

    def get_PerceivedEntityID(self):
        return self.PerceivedEntityID

    def set_PerceivedEntityID(self, value):
        self.PerceivedEntityID = int( value )

    def get_PerceiverID(self):
        return self.PerceiverID

    def set_PerceiverID(self, value):
        self.PerceiverID = int( value )

    def get_PerceiverPayloads(self):
        return self.PerceiverPayloads

    def get_Velocity(self):
        return self.Velocity

    def get_VelocityError(self):
        return self.VelocityError

    def get_VelocityValid(self):
        return self.VelocityValid

    def set_VelocityValid(self, value):
        self.VelocityValid = bool( value )

    def get_Attitude(self):
        return self.Attitude

    def get_AttitudeError(self):
        return self.AttitudeError

    def get_AttitudeValid(self):
        return self.AttitudeValid

    def set_AttitudeValid(self, value):
        self.AttitudeValid = bool( value )

    def get_Location(self):
        return self.Location

    def set_Location(self, value):
        self.Location = value 

    def get_LocationError(self):
        return self.LocationError

    def get_TimeLastSeen(self):
        return self.TimeLastSeen

    def set_TimeLastSeen(self, value):
        self.TimeLastSeen = int( value )



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = LMCPObject.LMCPObject.toString(self)
        buf += "From EntityPerception:\n"
        buf +=    "PerceivedEntityID = " + str( self.PerceivedEntityID ) + "\n" 
        buf +=    "PerceiverID = " + str( self.PerceiverID ) + "\n" 
        buf +=    "PerceiverPayloads = " + str( self.PerceiverPayloads ) + "\n" 
        buf +=    "Velocity = " + str( self.Velocity ) + "\n" 
        buf +=    "VelocityError = " + str( self.VelocityError ) + "\n" 
        buf +=    "VelocityValid = " + str( self.VelocityValid ) + "\n" 
        buf +=    "Attitude = " + str( self.Attitude ) + "\n" 
        buf +=    "AttitudeError = " + str( self.AttitudeError ) + "\n" 
        buf +=    "AttitudeValid = " + str( self.AttitudeValid ) + "\n" 
        buf +=    "Location = " + str( self.Location ) + "\n" 
        buf +=    "LocationError = " + str( self.LocationError ) + "\n" 
        buf +=    "TimeLastSeen = " + str( self.TimeLastSeen ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "PERCEIVE") or (len("PERCEIVE") == 0): # this should never happen
        	# Checks for "PERCEIVE" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/EntityPerception")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("PERCEIVE" + "/EntityPerception")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        LMCPObject.LMCPObject.toDictMembers(self, d)
        d['PerceivedEntityID'] = self.PerceivedEntityID
        d['PerceiverID'] = self.PerceiverID
        d['PerceiverPayloads'] = []
        for x in self.PerceiverPayloads:
            d['PerceiverPayloads'].append(x)
        d['Velocity'] = []
        for x in self.Velocity:
            d['Velocity'].append(x)
        d['VelocityError'] = []
        for x in self.VelocityError:
            d['VelocityError'].append(x)
        d['VelocityValid'] = self.VelocityValid
        d['Attitude'] = []
        for x in self.Attitude:
            d['Attitude'].append(x)
        d['AttitudeError'] = []
        for x in self.AttitudeError:
            d['AttitudeError'].append(x)
        d['AttitudeValid'] = self.AttitudeValid
        if self.Location == None:
            d['Location'] = None
        else:
            d['Location'] = self.Location.toDict()
        d['LocationError'] = []
        for x in self.LocationError:
            d['LocationError'].append(x)
        d['TimeLastSeen'] = self.TimeLastSeen

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
        str = ws + '<EntityPerception Series="PERCEIVE" >\n';
        #str +=LMCPObject.LMCPObject.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</EntityPerception>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += LMCPObject.LMCPObject.toXMLMembersStr(self, ws)
        buf += ws + "<PerceivedEntityID>" + str(self.PerceivedEntityID) + "</PerceivedEntityID>\n"
        buf += ws + "<PerceiverID>" + str(self.PerceiverID) + "</PerceiverID>\n"
        buf += ws + "<PerceiverPayloads>\n"
        for x in self.PerceiverPayloads:
            buf += ws + "<uint32>" + str(x) + "</uint32>\n"
        buf += ws + "</PerceiverPayloads>\n"
        buf += ws + "<Velocity>\n"
        for x in self.Velocity:
            buf += ws + "<real32>" + str(x) + "</real32>\n"
        buf += ws + "</Velocity>\n"
        buf += ws + "<VelocityError>\n"
        for x in self.VelocityError:
            buf += ws + "<real32>" + str(x) + "</real32>\n"
        buf += ws + "</VelocityError>\n"
        buf += ws + "<VelocityValid>" + ('True' if self.VelocityValid else 'False') + "</VelocityValid>\n"
        buf += ws + "<Attitude>\n"
        for x in self.Attitude:
            buf += ws + "<real32>" + str(x) + "</real32>\n"
        buf += ws + "</Attitude>\n"
        buf += ws + "<AttitudeError>\n"
        for x in self.AttitudeError:
            buf += ws + "<real32>" + str(x) + "</real32>\n"
        buf += ws + "</AttitudeError>\n"
        buf += ws + "<AttitudeValid>" + ('True' if self.AttitudeValid else 'False') + "</AttitudeValid>\n"
        if self.Location != None:
            buf += ws + "<Location>\n"
            buf += ws + self.Location.toXMLStr(ws + "    ") 
            buf += ws + "</Location>\n"
        buf += ws + "<LocationError>\n"
        for x in self.LocationError:
            buf += ws + "<real32>" + str(x) + "</real32>\n"
        buf += ws + "</LocationError>\n"
        buf += ws + "<TimeLastSeen>" + str(self.TimeLastSeen) + "</TimeLastSeen>\n"

        return buf
        
