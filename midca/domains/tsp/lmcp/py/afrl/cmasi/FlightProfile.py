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



class FlightProfile(LMCPObject.LMCPObject):

    def __init__(self):

        self.LMCP_TYPE = 12
        self.SERIES_NAME = "CMASI"
        self.FULL_LMCP_TYPE_NAME = "afrl.cmasi.FlightProfile"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 4849604199710720000
        self.SERIES_VERSION = 3

        #Define message fields
        self.Name = ""   #string
        self.Airspeed = 0   #real32
        self.PitchAngle = 0   #real32
        self.VerticalSpeed = 0   #real32
        self.MaxBankAngle = 0   #real32
        self.EnergyRate = 0   #real32


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(LMCPObject.LMCPObject.pack(self))
        buffer.extend(struct.pack(">H", len(self.Name) ))
        if len(self.Name) > 0:
            if (sys.version_info > (3, 0)):
                buffer.extend(struct.pack( repr(len(self.Name)) + "s", self.Name.encode('utf-8')))
            else:
                buffer.extend(struct.pack( repr(len(self.Name)) + "s", self.Name))
        buffer.extend(struct.pack(">f", self.Airspeed))
        buffer.extend(struct.pack(">f", self.PitchAngle))
        buffer.extend(struct.pack(">f", self.VerticalSpeed))
        buffer.extend(struct.pack(">f", self.MaxBankAngle))
        buffer.extend(struct.pack(">f", self.EnergyRate))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = LMCPObject.LMCPObject.unpack(self, buffer, _pos)
        _strlen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        if _strlen > 0:
            if (sys.version_info > (3, 0)):
                self.Name = struct.unpack_from( repr(_strlen) + "s", buffer, _pos )[0].decode('utf-8')
            else:
                 self.Name = struct.unpack_from( repr(_strlen) + "s", buffer, _pos )[0]
            _pos += _strlen
        else:
             self.Name = ""
        self.Airspeed = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.PitchAngle = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.VerticalSpeed = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.MaxBankAngle = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.EnergyRate = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        LMCPObject.LMCPObject.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "Name" :
                    self.Name = str(e.childNodes[0].nodeValue) if len(e.childNodes) > 0 else ""
                elif e.localName == "Airspeed" and len(e.childNodes) > 0 :
                    self.Airspeed = float(e.childNodes[0].nodeValue)
                elif e.localName == "PitchAngle" and len(e.childNodes) > 0 :
                    self.PitchAngle = float(e.childNodes[0].nodeValue)
                elif e.localName == "VerticalSpeed" and len(e.childNodes) > 0 :
                    self.VerticalSpeed = float(e.childNodes[0].nodeValue)
                elif e.localName == "MaxBankAngle" and len(e.childNodes) > 0 :
                    self.MaxBankAngle = float(e.childNodes[0].nodeValue)
                elif e.localName == "EnergyRate" and len(e.childNodes) > 0 :
                    self.EnergyRate = float(e.childNodes[0].nodeValue)

        return

    def unpackFromDict(self, d, seriesFactory):
        LMCPObject.LMCPObject.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "Name":
                self.Name = d[key]
            elif key == "Airspeed":
                self.Airspeed = d[key]
            elif key == "PitchAngle":
                self.PitchAngle = d[key]
            elif key == "VerticalSpeed":
                self.VerticalSpeed = d[key]
            elif key == "MaxBankAngle":
                self.MaxBankAngle = d[key]
            elif key == "EnergyRate":
                self.EnergyRate = d[key]

        return

    def get_Name(self):
        return self.Name

    def set_Name(self, value):
        self.Name = str( value )

    def get_Airspeed(self):
        return self.Airspeed

    def set_Airspeed(self, value):
        self.Airspeed = float( value )

    def get_PitchAngle(self):
        return self.PitchAngle

    def set_PitchAngle(self, value):
        self.PitchAngle = float( value )

    def get_VerticalSpeed(self):
        return self.VerticalSpeed

    def set_VerticalSpeed(self, value):
        self.VerticalSpeed = float( value )

    def get_MaxBankAngle(self):
        return self.MaxBankAngle

    def set_MaxBankAngle(self, value):
        self.MaxBankAngle = float( value )

    def get_EnergyRate(self):
        return self.EnergyRate

    def set_EnergyRate(self, value):
        self.EnergyRate = float( value )



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = LMCPObject.LMCPObject.toString(self)
        buf += "From FlightProfile:\n"
        buf +=    "Name = " + str( self.Name ) + "\n" 
        buf +=    "Airspeed = " + str( self.Airspeed ) + "\n" 
        buf +=    "PitchAngle = " + str( self.PitchAngle ) + "\n" 
        buf +=    "VerticalSpeed = " + str( self.VerticalSpeed ) + "\n" 
        buf +=    "MaxBankAngle = " + str( self.MaxBankAngle ) + "\n" 
        buf +=    "EnergyRate = " + str( self.EnergyRate ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "CMASI") or (len("CMASI") == 0): # this should never happen
        	# Checks for "CMASI" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/FlightProfile")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("CMASI" + "/FlightProfile")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        LMCPObject.LMCPObject.toDictMembers(self, d)
        d['Name'] = self.Name
        d['Airspeed'] = self.Airspeed
        d['PitchAngle'] = self.PitchAngle
        d['VerticalSpeed'] = self.VerticalSpeed
        d['MaxBankAngle'] = self.MaxBankAngle
        d['EnergyRate'] = self.EnergyRate

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
        str = ws + '<FlightProfile Series="CMASI" >\n';
        #str +=LMCPObject.LMCPObject.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</FlightProfile>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += LMCPObject.LMCPObject.toXMLMembersStr(self, ws)
        buf += ws + "<Name>" + str(self.Name) + "</Name>\n"
        buf += ws + "<Airspeed>" + str(self.Airspeed) + "</Airspeed>\n"
        buf += ws + "<PitchAngle>" + str(self.PitchAngle) + "</PitchAngle>\n"
        buf += ws + "<VerticalSpeed>" + str(self.VerticalSpeed) + "</VerticalSpeed>\n"
        buf += ws + "<MaxBankAngle>" + str(self.MaxBankAngle) + "</MaxBankAngle>\n"
        buf += ws + "<EnergyRate>" + str(self.EnergyRate) + "</EnergyRate>\n"

        return buf
        
