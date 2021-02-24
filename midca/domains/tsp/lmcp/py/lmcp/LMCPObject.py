
## ===============================================================================
## Authors: AFRL/RQQA
## Organization: Air Force Research Laboratory, Aerospace Systems Directorate, Power and Control Division
## 
## Copyright (c) 2017 Government of the United State of America, as represented by
## the Secretary of the Air Force.  No copyright is claimed in the United States under
## Title 17, U.S. Code.  All Other Rights Reserved.
## ===============================================================================

## This file was auto-created by LmcpGen. Modifications will be overwritten.

class LMCPObject:

    SERIES_NAME = ""
    LMCP_TYPE = 0
    FULL_LMCP_TYPE_NAME = "avtas.lmcp.Object"

    def pack(self):
        return bytearray()

    def unpack(self, buffer, _pos):
        return _pos

    def unpackFromXMLNode(self, el, seriesFactory):
        return

    def unpackFromDict(self, d, seriesFactory):
        return

    def toString(self):
        return ""

    def toDict(self):
        d = {}
        self.toDictMembers(d)
        return d

    def toDictMembers(self, d):
        return

    def toXML(self):
        return self.toXMLStr("");

    def toXMLStr(self, ws):
        return ""
    
    def toXMLMembersStr(self, ws):
        """
        Returns an XML String of all of the members.  Does not include open and closing object
        name.  This should be used by toXML() to get inherited members.
        """
        return ""

    def unpackFromXMLNode(self, el, seriesFactory):
        """
        Extracts members from a DOM Element.  Expects a minidom Element (e) and a
        SeriesFactory object
        """
        return
