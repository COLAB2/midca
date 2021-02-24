from midca.domains.tsp.lmcp.py.lmcp.LMCPObject import *

## ===============================================================================
## Authors: AFRL/RQQA
## Organization: Air Force Research Laboratory, Aerospace Systems Directorate, Power and Control Division
## 
## Copyright (c) 2017 Government of the United State of America, as represented by
## the Secretary of the Air Force.  No copyright is claimed in the United States under
## Title 17, U.S. Code.  All Other Rights Reserved.
## ===============================================================================

## This file was auto-created by LmcpGen. Modifications will be overwritten.

from midca.domains.tsp.lmcp.py.uxas.messages.uxnative import VideoRecord
from midca.domains.tsp.lmcp.py.uxas.messages.uxnative import StartupComplete
from midca.domains.tsp.lmcp.py.uxas.messages.uxnative import CreateNewService
from midca.domains.tsp.lmcp.py.uxas.messages.uxnative import KillService
from midca.domains.tsp.lmcp.py.uxas.messages.uxnative import IncrementWaypoint
from midca.domains.tsp.lmcp.py.uxas.messages.uxnative import SafeHeadingAction
from midca.domains.tsp.lmcp.py.uxas.messages.uxnative import EntityLocation
from midca.domains.tsp.lmcp.py.uxas.messages.uxnative import BandwidthTest
from midca.domains.tsp.lmcp.py.uxas.messages.uxnative import BandwidthReceiveReport
from midca.domains.tsp.lmcp.py.uxas.messages.uxnative import SubTaskExecution
from midca.domains.tsp.lmcp.py.uxas.messages.uxnative import SubTaskAssignment
from midca.domains.tsp.lmcp.py.uxas.messages.uxnative import AutopilotKeepAlive
from midca.domains.tsp.lmcp.py.uxas.messages.uxnative import OnboardStatusReport
from midca.domains.tsp.lmcp.py.uxas.messages.uxnative import EntityJoin
from midca.domains.tsp.lmcp.py.uxas.messages.uxnative import EntityExit
from midca.domains.tsp.lmcp.py.uxas.messages.uxnative import SimulationTimeStepAcknowledgement
from midca.domains.tsp.lmcp.py.uxas.messages.uxnative import SpeedOverrideAction


SERIES_NAME = "UXNATIVE"
#Series Name turned into a long for quick comparisons.
SERIES_NAME_ID = 6149751333668345413
SERIES_VERSION = 9


class SeriesEnum:

    def getName(self, type_):
        if(type_ ==  1): return "VideoRecord"
        if(type_ ==  2): return "StartupComplete"
        if(type_ ==  3): return "CreateNewService"
        if(type_ ==  4): return "KillService"
        if(type_ ==  5): return "IncrementWaypoint"
        if(type_ ==  6): return "SafeHeadingAction"
        if(type_ ==  7): return "EntityLocation"
        if(type_ ==  8): return "BandwidthTest"
        if(type_ ==  9): return "BandwidthReceiveReport"
        if(type_ ==  10): return "SubTaskExecution"
        if(type_ ==  11): return "SubTaskAssignment"
        if(type_ ==  12): return "AutopilotKeepAlive"
        if(type_ ==  13): return "OnboardStatusReport"
        if(type_ ==  14): return "EntityJoin"
        if(type_ ==  15): return "EntityExit"
        if(type_ ==  16): return "SimulationTimeStepAcknowledgement"
        if(type_ ==  17): return "SpeedOverrideAction"


    def getType(self, name):
        if ( name == "VideoRecord"): return 1
        if ( name == "StartupComplete"): return 2
        if ( name == "CreateNewService"): return 3
        if ( name == "KillService"): return 4
        if ( name == "IncrementWaypoint"): return 5
        if ( name == "SafeHeadingAction"): return 6
        if ( name == "EntityLocation"): return 7
        if ( name == "BandwidthTest"): return 8
        if ( name == "BandwidthReceiveReport"): return 9
        if ( name == "SubTaskExecution"): return 10
        if ( name == "SubTaskAssignment"): return 11
        if ( name == "AutopilotKeepAlive"): return 12
        if ( name == "OnboardStatusReport"): return 13
        if ( name == "EntityJoin"): return 14
        if ( name == "EntityExit"): return 15
        if ( name == "SimulationTimeStepAcknowledgement"): return 16
        if ( name == "SpeedOverrideAction"): return 17

        return -1

    def getInstance(self, type_):
        if(type_ ==  1): return VideoRecord.VideoRecord()
        if(type_ ==  2): return StartupComplete.StartupComplete()
        if(type_ ==  3): return CreateNewService.CreateNewService()
        if(type_ ==  4): return KillService.KillService()
        if(type_ ==  5): return IncrementWaypoint.IncrementWaypoint()
        if(type_ ==  6): return SafeHeadingAction.SafeHeadingAction()
        if(type_ ==  7): return EntityLocation.EntityLocation()
        if(type_ ==  8): return BandwidthTest.BandwidthTest()
        if(type_ ==  9): return BandwidthReceiveReport.BandwidthReceiveReport()
        if(type_ ==  10): return SubTaskExecution.SubTaskExecution()
        if(type_ ==  11): return SubTaskAssignment.SubTaskAssignment()
        if(type_ ==  12): return AutopilotKeepAlive.AutopilotKeepAlive()
        if(type_ ==  13): return OnboardStatusReport.OnboardStatusReport()
        if(type_ ==  14): return EntityJoin.EntityJoin()
        if(type_ ==  15): return EntityExit.EntityExit()
        if(type_ ==  16): return SimulationTimeStepAcknowledgement.SimulationTimeStepAcknowledgement()
        if(type_ ==  17): return SpeedOverrideAction.SpeedOverrideAction()

        return None
