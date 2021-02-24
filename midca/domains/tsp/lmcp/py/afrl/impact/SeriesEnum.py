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

from midca.domains.tsp.lmcp.py.afrl.impact import PowerConfiguration
from midca.domains.tsp.lmcp.py.afrl.impact import RadioConfiguration
from midca.domains.tsp.lmcp.py.afrl.impact import RadioTowerConfiguration
from midca.domains.tsp.lmcp.py.afrl.impact import RadioState
from midca.domains.tsp.lmcp.py.afrl.impact import RadioTowerState
from midca.domains.tsp.lmcp.py.afrl.impact import ImpactPayloadConfiguration
from midca.domains.tsp.lmcp.py.afrl.impact import DeployImpactPayload
from midca.domains.tsp.lmcp.py.afrl.impact import PowerPlantState
from midca.domains.tsp.lmcp.py.afrl.impact import BatchRoutePlanRequest
from midca.domains.tsp.lmcp.py.afrl.impact import BatchRoutePlanResponse
from midca.domains.tsp.lmcp.py.afrl.impact import TaskTimingPair
from midca.domains.tsp.lmcp.py.afrl.impact import BatchSummaryRequest
from midca.domains.tsp.lmcp.py.afrl.impact import BatchSummaryResponse
from midca.domains.tsp.lmcp.py.afrl.impact import TaskSummary
from midca.domains.tsp.lmcp.py.afrl.impact import VehicleSummary
from midca.domains.tsp.lmcp.py.afrl.impact import SpeedAltPair
from midca.domains.tsp.lmcp.py.afrl.impact import ImpactAutomationRequest
from midca.domains.tsp.lmcp.py.afrl.impact import ImpactAutomationResponse
from midca.domains.tsp.lmcp.py.afrl.impact import PointOfInterest
from midca.domains.tsp.lmcp.py.afrl.impact import LineOfInterest
from midca.domains.tsp.lmcp.py.afrl.impact import AreaOfInterest
from midca.domains.tsp.lmcp.py.afrl.impact import ImpactPointSearchTask
from midca.domains.tsp.lmcp.py.afrl.impact import PatternSearchTask
from midca.domains.tsp.lmcp.py.afrl.impact import AngledAreaSearchTask
from midca.domains.tsp.lmcp.py.afrl.impact import ImpactLineSearchTask
from midca.domains.tsp.lmcp.py.afrl.impact import WatchTask
from midca.domains.tsp.lmcp.py.afrl.impact import MultiVehicleWatchTask
from midca.domains.tsp.lmcp.py.afrl.impact import CommRelayTask
from midca.domains.tsp.lmcp.py.afrl.impact import CordonTask
from midca.domains.tsp.lmcp.py.afrl.impact import BlockadeTask
from midca.domains.tsp.lmcp.py.afrl.impact import EscortTask
from midca.domains.tsp.lmcp.py.afrl.impact import ConfigurationRequest
from midca.domains.tsp.lmcp.py.afrl.impact import WaterReport
from midca.domains.tsp.lmcp.py.afrl.impact import WaterZone
from midca.domains.tsp.lmcp.py.afrl.impact import PayloadDropTask


SERIES_NAME = "IMPACT"
#Series Name turned into a long for quick comparisons.
SERIES_NAME_ID = 5281966179208134656
SERIES_VERSION = 14


class SeriesEnum:

    def getName(self, type_):
        if(type_ ==  1): return "PowerConfiguration"
        if(type_ ==  2): return "RadioConfiguration"
        if(type_ ==  3): return "RadioTowerConfiguration"
        if(type_ ==  4): return "RadioState"
        if(type_ ==  5): return "RadioTowerState"
        if(type_ ==  6): return "ImpactPayloadConfiguration"
        if(type_ ==  7): return "DeployImpactPayload"
        if(type_ ==  8): return "PowerPlantState"
        if(type_ ==  9): return "BatchRoutePlanRequest"
        if(type_ ==  10): return "BatchRoutePlanResponse"
        if(type_ ==  11): return "TaskTimingPair"
        if(type_ ==  12): return "BatchSummaryRequest"
        if(type_ ==  13): return "BatchSummaryResponse"
        if(type_ ==  14): return "TaskSummary"
        if(type_ ==  15): return "VehicleSummary"
        if(type_ ==  16): return "SpeedAltPair"
        if(type_ ==  17): return "ImpactAutomationRequest"
        if(type_ ==  18): return "ImpactAutomationResponse"
        if(type_ ==  19): return "PointOfInterest"
        if(type_ ==  20): return "LineOfInterest"
        if(type_ ==  21): return "AreaOfInterest"
        if(type_ ==  22): return "ImpactPointSearchTask"
        if(type_ ==  23): return "PatternSearchTask"
        if(type_ ==  24): return "AngledAreaSearchTask"
        if(type_ ==  25): return "ImpactLineSearchTask"
        if(type_ ==  26): return "WatchTask"
        if(type_ ==  27): return "MultiVehicleWatchTask"
        if(type_ ==  28): return "CommRelayTask"
        if(type_ ==  29): return "CordonTask"
        if(type_ ==  30): return "BlockadeTask"
        if(type_ ==  31): return "EscortTask"
        if(type_ ==  32): return "ConfigurationRequest"
        if(type_ ==  33): return "WaterReport"
        if(type_ ==  34): return "WaterZone"
        if(type_ ==  35): return "PayloadDropTask"


    def getType(self, name):
        if ( name == "PowerConfiguration"): return 1
        if ( name == "RadioConfiguration"): return 2
        if ( name == "RadioTowerConfiguration"): return 3
        if ( name == "RadioState"): return 4
        if ( name == "RadioTowerState"): return 5
        if ( name == "ImpactPayloadConfiguration"): return 6
        if ( name == "DeployImpactPayload"): return 7
        if ( name == "PowerPlantState"): return 8
        if ( name == "BatchRoutePlanRequest"): return 9
        if ( name == "BatchRoutePlanResponse"): return 10
        if ( name == "TaskTimingPair"): return 11
        if ( name == "BatchSummaryRequest"): return 12
        if ( name == "BatchSummaryResponse"): return 13
        if ( name == "TaskSummary"): return 14
        if ( name == "VehicleSummary"): return 15
        if ( name == "SpeedAltPair"): return 16
        if ( name == "ImpactAutomationRequest"): return 17
        if ( name == "ImpactAutomationResponse"): return 18
        if ( name == "PointOfInterest"): return 19
        if ( name == "LineOfInterest"): return 20
        if ( name == "AreaOfInterest"): return 21
        if ( name == "ImpactPointSearchTask"): return 22
        if ( name == "PatternSearchTask"): return 23
        if ( name == "AngledAreaSearchTask"): return 24
        if ( name == "ImpactLineSearchTask"): return 25
        if ( name == "WatchTask"): return 26
        if ( name == "MultiVehicleWatchTask"): return 27
        if ( name == "CommRelayTask"): return 28
        if ( name == "CordonTask"): return 29
        if ( name == "BlockadeTask"): return 30
        if ( name == "EscortTask"): return 31
        if ( name == "ConfigurationRequest"): return 32
        if ( name == "WaterReport"): return 33
        if ( name == "WaterZone"): return 34
        if ( name == "PayloadDropTask"): return 35

        return -1

    def getInstance(self, type_):
        if(type_ ==  1): return PowerConfiguration.PowerConfiguration()
        if(type_ ==  2): return RadioConfiguration.RadioConfiguration()
        if(type_ ==  3): return RadioTowerConfiguration.RadioTowerConfiguration()
        if(type_ ==  4): return RadioState.RadioState()
        if(type_ ==  5): return RadioTowerState.RadioTowerState()
        if(type_ ==  6): return ImpactPayloadConfiguration.ImpactPayloadConfiguration()
        if(type_ ==  7): return DeployImpactPayload.DeployImpactPayload()
        if(type_ ==  8): return PowerPlantState.PowerPlantState()
        if(type_ ==  9): return BatchRoutePlanRequest.BatchRoutePlanRequest()
        if(type_ ==  10): return BatchRoutePlanResponse.BatchRoutePlanResponse()
        if(type_ ==  11): return TaskTimingPair.TaskTimingPair()
        if(type_ ==  12): return BatchSummaryRequest.BatchSummaryRequest()
        if(type_ ==  13): return BatchSummaryResponse.BatchSummaryResponse()
        if(type_ ==  14): return TaskSummary.TaskSummary()
        if(type_ ==  15): return VehicleSummary.VehicleSummary()
        if(type_ ==  16): return SpeedAltPair.SpeedAltPair()
        if(type_ ==  17): return ImpactAutomationRequest.ImpactAutomationRequest()
        if(type_ ==  18): return ImpactAutomationResponse.ImpactAutomationResponse()
        if(type_ ==  19): return PointOfInterest.PointOfInterest()
        if(type_ ==  20): return LineOfInterest.LineOfInterest()
        if(type_ ==  21): return AreaOfInterest.AreaOfInterest()
        if(type_ ==  22): return ImpactPointSearchTask.ImpactPointSearchTask()
        if(type_ ==  23): return PatternSearchTask.PatternSearchTask()
        if(type_ ==  24): return AngledAreaSearchTask.AngledAreaSearchTask()
        if(type_ ==  25): return ImpactLineSearchTask.ImpactLineSearchTask()
        if(type_ ==  26): return WatchTask.WatchTask()
        if(type_ ==  27): return MultiVehicleWatchTask.MultiVehicleWatchTask()
        if(type_ ==  28): return CommRelayTask.CommRelayTask()
        if(type_ ==  29): return CordonTask.CordonTask()
        if(type_ ==  30): return BlockadeTask.BlockadeTask()
        if(type_ ==  31): return EscortTask.EscortTask()
        if(type_ ==  32): return ConfigurationRequest.ConfigurationRequest()
        if(type_ ==  33): return WaterReport.WaterReport()
        if(type_ ==  34): return WaterZone.WaterZone()
        if(type_ ==  35): return PayloadDropTask.PayloadDropTask()

        return None
