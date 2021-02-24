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

from midca.domains.tsp.lmcp.py.afrl.cmasi import AbstractGeometry
from midca.domains.tsp.lmcp.py.afrl.cmasi import KeyValuePair
from midca.domains.tsp.lmcp.py.afrl.cmasi import Location3D
from midca.domains.tsp.lmcp.py.afrl.cmasi import PayloadAction
from midca.domains.tsp.lmcp.py.afrl.cmasi import PayloadConfiguration
from midca.domains.tsp.lmcp.py.afrl.cmasi import PayloadState
from midca.domains.tsp.lmcp.py.afrl.cmasi import VehicleAction
from midca.domains.tsp.lmcp.py.afrl.cmasi import Task
from midca.domains.tsp.lmcp.py.afrl.cmasi import SearchTask
from midca.domains.tsp.lmcp.py.afrl.cmasi import AbstractZone
from midca.domains.tsp.lmcp.py.afrl.cmasi import EntityConfiguration
from midca.domains.tsp.lmcp.py.afrl.cmasi import FlightProfile
from midca.domains.tsp.lmcp.py.afrl.cmasi import AirVehicleConfiguration
from midca.domains.tsp.lmcp.py.afrl.cmasi import EntityState
from midca.domains.tsp.lmcp.py.afrl.cmasi import AirVehicleState
from midca.domains.tsp.lmcp.py.afrl.cmasi import Wedge
from midca.domains.tsp.lmcp.py.afrl.cmasi import AreaSearchTask
from midca.domains.tsp.lmcp.py.afrl.cmasi import CameraAction
from midca.domains.tsp.lmcp.py.afrl.cmasi import CameraConfiguration
from midca.domains.tsp.lmcp.py.afrl.cmasi import GimballedPayloadState
from midca.domains.tsp.lmcp.py.afrl.cmasi import CameraState
from midca.domains.tsp.lmcp.py.afrl.cmasi import Circle
from midca.domains.tsp.lmcp.py.afrl.cmasi import GimbalAngleAction
from midca.domains.tsp.lmcp.py.afrl.cmasi import GimbalConfiguration
from midca.domains.tsp.lmcp.py.afrl.cmasi import GimbalScanAction
from midca.domains.tsp.lmcp.py.afrl.cmasi import GimbalStareAction
from midca.domains.tsp.lmcp.py.afrl.cmasi import GimbalState
from midca.domains.tsp.lmcp.py.afrl.cmasi import GoToWaypointAction
from midca.domains.tsp.lmcp.py.afrl.cmasi import KeepInZone
from midca.domains.tsp.lmcp.py.afrl.cmasi import KeepOutZone
from midca.domains.tsp.lmcp.py.afrl.cmasi import LineSearchTask
from midca.domains.tsp.lmcp.py.afrl.cmasi import NavigationAction
from midca.domains.tsp.lmcp.py.afrl.cmasi import LoiterAction
from midca.domains.tsp.lmcp.py.afrl.cmasi import LoiterTask
from midca.domains.tsp.lmcp.py.afrl.cmasi import Waypoint
from midca.domains.tsp.lmcp.py.afrl.cmasi import MissionCommand
from midca.domains.tsp.lmcp.py.afrl.cmasi import MustFlyTask
from midca.domains.tsp.lmcp.py.afrl.cmasi import OperatorSignal
from midca.domains.tsp.lmcp.py.afrl.cmasi import OperatingRegion
from midca.domains.tsp.lmcp.py.afrl.cmasi import AutomationRequest
from midca.domains.tsp.lmcp.py.afrl.cmasi import PointSearchTask
from midca.domains.tsp.lmcp.py.afrl.cmasi import Polygon
from midca.domains.tsp.lmcp.py.afrl.cmasi import Rectangle
from midca.domains.tsp.lmcp.py.afrl.cmasi import RemoveTasks
from midca.domains.tsp.lmcp.py.afrl.cmasi import ServiceStatus
from midca.domains.tsp.lmcp.py.afrl.cmasi import SessionStatus
from midca.domains.tsp.lmcp.py.afrl.cmasi import VehicleActionCommand
from midca.domains.tsp.lmcp.py.afrl.cmasi import VideoStreamAction
from midca.domains.tsp.lmcp.py.afrl.cmasi import VideoStreamConfiguration
from midca.domains.tsp.lmcp.py.afrl.cmasi import VideoStreamState
from midca.domains.tsp.lmcp.py.afrl.cmasi import AutomationResponse
from midca.domains.tsp.lmcp.py.afrl.cmasi import RemoveZones
from midca.domains.tsp.lmcp.py.afrl.cmasi import RemoveEntities
from midca.domains.tsp.lmcp.py.afrl.cmasi import FlightDirectorAction
from midca.domains.tsp.lmcp.py.afrl.cmasi import WeatherReport
from midca.domains.tsp.lmcp.py.afrl.cmasi import FollowPathCommand
from midca.domains.tsp.lmcp.py.afrl.cmasi import PathWaypoint
from midca.domains.tsp.lmcp.py.afrl.cmasi import StopMovementAction
from midca.domains.tsp.lmcp.py.afrl.cmasi import WaypointTransfer
from midca.domains.tsp.lmcp.py.afrl.cmasi import PayloadStowAction


SERIES_NAME = "CMASI"
#Series Name turned into a long for quick comparisons.
SERIES_NAME_ID = 4849604199710720000
SERIES_VERSION = 3


class SeriesEnum:

    def getName(self, type_):
        if(type_ ==  1): return "AbstractGeometry"
        if(type_ ==  2): return "KeyValuePair"
        if(type_ ==  3): return "Location3D"
        if(type_ ==  4): return "PayloadAction"
        if(type_ ==  5): return "PayloadConfiguration"
        if(type_ ==  6): return "PayloadState"
        if(type_ ==  7): return "VehicleAction"
        if(type_ ==  8): return "Task"
        if(type_ ==  9): return "SearchTask"
        if(type_ ==  10): return "AbstractZone"
        if(type_ ==  11): return "EntityConfiguration"
        if(type_ ==  12): return "FlightProfile"
        if(type_ ==  13): return "AirVehicleConfiguration"
        if(type_ ==  14): return "EntityState"
        if(type_ ==  15): return "AirVehicleState"
        if(type_ ==  16): return "Wedge"
        if(type_ ==  17): return "AreaSearchTask"
        if(type_ ==  18): return "CameraAction"
        if(type_ ==  19): return "CameraConfiguration"
        if(type_ ==  20): return "GimballedPayloadState"
        if(type_ ==  21): return "CameraState"
        if(type_ ==  22): return "Circle"
        if(type_ ==  23): return "GimbalAngleAction"
        if(type_ ==  24): return "GimbalConfiguration"
        if(type_ ==  25): return "GimbalScanAction"
        if(type_ ==  26): return "GimbalStareAction"
        if(type_ ==  27): return "GimbalState"
        if(type_ ==  28): return "GoToWaypointAction"
        if(type_ ==  29): return "KeepInZone"
        if(type_ ==  30): return "KeepOutZone"
        if(type_ ==  31): return "LineSearchTask"
        if(type_ ==  32): return "NavigationAction"
        if(type_ ==  33): return "LoiterAction"
        if(type_ ==  34): return "LoiterTask"
        if(type_ ==  35): return "Waypoint"
        if(type_ ==  36): return "MissionCommand"
        if(type_ ==  37): return "MustFlyTask"
        if(type_ ==  38): return "OperatorSignal"
        if(type_ ==  39): return "OperatingRegion"
        if(type_ ==  40): return "AutomationRequest"
        if(type_ ==  41): return "PointSearchTask"
        if(type_ ==  42): return "Polygon"
        if(type_ ==  43): return "Rectangle"
        if(type_ ==  44): return "RemoveTasks"
        if(type_ ==  45): return "ServiceStatus"
        if(type_ ==  46): return "SessionStatus"
        if(type_ ==  47): return "VehicleActionCommand"
        if(type_ ==  48): return "VideoStreamAction"
        if(type_ ==  49): return "VideoStreamConfiguration"
        if(type_ ==  50): return "VideoStreamState"
        if(type_ ==  51): return "AutomationResponse"
        if(type_ ==  52): return "RemoveZones"
        if(type_ ==  53): return "RemoveEntities"
        if(type_ ==  54): return "FlightDirectorAction"
        if(type_ ==  55): return "WeatherReport"
        if(type_ ==  56): return "FollowPathCommand"
        if(type_ ==  57): return "PathWaypoint"
        if(type_ ==  58): return "StopMovementAction"
        if(type_ ==  59): return "WaypointTransfer"
        if(type_ ==  60): return "PayloadStowAction"


    def getType(self, name):
        if ( name == "AbstractGeometry"): return 1
        if ( name == "KeyValuePair"): return 2
        if ( name == "Location3D"): return 3
        if ( name == "PayloadAction"): return 4
        if ( name == "PayloadConfiguration"): return 5
        if ( name == "PayloadState"): return 6
        if ( name == "VehicleAction"): return 7
        if ( name == "Task"): return 8
        if ( name == "SearchTask"): return 9
        if ( name == "AbstractZone"): return 10
        if ( name == "EntityConfiguration"): return 11
        if ( name == "FlightProfile"): return 12
        if ( name == "AirVehicleConfiguration"): return 13
        if ( name == "EntityState"): return 14
        if ( name == "AirVehicleState"): return 15
        if ( name == "Wedge"): return 16
        if ( name == "AreaSearchTask"): return 17
        if ( name == "CameraAction"): return 18
        if ( name == "CameraConfiguration"): return 19
        if ( name == "GimballedPayloadState"): return 20
        if ( name == "CameraState"): return 21
        if ( name == "Circle"): return 22
        if ( name == "GimbalAngleAction"): return 23
        if ( name == "GimbalConfiguration"): return 24
        if ( name == "GimbalScanAction"): return 25
        if ( name == "GimbalStareAction"): return 26
        if ( name == "GimbalState"): return 27
        if ( name == "GoToWaypointAction"): return 28
        if ( name == "KeepInZone"): return 29
        if ( name == "KeepOutZone"): return 30
        if ( name == "LineSearchTask"): return 31
        if ( name == "NavigationAction"): return 32
        if ( name == "LoiterAction"): return 33
        if ( name == "LoiterTask"): return 34
        if ( name == "Waypoint"): return 35
        if ( name == "MissionCommand"): return 36
        if ( name == "MustFlyTask"): return 37
        if ( name == "OperatorSignal"): return 38
        if ( name == "OperatingRegion"): return 39
        if ( name == "AutomationRequest"): return 40
        if ( name == "PointSearchTask"): return 41
        if ( name == "Polygon"): return 42
        if ( name == "Rectangle"): return 43
        if ( name == "RemoveTasks"): return 44
        if ( name == "ServiceStatus"): return 45
        if ( name == "SessionStatus"): return 46
        if ( name == "VehicleActionCommand"): return 47
        if ( name == "VideoStreamAction"): return 48
        if ( name == "VideoStreamConfiguration"): return 49
        if ( name == "VideoStreamState"): return 50
        if ( name == "AutomationResponse"): return 51
        if ( name == "RemoveZones"): return 52
        if ( name == "RemoveEntities"): return 53
        if ( name == "FlightDirectorAction"): return 54
        if ( name == "WeatherReport"): return 55
        if ( name == "FollowPathCommand"): return 56
        if ( name == "PathWaypoint"): return 57
        if ( name == "StopMovementAction"): return 58
        if ( name == "WaypointTransfer"): return 59
        if ( name == "PayloadStowAction"): return 60

        return -1

    def getInstance(self, type_):
        if(type_ ==  1): return AbstractGeometry.AbstractGeometry()
        if(type_ ==  2): return KeyValuePair.KeyValuePair()
        if(type_ ==  3): return Location3D.Location3D()
        if(type_ ==  4): return PayloadAction.PayloadAction()
        if(type_ ==  5): return PayloadConfiguration.PayloadConfiguration()
        if(type_ ==  6): return PayloadState.PayloadState()
        if(type_ ==  7): return VehicleAction.VehicleAction()
        if(type_ ==  8): return Task.Task()
        if(type_ ==  9): return SearchTask.SearchTask()
        if(type_ ==  10): return AbstractZone.AbstractZone()
        if(type_ ==  11): return EntityConfiguration.EntityConfiguration()
        if(type_ ==  12): return FlightProfile.FlightProfile()
        if(type_ ==  13): return AirVehicleConfiguration.AirVehicleConfiguration()
        if(type_ ==  14): return EntityState.EntityState()
        if(type_ ==  15): return AirVehicleState.AirVehicleState()
        if(type_ ==  16): return Wedge.Wedge()
        if(type_ ==  17): return AreaSearchTask.AreaSearchTask()
        if(type_ ==  18): return CameraAction.CameraAction()
        if(type_ ==  19): return CameraConfiguration.CameraConfiguration()
        if(type_ ==  20): return GimballedPayloadState.GimballedPayloadState()
        if(type_ ==  21): return CameraState.CameraState()
        if(type_ ==  22): return Circle.Circle()
        if(type_ ==  23): return GimbalAngleAction.GimbalAngleAction()
        if(type_ ==  24): return GimbalConfiguration.GimbalConfiguration()
        if(type_ ==  25): return GimbalScanAction.GimbalScanAction()
        if(type_ ==  26): return GimbalStareAction.GimbalStareAction()
        if(type_ ==  27): return GimbalState.GimbalState()
        if(type_ ==  28): return GoToWaypointAction.GoToWaypointAction()
        if(type_ ==  29): return KeepInZone.KeepInZone()
        if(type_ ==  30): return KeepOutZone.KeepOutZone()
        if(type_ ==  31): return LineSearchTask.LineSearchTask()
        if(type_ ==  32): return NavigationAction.NavigationAction()
        if(type_ ==  33): return LoiterAction.LoiterAction()
        if(type_ ==  34): return LoiterTask.LoiterTask()
        if(type_ ==  35): return Waypoint.Waypoint()
        if(type_ ==  36): return MissionCommand.MissionCommand()
        if(type_ ==  37): return MustFlyTask.MustFlyTask()
        if(type_ ==  38): return OperatorSignal.OperatorSignal()
        if(type_ ==  39): return OperatingRegion.OperatingRegion()
        if(type_ ==  40): return AutomationRequest.AutomationRequest()
        if(type_ ==  41): return PointSearchTask.PointSearchTask()
        if(type_ ==  42): return Polygon.Polygon()
        if(type_ ==  43): return Rectangle.Rectangle()
        if(type_ ==  44): return RemoveTasks.RemoveTasks()
        if(type_ ==  45): return ServiceStatus.ServiceStatus()
        if(type_ ==  46): return SessionStatus.SessionStatus()
        if(type_ ==  47): return VehicleActionCommand.VehicleActionCommand()
        if(type_ ==  48): return VideoStreamAction.VideoStreamAction()
        if(type_ ==  49): return VideoStreamConfiguration.VideoStreamConfiguration()
        if(type_ ==  50): return VideoStreamState.VideoStreamState()
        if(type_ ==  51): return AutomationResponse.AutomationResponse()
        if(type_ ==  52): return RemoveZones.RemoveZones()
        if(type_ ==  53): return RemoveEntities.RemoveEntities()
        if(type_ ==  54): return FlightDirectorAction.FlightDirectorAction()
        if(type_ ==  55): return WeatherReport.WeatherReport()
        if(type_ ==  56): return FollowPathCommand.FollowPathCommand()
        if(type_ ==  57): return PathWaypoint.PathWaypoint()
        if(type_ ==  58): return StopMovementAction.StopMovementAction()
        if(type_ ==  59): return WaypointTransfer.WaypointTransfer()
        if(type_ ==  60): return PayloadStowAction.PayloadStowAction()

        return None
