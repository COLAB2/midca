import socket
from midca.domains.tsp.lmcp.py.lmcp import LMCPFactory

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
from midca.domains.tsp.lmcp.py.afrl.cmasi.perceive import EntityPerception
from midca.domains.tsp.lmcp.py.afrl.cmasi.perceive import TrackEntityAction
from midca.domains.tsp.lmcp.py.afrl.cmasi.perceive import TrackEntityTask
from midca.domains.tsp.lmcp.py.uxas.messages.route import GraphNode
from midca.domains.tsp.lmcp.py.uxas.messages.route import GraphEdge
from midca.domains.tsp.lmcp.py.uxas.messages.route import GraphRegion
from midca.domains.tsp.lmcp.py.uxas.messages.route import RouteConstraints
from midca.domains.tsp.lmcp.py.uxas.messages.route import RouteRequest
from midca.domains.tsp.lmcp.py.uxas.messages.route import RoutePlanRequest
from midca.domains.tsp.lmcp.py.uxas.messages.route import RoutePlan
from midca.domains.tsp.lmcp.py.uxas.messages.route import RoutePlanResponse
from midca.domains.tsp.lmcp.py.uxas.messages.route import RouteResponse
from midca.domains.tsp.lmcp.py.uxas.messages.route import EgressRouteRequest
from midca.domains.tsp.lmcp.py.uxas.messages.route import EgressRouteResponse
from midca.domains.tsp.lmcp.py.uxas.messages.route import RoadPointsConstraints
from midca.domains.tsp.lmcp.py.uxas.messages.route import RoadPointsRequest
from midca.domains.tsp.lmcp.py.uxas.messages.route import RoadPointsResponse
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
from midca.domains.tsp.lmcp.py.uxas.messages.task import AssignmentCoordinatorTask
from midca.domains.tsp.lmcp.py.uxas.messages.task import RendezvousTask
from midca.domains.tsp.lmcp.py.uxas.messages.task import PlanningState
from midca.domains.tsp.lmcp.py.uxas.messages.task import AssignmentCoordination
from midca.domains.tsp.lmcp.py.uxas.messages.task import CoordinatedAutomationRequest
from midca.domains.tsp.lmcp.py.uxas.messages.task import TaskAutomationRequest
from midca.domains.tsp.lmcp.py.uxas.messages.task import TaskAutomationResponse
from midca.domains.tsp.lmcp.py.uxas.messages.task import UniqueAutomationRequest
from midca.domains.tsp.lmcp.py.uxas.messages.task import UniqueAutomationResponse
from midca.domains.tsp.lmcp.py.uxas.messages.task import SensorFootprintRequests
from midca.domains.tsp.lmcp.py.uxas.messages.task import FootprintRequest
from midca.domains.tsp.lmcp.py.uxas.messages.task import SensorFootprint
from midca.domains.tsp.lmcp.py.uxas.messages.task import SensorFootprintResponse
from midca.domains.tsp.lmcp.py.uxas.messages.task import TaskImplementationRequest
from midca.domains.tsp.lmcp.py.uxas.messages.task import TaskImplementationResponse
from midca.domains.tsp.lmcp.py.uxas.messages.task import AssignmentCostMatrix
from midca.domains.tsp.lmcp.py.uxas.messages.task import TaskOptionCost
from midca.domains.tsp.lmcp.py.uxas.messages.task import TaskAssignment
from midca.domains.tsp.lmcp.py.uxas.messages.task import TaskAssignmentSummary
from midca.domains.tsp.lmcp.py.uxas.messages.task import TaskOption
from midca.domains.tsp.lmcp.py.uxas.messages.task import TaskPlanOptions
from midca.domains.tsp.lmcp.py.uxas.messages.task import TaskPause
from midca.domains.tsp.lmcp.py.uxas.messages.task import TaskResume
from midca.domains.tsp.lmcp.py.uxas.messages.task import TaskProgress
from midca.domains.tsp.lmcp.py.uxas.messages.task import TaskProgressRequest
from midca.domains.tsp.lmcp.py.uxas.messages.task import TaskInitialized
from midca.domains.tsp.lmcp.py.uxas.messages.task import TaskActive
from midca.domains.tsp.lmcp.py.uxas.messages.task import TaskComplete
from midca.domains.tsp.lmcp.py.uxas.messages.task import CancelTask
from midca.domains.tsp.lmcp.py.afrl.vehicles import GroundVehicleConfiguration
from midca.domains.tsp.lmcp.py.afrl.vehicles import GroundVehicleState
from midca.domains.tsp.lmcp.py.afrl.vehicles import SurfaceVehicleConfiguration
from midca.domains.tsp.lmcp.py.afrl.vehicles import SurfaceVehicleState
from midca.domains.tsp.lmcp.py.afrl.vehicles import StationarySensorConfiguration
from midca.domains.tsp.lmcp.py.afrl.vehicles import StationarySensorState


s = socket.socket()
host = socket.gethostname()
port = 11041
s.connect((host, port))
buf = bytearray()

#Pack AbstractGeometry
obj = AbstractGeometry.AbstractGeometry()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack KeyValuePair
obj = KeyValuePair.KeyValuePair()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack Location3D
obj = Location3D.Location3D()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack PayloadAction
obj = PayloadAction.PayloadAction()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack PayloadConfiguration
obj = PayloadConfiguration.PayloadConfiguration()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack PayloadState
obj = PayloadState.PayloadState()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack VehicleAction
obj = VehicleAction.VehicleAction()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack Task
obj = Task.Task()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack SearchTask
obj = SearchTask.SearchTask()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack AbstractZone
obj = AbstractZone.AbstractZone()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack EntityConfiguration
obj = EntityConfiguration.EntityConfiguration()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack FlightProfile
obj = FlightProfile.FlightProfile()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack AirVehicleConfiguration
obj = AirVehicleConfiguration.AirVehicleConfiguration()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack EntityState
obj = EntityState.EntityState()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack AirVehicleState
obj = AirVehicleState.AirVehicleState()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack Wedge
obj = Wedge.Wedge()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack AreaSearchTask
obj = AreaSearchTask.AreaSearchTask()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack CameraAction
obj = CameraAction.CameraAction()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack CameraConfiguration
obj = CameraConfiguration.CameraConfiguration()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack GimballedPayloadState
obj = GimballedPayloadState.GimballedPayloadState()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack CameraState
obj = CameraState.CameraState()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack Circle
obj = Circle.Circle()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack GimbalAngleAction
obj = GimbalAngleAction.GimbalAngleAction()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack GimbalConfiguration
obj = GimbalConfiguration.GimbalConfiguration()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack GimbalScanAction
obj = GimbalScanAction.GimbalScanAction()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack GimbalStareAction
obj = GimbalStareAction.GimbalStareAction()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack GimbalState
obj = GimbalState.GimbalState()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack GoToWaypointAction
obj = GoToWaypointAction.GoToWaypointAction()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack KeepInZone
obj = KeepInZone.KeepInZone()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack KeepOutZone
obj = KeepOutZone.KeepOutZone()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack LineSearchTask
obj = LineSearchTask.LineSearchTask()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack NavigationAction
obj = NavigationAction.NavigationAction()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack LoiterAction
obj = LoiterAction.LoiterAction()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack LoiterTask
obj = LoiterTask.LoiterTask()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack Waypoint
obj = Waypoint.Waypoint()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack MissionCommand
obj = MissionCommand.MissionCommand()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack MustFlyTask
obj = MustFlyTask.MustFlyTask()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack OperatorSignal
obj = OperatorSignal.OperatorSignal()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack OperatingRegion
obj = OperatingRegion.OperatingRegion()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack AutomationRequest
obj = AutomationRequest.AutomationRequest()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack PointSearchTask
obj = PointSearchTask.PointSearchTask()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack Polygon
obj = Polygon.Polygon()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack Rectangle
obj = Rectangle.Rectangle()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack RemoveTasks
obj = RemoveTasks.RemoveTasks()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack ServiceStatus
obj = ServiceStatus.ServiceStatus()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack SessionStatus
obj = SessionStatus.SessionStatus()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack VehicleActionCommand
obj = VehicleActionCommand.VehicleActionCommand()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack VideoStreamAction
obj = VideoStreamAction.VideoStreamAction()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack VideoStreamConfiguration
obj = VideoStreamConfiguration.VideoStreamConfiguration()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack VideoStreamState
obj = VideoStreamState.VideoStreamState()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack AutomationResponse
obj = AutomationResponse.AutomationResponse()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack RemoveZones
obj = RemoveZones.RemoveZones()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack RemoveEntities
obj = RemoveEntities.RemoveEntities()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack FlightDirectorAction
obj = FlightDirectorAction.FlightDirectorAction()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack WeatherReport
obj = WeatherReport.WeatherReport()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack FollowPathCommand
obj = FollowPathCommand.FollowPathCommand()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack PathWaypoint
obj = PathWaypoint.PathWaypoint()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack StopMovementAction
obj = StopMovementAction.StopMovementAction()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack WaypointTransfer
obj = WaypointTransfer.WaypointTransfer()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack PayloadStowAction
obj = PayloadStowAction.PayloadStowAction()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack PowerConfiguration
obj = PowerConfiguration.PowerConfiguration()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack RadioConfiguration
obj = RadioConfiguration.RadioConfiguration()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack RadioTowerConfiguration
obj = RadioTowerConfiguration.RadioTowerConfiguration()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack RadioState
obj = RadioState.RadioState()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack RadioTowerState
obj = RadioTowerState.RadioTowerState()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack ImpactPayloadConfiguration
obj = ImpactPayloadConfiguration.ImpactPayloadConfiguration()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack DeployImpactPayload
obj = DeployImpactPayload.DeployImpactPayload()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack PowerPlantState
obj = PowerPlantState.PowerPlantState()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack BatchRoutePlanRequest
obj = BatchRoutePlanRequest.BatchRoutePlanRequest()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack BatchRoutePlanResponse
obj = BatchRoutePlanResponse.BatchRoutePlanResponse()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack TaskTimingPair
obj = TaskTimingPair.TaskTimingPair()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack BatchSummaryRequest
obj = BatchSummaryRequest.BatchSummaryRequest()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack BatchSummaryResponse
obj = BatchSummaryResponse.BatchSummaryResponse()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack TaskSummary
obj = TaskSummary.TaskSummary()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack VehicleSummary
obj = VehicleSummary.VehicleSummary()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack SpeedAltPair
obj = SpeedAltPair.SpeedAltPair()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack ImpactAutomationRequest
obj = ImpactAutomationRequest.ImpactAutomationRequest()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack ImpactAutomationResponse
obj = ImpactAutomationResponse.ImpactAutomationResponse()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack PointOfInterest
obj = PointOfInterest.PointOfInterest()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack LineOfInterest
obj = LineOfInterest.LineOfInterest()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack AreaOfInterest
obj = AreaOfInterest.AreaOfInterest()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack ImpactPointSearchTask
obj = ImpactPointSearchTask.ImpactPointSearchTask()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack PatternSearchTask
obj = PatternSearchTask.PatternSearchTask()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack AngledAreaSearchTask
obj = AngledAreaSearchTask.AngledAreaSearchTask()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack ImpactLineSearchTask
obj = ImpactLineSearchTask.ImpactLineSearchTask()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack WatchTask
obj = WatchTask.WatchTask()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack MultiVehicleWatchTask
obj = MultiVehicleWatchTask.MultiVehicleWatchTask()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack CommRelayTask
obj = CommRelayTask.CommRelayTask()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack CordonTask
obj = CordonTask.CordonTask()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack BlockadeTask
obj = BlockadeTask.BlockadeTask()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack EscortTask
obj = EscortTask.EscortTask()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack ConfigurationRequest
obj = ConfigurationRequest.ConfigurationRequest()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack WaterReport
obj = WaterReport.WaterReport()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack WaterZone
obj = WaterZone.WaterZone()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack PayloadDropTask
obj = PayloadDropTask.PayloadDropTask()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack EntityPerception
obj = EntityPerception.EntityPerception()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack TrackEntityAction
obj = TrackEntityAction.TrackEntityAction()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack TrackEntityTask
obj = TrackEntityTask.TrackEntityTask()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack GraphNode
obj = GraphNode.GraphNode()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack GraphEdge
obj = GraphEdge.GraphEdge()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack GraphRegion
obj = GraphRegion.GraphRegion()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack RouteConstraints
obj = RouteConstraints.RouteConstraints()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack RouteRequest
obj = RouteRequest.RouteRequest()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack RoutePlanRequest
obj = RoutePlanRequest.RoutePlanRequest()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack RoutePlan
obj = RoutePlan.RoutePlan()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack RoutePlanResponse
obj = RoutePlanResponse.RoutePlanResponse()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack RouteResponse
obj = RouteResponse.RouteResponse()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack EgressRouteRequest
obj = EgressRouteRequest.EgressRouteRequest()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack EgressRouteResponse
obj = EgressRouteResponse.EgressRouteResponse()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack RoadPointsConstraints
obj = RoadPointsConstraints.RoadPointsConstraints()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack RoadPointsRequest
obj = RoadPointsRequest.RoadPointsRequest()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack RoadPointsResponse
obj = RoadPointsResponse.RoadPointsResponse()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack VideoRecord
obj = VideoRecord.VideoRecord()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack StartupComplete
obj = StartupComplete.StartupComplete()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack CreateNewService
obj = CreateNewService.CreateNewService()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack KillService
obj = KillService.KillService()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack IncrementWaypoint
obj = IncrementWaypoint.IncrementWaypoint()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack SafeHeadingAction
obj = SafeHeadingAction.SafeHeadingAction()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack EntityLocation
obj = EntityLocation.EntityLocation()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack BandwidthTest
obj = BandwidthTest.BandwidthTest()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack BandwidthReceiveReport
obj = BandwidthReceiveReport.BandwidthReceiveReport()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack SubTaskExecution
obj = SubTaskExecution.SubTaskExecution()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack SubTaskAssignment
obj = SubTaskAssignment.SubTaskAssignment()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack AutopilotKeepAlive
obj = AutopilotKeepAlive.AutopilotKeepAlive()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack OnboardStatusReport
obj = OnboardStatusReport.OnboardStatusReport()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack EntityJoin
obj = EntityJoin.EntityJoin()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack EntityExit
obj = EntityExit.EntityExit()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack SimulationTimeStepAcknowledgement
obj = SimulationTimeStepAcknowledgement.SimulationTimeStepAcknowledgement()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack SpeedOverrideAction
obj = SpeedOverrideAction.SpeedOverrideAction()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack AssignmentCoordinatorTask
obj = AssignmentCoordinatorTask.AssignmentCoordinatorTask()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack RendezvousTask
obj = RendezvousTask.RendezvousTask()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack PlanningState
obj = PlanningState.PlanningState()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack AssignmentCoordination
obj = AssignmentCoordination.AssignmentCoordination()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack CoordinatedAutomationRequest
obj = CoordinatedAutomationRequest.CoordinatedAutomationRequest()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack TaskAutomationRequest
obj = TaskAutomationRequest.TaskAutomationRequest()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack TaskAutomationResponse
obj = TaskAutomationResponse.TaskAutomationResponse()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack UniqueAutomationRequest
obj = UniqueAutomationRequest.UniqueAutomationRequest()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack UniqueAutomationResponse
obj = UniqueAutomationResponse.UniqueAutomationResponse()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack SensorFootprintRequests
obj = SensorFootprintRequests.SensorFootprintRequests()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack FootprintRequest
obj = FootprintRequest.FootprintRequest()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack SensorFootprint
obj = SensorFootprint.SensorFootprint()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack SensorFootprintResponse
obj = SensorFootprintResponse.SensorFootprintResponse()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack TaskImplementationRequest
obj = TaskImplementationRequest.TaskImplementationRequest()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack TaskImplementationResponse
obj = TaskImplementationResponse.TaskImplementationResponse()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack AssignmentCostMatrix
obj = AssignmentCostMatrix.AssignmentCostMatrix()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack TaskOptionCost
obj = TaskOptionCost.TaskOptionCost()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack TaskAssignment
obj = TaskAssignment.TaskAssignment()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack TaskAssignmentSummary
obj = TaskAssignmentSummary.TaskAssignmentSummary()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack TaskOption
obj = TaskOption.TaskOption()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack TaskPlanOptions
obj = TaskPlanOptions.TaskPlanOptions()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack TaskPause
obj = TaskPause.TaskPause()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack TaskResume
obj = TaskResume.TaskResume()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack TaskProgress
obj = TaskProgress.TaskProgress()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack TaskProgressRequest
obj = TaskProgressRequest.TaskProgressRequest()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack TaskInitialized
obj = TaskInitialized.TaskInitialized()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack TaskActive
obj = TaskActive.TaskActive()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack TaskComplete
obj = TaskComplete.TaskComplete()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack CancelTask
obj = CancelTask.CancelTask()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack GroundVehicleConfiguration
obj = GroundVehicleConfiguration.GroundVehicleConfiguration()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack GroundVehicleState
obj = GroundVehicleState.GroundVehicleState()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack SurfaceVehicleConfiguration
obj = SurfaceVehicleConfiguration.SurfaceVehicleConfiguration()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack SurfaceVehicleState
obj = SurfaceVehicleState.SurfaceVehicleState()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack StationarySensorConfiguration
obj = StationarySensorConfiguration.StationarySensorConfiguration()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack StationarySensorState
obj = StationarySensorState.StationarySensorState()
buf.extend(LMCPFactory.packMessage(obj, True))


s.send(buf)


