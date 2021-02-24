#!/usr/bin/env python3

## ===============================================================================
## Authors: AFRL/RQQA
## Organization: Air Force Research Laboratory, Aerospace Systems Directorate, Power and Control Division
##
## Copyright (c) 2017 Government of the United State of America, as represented by
## the Secretary of the Air Force.  No copyright is claimed in the United States under
## Title 17, U.S. Code.  All Other Rights Reserved.
## ===============================================================================

## This file was auto-created by LmcpGen. Modifications will be overwritten.

import argparse
from pathlib import Path
import os
import random
import string
import sys

from midca.domains.tsp.lmcp.py.lmcp import LMCPFactory

from midca.domains.tsp.lmcp.py.afrl.cmasi import WavelengthBand
from midca.domains.tsp.lmcp.py.afrl.cmasi import NavigationMode
from midca.domains.tsp.lmcp.py.afrl.cmasi import FOVOperationMode
from midca.domains.tsp.lmcp.py.afrl.cmasi import GimbalPointingMode
from midca.domains.tsp.lmcp.py.afrl.cmasi import ZoneAvoidanceType
from midca.domains.tsp.lmcp.py.afrl.cmasi import LoiterType
from midca.domains.tsp.lmcp.py.afrl.cmasi import LoiterDirection
from midca.domains.tsp.lmcp.py.afrl.cmasi import ServiceStatusType
from midca.domains.tsp.lmcp.py.afrl.cmasi import SimulationStatusType
from midca.domains.tsp.lmcp.py.afrl.cmasi import SpeedType
from midca.domains.tsp.lmcp.py.afrl.cmasi import TurnType
from midca.domains.tsp.lmcp.py.afrl.cmasi import CommandStatusType
from midca.domains.tsp.lmcp.py.afrl.cmasi import AltitudeType
from midca.domains.tsp.lmcp.py.afrl.cmasi import TravelMode
from midca.domains.tsp.lmcp.py.afrl.cmasi import WaypointTransferMode
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
from midca.domains.tsp.lmcp.py.afrl.impact import AreaSearchPattern
from midca.domains.tsp.lmcp.py.afrl.impact import PowerPlant
from midca.domains.tsp.lmcp.py.afrl.impact import AreaActionOptions
from midca.domains.tsp.lmcp.py.afrl.impact import ImpactPayloadType
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


class Generator:

    def __init__(self, dir, seed, limit, checksum):
        self.dir = dir
        self.seed = seed
        self.limit = limit
        self.checksum = checksum
        self.rng = random.Random(self.seed)

    def options_AbstractGeometry(self):
        return {
            0: self.build_AbstractGeometry,
            1: self.build_Rectangle,
            2: self.build_Circle,
            3: self.build_Polygon,
        }

    def options_Location3D(self):
        return {
            0: self.build_Location3D,
            1: self.build_PathWaypoint,
            2: self.build_Waypoint,
        }

    def options_PayloadAction(self):
        return {
            0: self.build_PayloadAction,
            1: self.build_CameraAction,
            2: self.build_GimbalStareAction,
            3: self.build_GimbalScanAction,
            4: self.build_GimbalAngleAction,
        }

    def options_PayloadConfiguration(self):
        return {
            0: self.build_PayloadConfiguration,
            1: self.build_RadioConfiguration,
            2: self.build_GimbalConfiguration,
            3: self.build_CameraConfiguration,
            4: self.build_ImpactPayloadConfiguration,
            5: self.build_VideoStreamConfiguration,
            6: self.build_PowerConfiguration,
        }

    def options_PayloadState(self):
        return {
            0: self.build_PayloadState,
            1: self.build_PowerPlantState,
            2: self.build_RadioState,
            3: self.build_CameraState,
            4: self.build_GimbalState,
            5: self.build_VideoStreamState,
            6: self.build_GimballedPayloadState,
        }

    def options_VehicleAction(self):
        return {
            0: self.build_VehicleAction,
            1: self.build_CameraAction,
            2: self.build_GoToWaypointAction,
            3: self.build_DeployImpactPayload,
            4: self.build_GimbalStareAction,
            5: self.build_LoiterAction,
            6: self.build_StopMovementAction,
            7: self.build_FlightDirectorAction,
            8: self.build_PayloadAction,
            9: self.build_SpeedOverrideAction,
            10: self.build_VideoStreamAction,
            11: self.build_GimbalScanAction,
            12: self.build_SafeHeadingAction,
            13: self.build_GimbalAngleAction,
            14: self.build_NavigationAction,
            15: self.build_TrackEntityAction,
        }

    def options_Task(self):
        return {
            0: self.build_Task,
            1: self.build_LineSearchTask,
            2: self.build_PointSearchTask,
            3: self.build_SearchTask,
            4: self.build_AreaSearchTask,
            5: self.build_RendezvousTask,
            6: self.build_MultiVehicleWatchTask,
            7: self.build_AngledAreaSearchTask,
            8: self.build_ImpactLineSearchTask,
            9: self.build_CordonTask,
            10: self.build_EscortTask,
            11: self.build_CommRelayTask,
            12: self.build_TrackEntityTask,
            13: self.build_LoiterTask,
            14: self.build_BlockadeTask,
            15: self.build_PayloadDropTask,
            16: self.build_PatternSearchTask,
            17: self.build_MustFlyTask,
            18: self.build_AssignmentCoordinatorTask,
            19: self.build_ImpactPointSearchTask,
            20: self.build_WatchTask,
        }

    def options_SearchTask(self):
        return {
            0: self.build_SearchTask,
            1: self.build_LineSearchTask,
            2: self.build_PointSearchTask,
            3: self.build_AreaSearchTask,
            4: self.build_MultiVehicleWatchTask,
            5: self.build_PatternSearchTask,
            6: self.build_AngledAreaSearchTask,
            7: self.build_ImpactLineSearchTask,
            8: self.build_ImpactPointSearchTask,
            9: self.build_WatchTask,
            10: self.build_EscortTask,
        }

    def options_AbstractZone(self):
        return {
            0: self.build_AbstractZone,
            1: self.build_KeepOutZone,
            2: self.build_WaterZone,
            3: self.build_KeepInZone,
        }

    def options_EntityConfiguration(self):
        return {
            0: self.build_EntityConfiguration,
            1: self.build_SurfaceVehicleConfiguration,
            2: self.build_AirVehicleConfiguration,
            3: self.build_RadioTowerConfiguration,
            4: self.build_GroundVehicleConfiguration,
            5: self.build_StationarySensorConfiguration,
        }

    def options_EntityState(self):
        return {
            0: self.build_EntityState,
            1: self.build_GroundVehicleState,
            2: self.build_SurfaceVehicleState,
            3: self.build_StationarySensorState,
            4: self.build_RadioTowerState,
            5: self.build_AirVehicleState,
        }

    def options_GimballedPayloadState(self):
        return {
            0: self.build_GimballedPayloadState,
            1: self.build_CameraState,
        }

    def options_NavigationAction(self):
        return {
            0: self.build_NavigationAction,
            1: self.build_GoToWaypointAction,
            2: self.build_LoiterAction,
            3: self.build_FlightDirectorAction,
        }

    def options_Waypoint(self):
        return {
            0: self.build_Waypoint,
            1: self.build_PathWaypoint,
        }

    def options_VehicleActionCommand(self):
        return {
            0: self.build_VehicleActionCommand,
            1: self.build_FollowPathCommand,
            2: self.build_MissionCommand,
        }

    def options_EntityLocation(self):
        return {
            0: self.build_EntityLocation,
            1: self.build_BandwidthTest,
        }


    def build_AbstractGeometry(self):
        msg = AbstractGeometry.AbstractGeometry()
        # AbstractGeometry fields
        return msg

    def build_KeyValuePair(self):
        msg = KeyValuePair.KeyValuePair()
        # KeyValuePair fields
        msg.set_Key(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        msg.set_Value(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        return msg

    def build_Location3D(self):
        msg = Location3D.Location3D()
        # Location3D fields
        msg.set_Latitude(self.rng.getrandbits(64))
        msg.set_Longitude(self.rng.getrandbits(64))
        msg.set_Altitude(self.rng.random())
        msg.set_AltitudeType(AltitudeType.get_AltitudeType_str(AltitudeType.get_AltitudeType_int(self.rng.choice([0,1]))))
        return msg

    def build_PayloadAction(self):
        msg = PayloadAction.PayloadAction()
        # VehicleAction fields
        v = msg.get_AssociatedTaskList()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        # PayloadAction fields
        msg.set_PayloadID(self.rng.randint(-(2**63-1), 2**63-1))
        return msg

    def build_PayloadConfiguration(self):
        msg = PayloadConfiguration.PayloadConfiguration()
        # PayloadConfiguration fields
        msg.set_PayloadID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_PayloadKind(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        v = msg.get_Parameters()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.build_KeyValuePair())
        return msg

    def build_PayloadState(self):
        msg = PayloadState.PayloadState()
        # PayloadState fields
        msg.set_PayloadID(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_Parameters()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.build_KeyValuePair())
        return msg

    def build_VehicleAction(self):
        msg = VehicleAction.VehicleAction()
        # VehicleAction fields
        v = msg.get_AssociatedTaskList()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        return msg

    def build_Task(self):
        msg = Task.Task()
        # Task fields
        msg.set_TaskID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Label(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        v = msg.get_EligibleEntities()
        for i in range(self.rng.randint(0, min(self.limit, 32))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_RevisitRate(self.rng.random())
        v = msg.get_Parameters()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.build_KeyValuePair())
        msg.set_Priority(self.rng.getrandbits(8))
        msg.set_Required(self.rng.choice([True,False]))
        return msg

    def build_SearchTask(self):
        msg = SearchTask.SearchTask()
        # Task fields
        msg.set_TaskID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Label(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        v = msg.get_EligibleEntities()
        for i in range(self.rng.randint(0, min(self.limit, 32))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_RevisitRate(self.rng.random())
        v = msg.get_Parameters()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.build_KeyValuePair())
        msg.set_Priority(self.rng.getrandbits(8))
        msg.set_Required(self.rng.choice([True,False]))
        # SearchTask fields
        v = msg.get_DesiredWavelengthBands()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(WavelengthBand.get_WavelengthBand_str(WavelengthBand.get_WavelengthBand_int(self.rng.choice([0,1,2,3,4,5]))))
        msg.set_DwellTime(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_GroundSampleDistance(self.rng.random())
        return msg

    def build_AbstractZone(self):
        msg = AbstractZone.AbstractZone()
        # AbstractZone fields
        msg.set_ZoneID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_MinAltitude(self.rng.random())
        msg.set_MinAltitudeType(AltitudeType.get_AltitudeType_str(AltitudeType.get_AltitudeType_int(self.rng.choice([0,1]))))
        msg.set_MaxAltitude(self.rng.random())
        msg.set_MaxAltitudeType(AltitudeType.get_AltitudeType_str(AltitudeType.get_AltitudeType_int(self.rng.choice([0,1]))))
        v = msg.get_AffectedAircraft()
        for i in range(self.rng.randint(0, min(self.limit, 32))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_StartTime(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_EndTime(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Padding(self.rng.random())
        msg.set_Label(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        msg.set_Boundary(self.options_AbstractGeometry()[self.rng.randint(0, 3)]())
        return msg

    def build_EntityConfiguration(self):
        msg = EntityConfiguration.EntityConfiguration()
        # EntityConfiguration fields
        msg.set_ID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Affiliation(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        msg.set_EntityType(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        msg.set_Label(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        msg.set_NominalSpeed(self.rng.random())
        msg.set_NominalAltitude(self.rng.random())
        msg.set_NominalAltitudeType(AltitudeType.get_AltitudeType_str(AltitudeType.get_AltitudeType_int(self.rng.choice([0,1]))))
        v = msg.get_PayloadConfigurationList()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.options_PayloadConfiguration()[self.rng.randint(0, 6)]())
        v = msg.get_Info()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.build_KeyValuePair())
        return msg

    def build_FlightProfile(self):
        msg = FlightProfile.FlightProfile()
        # FlightProfile fields
        msg.set_Name(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        msg.set_Airspeed(self.rng.random())
        msg.set_PitchAngle(self.rng.random())
        msg.set_VerticalSpeed(self.rng.random())
        msg.set_MaxBankAngle(self.rng.random())
        msg.set_EnergyRate(self.rng.random())
        return msg

    def build_AirVehicleConfiguration(self):
        msg = AirVehicleConfiguration.AirVehicleConfiguration()
        # EntityConfiguration fields
        msg.set_ID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Affiliation(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        msg.set_EntityType(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        msg.set_Label(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        msg.set_NominalSpeed(self.rng.random())
        msg.set_NominalAltitude(self.rng.random())
        msg.set_NominalAltitudeType(AltitudeType.get_AltitudeType_str(AltitudeType.get_AltitudeType_int(self.rng.choice([0,1]))))
        v = msg.get_PayloadConfigurationList()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.options_PayloadConfiguration()[self.rng.randint(0, 6)]())
        v = msg.get_Info()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.build_KeyValuePair())
        # AirVehicleConfiguration fields
        msg.set_MinimumSpeed(self.rng.random())
        msg.set_MaximumSpeed(self.rng.random())
        msg.set_NominalFlightProfile(self.build_FlightProfile())
        v = msg.get_AlternateFlightProfiles()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.build_FlightProfile())
        v = msg.get_AvailableLoiterTypes()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(LoiterType.get_LoiterType_str(LoiterType.get_LoiterType_int(self.rng.choice([0,1,2,3,4]))))
        v = msg.get_AvailableTurnTypes()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(TurnType.get_TurnType_str(TurnType.get_TurnType_int(self.rng.choice([0,1]))))
        msg.set_MinimumAltitude(self.rng.random())
        msg.set_MinAltitudeType(AltitudeType.get_AltitudeType_str(AltitudeType.get_AltitudeType_int(self.rng.choice([0,1]))))
        msg.set_MaximumAltitude(self.rng.random())
        msg.set_MaxAltitudeType(AltitudeType.get_AltitudeType_str(AltitudeType.get_AltitudeType_int(self.rng.choice([0,1]))))
        return msg

    def build_EntityState(self):
        msg = EntityState.EntityState()
        # EntityState fields
        msg.set_ID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_u(self.rng.random())
        msg.set_v(self.rng.random())
        msg.set_w(self.rng.random())
        msg.set_udot(self.rng.random())
        msg.set_vdot(self.rng.random())
        msg.set_wdot(self.rng.random())
        msg.set_Heading(self.rng.random())
        msg.set_Pitch(self.rng.random())
        msg.set_Roll(self.rng.random())
        msg.set_p(self.rng.random())
        msg.set_q(self.rng.random())
        msg.set_r(self.rng.random())
        msg.set_Course(self.rng.random())
        msg.set_Groundspeed(self.rng.random())
        msg.set_Location(self.options_Location3D()[self.rng.randint(0, 2)]())
        msg.set_EnergyAvailable(self.rng.random())
        msg.set_ActualEnergyRate(self.rng.random())
        v = msg.get_PayloadStateList()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.options_PayloadState()[self.rng.randint(0, 6)]())
        msg.set_CurrentWaypoint(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_CurrentCommand(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Mode(NavigationMode.get_NavigationMode_str(NavigationMode.get_NavigationMode_int(self.rng.choice([0,1,2,3,4,5]))))
        v = msg.get_AssociatedTasks()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Time(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_Info()
        for i in range(self.rng.randint(0, min(self.limit, 32))):
            v.append(self.build_KeyValuePair())
        return msg

    def build_AirVehicleState(self):
        msg = AirVehicleState.AirVehicleState()
        # EntityState fields
        msg.set_ID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_u(self.rng.random())
        msg.set_v(self.rng.random())
        msg.set_w(self.rng.random())
        msg.set_udot(self.rng.random())
        msg.set_vdot(self.rng.random())
        msg.set_wdot(self.rng.random())
        msg.set_Heading(self.rng.random())
        msg.set_Pitch(self.rng.random())
        msg.set_Roll(self.rng.random())
        msg.set_p(self.rng.random())
        msg.set_q(self.rng.random())
        msg.set_r(self.rng.random())
        msg.set_Course(self.rng.random())
        msg.set_Groundspeed(self.rng.random())
        msg.set_Location(self.options_Location3D()[self.rng.randint(0, 2)]())
        msg.set_EnergyAvailable(self.rng.random())
        msg.set_ActualEnergyRate(self.rng.random())
        v = msg.get_PayloadStateList()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.options_PayloadState()[self.rng.randint(0, 6)]())
        msg.set_CurrentWaypoint(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_CurrentCommand(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Mode(NavigationMode.get_NavigationMode_str(NavigationMode.get_NavigationMode_int(self.rng.choice([0,1,2,3,4,5]))))
        v = msg.get_AssociatedTasks()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Time(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_Info()
        for i in range(self.rng.randint(0, min(self.limit, 32))):
            v.append(self.build_KeyValuePair())
        # AirVehicleState fields
        msg.set_Airspeed(self.rng.random())
        msg.set_VerticalSpeed(self.rng.random())
        msg.set_WindSpeed(self.rng.random())
        msg.set_WindDirection(self.rng.random())
        return msg

    def build_Wedge(self):
        msg = Wedge.Wedge()
        # Wedge fields
        msg.set_AzimuthCenterline(self.rng.random())
        msg.set_VerticalCenterline(self.rng.random())
        msg.set_AzimuthExtent(self.rng.random())
        msg.set_VerticalExtent(self.rng.random())
        return msg

    def build_AreaSearchTask(self):
        msg = AreaSearchTask.AreaSearchTask()
        # Task fields
        msg.set_TaskID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Label(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        v = msg.get_EligibleEntities()
        for i in range(self.rng.randint(0, min(self.limit, 32))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_RevisitRate(self.rng.random())
        v = msg.get_Parameters()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.build_KeyValuePair())
        msg.set_Priority(self.rng.getrandbits(8))
        msg.set_Required(self.rng.choice([True,False]))
        # SearchTask fields
        v = msg.get_DesiredWavelengthBands()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(WavelengthBand.get_WavelengthBand_str(WavelengthBand.get_WavelengthBand_int(self.rng.choice([0,1,2,3,4,5]))))
        msg.set_DwellTime(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_GroundSampleDistance(self.rng.random())
        # AreaSearchTask fields
        msg.set_SearchArea(self.options_AbstractGeometry()[self.rng.randint(0, 3)]())
        v = msg.get_ViewAngleList()
        for i in range(self.rng.randint(0, min(self.limit, 16))):
            v.append(self.build_Wedge())
        return msg

    def build_CameraAction(self):
        msg = CameraAction.CameraAction()
        # VehicleAction fields
        v = msg.get_AssociatedTaskList()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        # PayloadAction fields
        msg.set_PayloadID(self.rng.randint(-(2**63-1), 2**63-1))
        # CameraAction fields
        msg.set_HorizontalFieldOfView(self.rng.random())
        return msg

    def build_CameraConfiguration(self):
        msg = CameraConfiguration.CameraConfiguration()
        # PayloadConfiguration fields
        msg.set_PayloadID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_PayloadKind(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        v = msg.get_Parameters()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.build_KeyValuePair())
        # CameraConfiguration fields
        msg.set_SupportedWavelengthBand(WavelengthBand.get_WavelengthBand_str(WavelengthBand.get_WavelengthBand_int(self.rng.choice([0,1,2,3,4,5]))))
        msg.set_FieldOfViewMode(FOVOperationMode.get_FOVOperationMode_str(FOVOperationMode.get_FOVOperationMode_int(self.rng.choice([0,1]))))
        msg.set_MinHorizontalFieldOfView(self.rng.random())
        msg.set_MaxHorizontalFieldOfView(self.rng.random())
        v = msg.get_DiscreteHorizontalFieldOfViewList()
        for i in range(self.rng.randint(0, min(self.limit, 16))):
            v.append(self.rng.random())
        msg.set_VideoStreamHorizontalResolution(self.rng.getrandbits(32))
        msg.set_VideoStreamVerticalResolution(self.rng.getrandbits(32))
        return msg

    def build_GimballedPayloadState(self):
        msg = GimballedPayloadState.GimballedPayloadState()
        # PayloadState fields
        msg.set_PayloadID(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_Parameters()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.build_KeyValuePair())
        # GimballedPayloadState fields
        msg.set_PointingMode(GimbalPointingMode.get_GimbalPointingMode_str(GimbalPointingMode.get_GimbalPointingMode_int(self.rng.choice([0,1,2,3,4,5,6]))))
        msg.set_Azimuth(self.rng.random())
        msg.set_Elevation(self.rng.random())
        msg.set_Rotation(self.rng.random())
        return msg

    def build_CameraState(self):
        msg = CameraState.CameraState()
        # PayloadState fields
        msg.set_PayloadID(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_Parameters()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.build_KeyValuePair())
        # GimballedPayloadState fields
        msg.set_PointingMode(GimbalPointingMode.get_GimbalPointingMode_str(GimbalPointingMode.get_GimbalPointingMode_int(self.rng.choice([0,1,2,3,4,5,6]))))
        msg.set_Azimuth(self.rng.random())
        msg.set_Elevation(self.rng.random())
        msg.set_Rotation(self.rng.random())
        # CameraState fields
        msg.set_HorizontalFieldOfView(self.rng.random())
        msg.set_VerticalFieldOfView(self.rng.random())
        v = msg.get_Footprint()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.options_Location3D()[self.rng.randint(0, 2)]())
        msg.set_Centerpoint(self.options_Location3D()[self.rng.randint(0, 2)]() if self.rng.randint(0, 1) else None)
        return msg

    def build_Circle(self):
        msg = Circle.Circle()
        # AbstractGeometry fields
        # Circle fields
        msg.set_CenterPoint(self.options_Location3D()[self.rng.randint(0, 2)]())
        msg.set_Radius(self.rng.random())
        return msg

    def build_GimbalAngleAction(self):
        msg = GimbalAngleAction.GimbalAngleAction()
        # VehicleAction fields
        v = msg.get_AssociatedTaskList()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        # PayloadAction fields
        msg.set_PayloadID(self.rng.randint(-(2**63-1), 2**63-1))
        # GimbalAngleAction fields
        msg.set_Azimuth(self.rng.random())
        msg.set_Elevation(self.rng.random())
        msg.set_Rotation(self.rng.random())
        return msg

    def build_GimbalConfiguration(self):
        msg = GimbalConfiguration.GimbalConfiguration()
        # PayloadConfiguration fields
        msg.set_PayloadID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_PayloadKind(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        v = msg.get_Parameters()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.build_KeyValuePair())
        # GimbalConfiguration fields
        v = msg.get_SupportedPointingModes()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(GimbalPointingMode.get_GimbalPointingMode_str(GimbalPointingMode.get_GimbalPointingMode_int(self.rng.choice([0,1,2,3,4,5,6]))))
        msg.set_MinAzimuth(self.rng.random())
        msg.set_MaxAzimuth(self.rng.random())
        msg.set_IsAzimuthClamped(self.rng.choice([True,False]))
        msg.set_MinElevation(self.rng.random())
        msg.set_MaxElevation(self.rng.random())
        msg.set_IsElevationClamped(self.rng.choice([True,False]))
        msg.set_MinRotation(self.rng.random())
        msg.set_MaxRotation(self.rng.random())
        msg.set_IsRotationClamped(self.rng.choice([True,False]))
        msg.set_MaxAzimuthSlewRate(self.rng.random())
        msg.set_MaxElevationSlewRate(self.rng.random())
        msg.set_MaxRotationRate(self.rng.random())
        v = msg.get_ContainedPayloadList()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        return msg

    def build_GimbalScanAction(self):
        msg = GimbalScanAction.GimbalScanAction()
        # VehicleAction fields
        v = msg.get_AssociatedTaskList()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        # PayloadAction fields
        msg.set_PayloadID(self.rng.randint(-(2**63-1), 2**63-1))
        # GimbalScanAction fields
        msg.set_AzimuthSlewRate(self.rng.random())
        msg.set_ElevationSlewRate(self.rng.random())
        msg.set_StartAzimuth(self.rng.random())
        msg.set_EndAzimuth(self.rng.random())
        msg.set_StartElevation(self.rng.random())
        msg.set_EndElevation(self.rng.random())
        msg.set_Cycles(self.rng.getrandbits(32))
        return msg

    def build_GimbalStareAction(self):
        msg = GimbalStareAction.GimbalStareAction()
        # VehicleAction fields
        v = msg.get_AssociatedTaskList()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        # PayloadAction fields
        msg.set_PayloadID(self.rng.randint(-(2**63-1), 2**63-1))
        # GimbalStareAction fields
        msg.set_Starepoint(self.options_Location3D()[self.rng.randint(0, 2)]())
        msg.set_Duration(self.rng.randint(-(2**63-1), 2**63-1))
        return msg

    def build_GimbalState(self):
        msg = GimbalState.GimbalState()
        # PayloadState fields
        msg.set_PayloadID(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_Parameters()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.build_KeyValuePair())
        # GimbalState fields
        msg.set_PointingMode(GimbalPointingMode.get_GimbalPointingMode_str(GimbalPointingMode.get_GimbalPointingMode_int(self.rng.choice([0,1,2,3,4,5,6]))))
        msg.set_Azimuth(self.rng.random())
        msg.set_Elevation(self.rng.random())
        msg.set_Rotation(self.rng.random())
        return msg

    def build_GoToWaypointAction(self):
        msg = GoToWaypointAction.GoToWaypointAction()
        # VehicleAction fields
        v = msg.get_AssociatedTaskList()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        # NavigationAction fields
        # GoToWaypointAction fields
        msg.set_WaypointNumber(self.rng.randint(-(2**63-1), 2**63-1))
        return msg

    def build_KeepInZone(self):
        msg = KeepInZone.KeepInZone()
        # AbstractZone fields
        msg.set_ZoneID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_MinAltitude(self.rng.random())
        msg.set_MinAltitudeType(AltitudeType.get_AltitudeType_str(AltitudeType.get_AltitudeType_int(self.rng.choice([0,1]))))
        msg.set_MaxAltitude(self.rng.random())
        msg.set_MaxAltitudeType(AltitudeType.get_AltitudeType_str(AltitudeType.get_AltitudeType_int(self.rng.choice([0,1]))))
        v = msg.get_AffectedAircraft()
        for i in range(self.rng.randint(0, min(self.limit, 32))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_StartTime(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_EndTime(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Padding(self.rng.random())
        msg.set_Label(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        msg.set_Boundary(self.options_AbstractGeometry()[self.rng.randint(0, 3)]())
        # KeepInZone fields
        return msg

    def build_KeepOutZone(self):
        msg = KeepOutZone.KeepOutZone()
        # AbstractZone fields
        msg.set_ZoneID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_MinAltitude(self.rng.random())
        msg.set_MinAltitudeType(AltitudeType.get_AltitudeType_str(AltitudeType.get_AltitudeType_int(self.rng.choice([0,1]))))
        msg.set_MaxAltitude(self.rng.random())
        msg.set_MaxAltitudeType(AltitudeType.get_AltitudeType_str(AltitudeType.get_AltitudeType_int(self.rng.choice([0,1]))))
        v = msg.get_AffectedAircraft()
        for i in range(self.rng.randint(0, min(self.limit, 32))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_StartTime(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_EndTime(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Padding(self.rng.random())
        msg.set_Label(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        msg.set_Boundary(self.options_AbstractGeometry()[self.rng.randint(0, 3)]())
        # KeepOutZone fields
        msg.set_ZoneType(ZoneAvoidanceType.get_ZoneAvoidanceType_str(ZoneAvoidanceType.get_ZoneAvoidanceType_int(self.rng.choice([1,2,3,4,5]))))
        return msg

    def build_LineSearchTask(self):
        msg = LineSearchTask.LineSearchTask()
        # Task fields
        msg.set_TaskID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Label(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        v = msg.get_EligibleEntities()
        for i in range(self.rng.randint(0, min(self.limit, 32))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_RevisitRate(self.rng.random())
        v = msg.get_Parameters()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.build_KeyValuePair())
        msg.set_Priority(self.rng.getrandbits(8))
        msg.set_Required(self.rng.choice([True,False]))
        # SearchTask fields
        v = msg.get_DesiredWavelengthBands()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(WavelengthBand.get_WavelengthBand_str(WavelengthBand.get_WavelengthBand_int(self.rng.choice([0,1,2,3,4,5]))))
        msg.set_DwellTime(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_GroundSampleDistance(self.rng.random())
        # LineSearchTask fields
        v = msg.get_PointList()
        for i in range(self.rng.randint(0, min(self.limit, 1024))):
            v.append(self.options_Location3D()[self.rng.randint(0, 2)]())
        v = msg.get_ViewAngleList()
        for i in range(self.rng.randint(0, min(self.limit, 16))):
            v.append(self.build_Wedge())
        msg.set_UseInertialViewAngles(self.rng.choice([True,False]))
        return msg

    def build_NavigationAction(self):
        msg = NavigationAction.NavigationAction()
        # VehicleAction fields
        v = msg.get_AssociatedTaskList()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        # NavigationAction fields
        return msg

    def build_LoiterAction(self):
        msg = LoiterAction.LoiterAction()
        # VehicleAction fields
        v = msg.get_AssociatedTaskList()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        # NavigationAction fields
        # LoiterAction fields
        msg.set_LoiterType(LoiterType.get_LoiterType_str(LoiterType.get_LoiterType_int(self.rng.choice([0,1,2,3,4]))))
        msg.set_Radius(self.rng.random())
        msg.set_Axis(self.rng.random())
        msg.set_Length(self.rng.random())
        msg.set_Direction(LoiterDirection.get_LoiterDirection_str(LoiterDirection.get_LoiterDirection_int(self.rng.choice([0,1,2]))))
        msg.set_Duration(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Airspeed(self.rng.random())
        msg.set_Location(self.options_Location3D()[self.rng.randint(0, 2)]())
        return msg

    def build_LoiterTask(self):
        msg = LoiterTask.LoiterTask()
        # Task fields
        msg.set_TaskID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Label(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        v = msg.get_EligibleEntities()
        for i in range(self.rng.randint(0, min(self.limit, 32))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_RevisitRate(self.rng.random())
        v = msg.get_Parameters()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.build_KeyValuePair())
        msg.set_Priority(self.rng.getrandbits(8))
        msg.set_Required(self.rng.choice([True,False]))
        # LoiterTask fields
        msg.set_DesiredAction(self.build_LoiterAction())
        return msg

    def build_Waypoint(self):
        msg = Waypoint.Waypoint()
        # Location3D fields
        msg.set_Latitude(self.rng.getrandbits(64))
        msg.set_Longitude(self.rng.getrandbits(64))
        msg.set_Altitude(self.rng.random())
        msg.set_AltitudeType(AltitudeType.get_AltitudeType_str(AltitudeType.get_AltitudeType_int(self.rng.choice([0,1]))))
        # Waypoint fields
        msg.set_Number(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_NextWaypoint(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Speed(self.rng.random())
        msg.set_SpeedType(SpeedType.get_SpeedType_str(SpeedType.get_SpeedType_int(self.rng.choice([0,1]))))
        msg.set_ClimbRate(self.rng.random())
        msg.set_TurnType(TurnType.get_TurnType_str(TurnType.get_TurnType_int(self.rng.choice([0,1]))))
        v = msg.get_VehicleActionList()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.options_VehicleAction()[self.rng.randint(0, 15)]())
        msg.set_ContingencyWaypointA(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_ContingencyWaypointB(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_AssociatedTasks()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        return msg

    def build_MissionCommand(self):
        msg = MissionCommand.MissionCommand()
        # VehicleActionCommand fields
        msg.set_CommandID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_VehicleID(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_VehicleActionList()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.options_VehicleAction()[self.rng.randint(0, 15)]())
        msg.set_Status(CommandStatusType.get_CommandStatusType_str(CommandStatusType.get_CommandStatusType_int(self.rng.choice([0,1,2,3,4]))))
        # MissionCommand fields
        v = msg.get_WaypointList()
        for i in range(self.rng.randint(0, min(self.limit, 1024))):
            v.append(self.options_Waypoint()[self.rng.randint(0, 1)]())
        msg.set_FirstWaypoint(self.rng.randint(-(2**63-1), 2**63-1))
        return msg

    def build_MustFlyTask(self):
        msg = MustFlyTask.MustFlyTask()
        # Task fields
        msg.set_TaskID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Label(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        v = msg.get_EligibleEntities()
        for i in range(self.rng.randint(0, min(self.limit, 32))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_RevisitRate(self.rng.random())
        v = msg.get_Parameters()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.build_KeyValuePair())
        msg.set_Priority(self.rng.getrandbits(8))
        msg.set_Required(self.rng.choice([True,False]))
        # MustFlyTask fields
        msg.set_Position(self.options_Location3D()[self.rng.randint(0, 2)]())
        msg.set_UseAltitude(self.rng.choice([True,False]))
        return msg

    def build_OperatorSignal(self):
        msg = OperatorSignal.OperatorSignal()
        # OperatorSignal fields
        v = msg.get_Signals()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.build_KeyValuePair())
        return msg

    def build_OperatingRegion(self):
        msg = OperatingRegion.OperatingRegion()
        # OperatingRegion fields
        msg.set_ID(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_KeepInAreas()
        for i in range(self.rng.randint(0, min(self.limit, 32))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_KeepOutAreas()
        for i in range(self.rng.randint(0, min(self.limit, 32))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        return msg

    def build_AutomationRequest(self):
        msg = AutomationRequest.AutomationRequest()
        # AutomationRequest fields
        v = msg.get_EntityList()
        for i in range(self.rng.randint(0, min(self.limit, 16))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_TaskList()
        for i in range(self.rng.randint(0, min(self.limit, 32))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_TaskRelationships(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        msg.set_OperatingRegion(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_RedoAllTasks(self.rng.choice([True,False]))
        return msg

    def build_PointSearchTask(self):
        msg = PointSearchTask.PointSearchTask()
        # Task fields
        msg.set_TaskID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Label(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        v = msg.get_EligibleEntities()
        for i in range(self.rng.randint(0, min(self.limit, 32))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_RevisitRate(self.rng.random())
        v = msg.get_Parameters()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.build_KeyValuePair())
        msg.set_Priority(self.rng.getrandbits(8))
        msg.set_Required(self.rng.choice([True,False]))
        # SearchTask fields
        v = msg.get_DesiredWavelengthBands()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(WavelengthBand.get_WavelengthBand_str(WavelengthBand.get_WavelengthBand_int(self.rng.choice([0,1,2,3,4,5]))))
        msg.set_DwellTime(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_GroundSampleDistance(self.rng.random())
        # PointSearchTask fields
        msg.set_SearchLocation(self.options_Location3D()[self.rng.randint(0, 2)]())
        msg.set_StandoffDistance(self.rng.random())
        v = msg.get_ViewAngleList()
        for i in range(self.rng.randint(0, min(self.limit, 64))):
            v.append(self.build_Wedge())
        return msg

    def build_Polygon(self):
        msg = Polygon.Polygon()
        # AbstractGeometry fields
        # Polygon fields
        v = msg.get_BoundaryPoints()
        for i in range(self.rng.randint(0, min(self.limit, 64))):
            v.append(self.options_Location3D()[self.rng.randint(0, 2)]())
        return msg

    def build_Rectangle(self):
        msg = Rectangle.Rectangle()
        # AbstractGeometry fields
        # Rectangle fields
        msg.set_CenterPoint(self.options_Location3D()[self.rng.randint(0, 2)]())
        msg.set_Width(self.rng.random())
        msg.set_Height(self.rng.random())
        msg.set_Rotation(self.rng.random())
        return msg

    def build_RemoveTasks(self):
        msg = RemoveTasks.RemoveTasks()
        # RemoveTasks fields
        v = msg.get_TaskList()
        for i in range(self.rng.randint(0, min(self.limit, 32))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        return msg

    def build_ServiceStatus(self):
        msg = ServiceStatus.ServiceStatus()
        # ServiceStatus fields
        msg.set_PercentComplete(self.rng.random())
        v = msg.get_Info()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.build_KeyValuePair())
        msg.set_StatusType(ServiceStatusType.get_ServiceStatusType_str(ServiceStatusType.get_ServiceStatusType_int(self.rng.choice([0,1,2]))))
        return msg

    def build_SessionStatus(self):
        msg = SessionStatus.SessionStatus()
        # SessionStatus fields
        msg.set_State(SimulationStatusType.get_SimulationStatusType_str(SimulationStatusType.get_SimulationStatusType_int(self.rng.choice([0,1,2,3]))))
        msg.set_StartTime(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_ScenarioTime(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_RealTimeMultiple(self.rng.random())
        v = msg.get_Parameters()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.build_KeyValuePair())
        return msg

    def build_VehicleActionCommand(self):
        msg = VehicleActionCommand.VehicleActionCommand()
        # VehicleActionCommand fields
        msg.set_CommandID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_VehicleID(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_VehicleActionList()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.options_VehicleAction()[self.rng.randint(0, 15)]())
        msg.set_Status(CommandStatusType.get_CommandStatusType_str(CommandStatusType.get_CommandStatusType_int(self.rng.choice([0,1,2,3,4]))))
        return msg

    def build_VideoStreamAction(self):
        msg = VideoStreamAction.VideoStreamAction()
        # VehicleAction fields
        v = msg.get_AssociatedTaskList()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        # VideoStreamAction fields
        msg.set_VideoStreamID(self.rng.randint(-(2**31-1), 2**31-1))
        msg.set_ActiveSensor(self.rng.randint(-(2**31-1), 2**31-1))
        return msg

    def build_VideoStreamConfiguration(self):
        msg = VideoStreamConfiguration.VideoStreamConfiguration()
        # PayloadConfiguration fields
        msg.set_PayloadID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_PayloadKind(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        v = msg.get_Parameters()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.build_KeyValuePair())
        # VideoStreamConfiguration fields
        v = msg.get_AvailableSensorList()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        return msg

    def build_VideoStreamState(self):
        msg = VideoStreamState.VideoStreamState()
        # PayloadState fields
        msg.set_PayloadID(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_Parameters()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.build_KeyValuePair())
        # VideoStreamState fields
        msg.set_ActiveSensor(self.rng.randint(-(2**63-1), 2**63-1))
        return msg

    def build_AutomationResponse(self):
        msg = AutomationResponse.AutomationResponse()
        # AutomationResponse fields
        v = msg.get_MissionCommandList()
        for i in range(self.rng.randint(0, min(self.limit, 16))):
            v.append(self.build_MissionCommand())
        v = msg.get_VehicleCommandList()
        for i in range(self.rng.randint(0, min(self.limit, 64))):
            v.append(self.options_VehicleActionCommand()[self.rng.randint(0, 2)]())
        v = msg.get_Info()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.build_KeyValuePair())
        return msg

    def build_RemoveZones(self):
        msg = RemoveZones.RemoveZones()
        # RemoveZones fields
        v = msg.get_ZoneList()
        for i in range(self.rng.randint(0, min(self.limit, 32))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        return msg

    def build_RemoveEntities(self):
        msg = RemoveEntities.RemoveEntities()
        # RemoveEntities fields
        v = msg.get_EntityList()
        for i in range(self.rng.randint(0, min(self.limit, 32))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        return msg

    def build_FlightDirectorAction(self):
        msg = FlightDirectorAction.FlightDirectorAction()
        # VehicleAction fields
        v = msg.get_AssociatedTaskList()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        # NavigationAction fields
        # FlightDirectorAction fields
        msg.set_Speed(self.rng.random())
        msg.set_SpeedType(SpeedType.get_SpeedType_str(SpeedType.get_SpeedType_int(self.rng.choice([0,1]))))
        msg.set_Heading(self.rng.random())
        msg.set_Altitude(self.rng.random())
        msg.set_AltitudeType(AltitudeType.get_AltitudeType_str(AltitudeType.get_AltitudeType_int(self.rng.choice([0,1]))))
        msg.set_ClimbRate(self.rng.random())
        return msg

    def build_WeatherReport(self):
        msg = WeatherReport.WeatherReport()
        # WeatherReport fields
        msg.set_Area(self.options_AbstractZone()[self.rng.randint(0, 3)]() if self.rng.randint(0, 1) else None)
        msg.set_WindSpeed(self.rng.random())
        msg.set_WindDirection(self.rng.random())
        msg.set_Visibility(self.rng.random())
        msg.set_CloudCeiling(self.rng.random())
        msg.set_CloudCoverage(self.rng.random())
        return msg

    def build_FollowPathCommand(self):
        msg = FollowPathCommand.FollowPathCommand()
        # VehicleActionCommand fields
        msg.set_CommandID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_VehicleID(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_VehicleActionList()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.options_VehicleAction()[self.rng.randint(0, 15)]())
        msg.set_Status(CommandStatusType.get_CommandStatusType_str(CommandStatusType.get_CommandStatusType_int(self.rng.choice([0,1,2,3,4]))))
        # FollowPathCommand fields
        msg.set_FirstWaypoint(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_WaypointList()
        for i in range(self.rng.randint(0, min(self.limit, 1024))):
            v.append(self.build_PathWaypoint())
        msg.set_StartTime(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_StopTime(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_RepeatMode(TravelMode.get_TravelMode_str(TravelMode.get_TravelMode_int(self.rng.choice([0,1,2]))))
        return msg

    def build_PathWaypoint(self):
        msg = PathWaypoint.PathWaypoint()
        # Location3D fields
        msg.set_Latitude(self.rng.getrandbits(64))
        msg.set_Longitude(self.rng.getrandbits(64))
        msg.set_Altitude(self.rng.random())
        msg.set_AltitudeType(AltitudeType.get_AltitudeType_str(AltitudeType.get_AltitudeType_int(self.rng.choice([0,1]))))
        # Waypoint fields
        msg.set_Number(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_NextWaypoint(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Speed(self.rng.random())
        msg.set_SpeedType(SpeedType.get_SpeedType_str(SpeedType.get_SpeedType_int(self.rng.choice([0,1]))))
        msg.set_ClimbRate(self.rng.random())
        msg.set_TurnType(TurnType.get_TurnType_str(TurnType.get_TurnType_int(self.rng.choice([0,1]))))
        v = msg.get_VehicleActionList()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.options_VehicleAction()[self.rng.randint(0, 15)]())
        msg.set_ContingencyWaypointA(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_ContingencyWaypointB(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_AssociatedTasks()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        # PathWaypoint fields
        msg.set_PauseTime(self.rng.randint(-(2**63-1), 2**63-1))
        return msg

    def build_StopMovementAction(self):
        msg = StopMovementAction.StopMovementAction()
        # VehicleAction fields
        v = msg.get_AssociatedTaskList()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        # StopMovementAction fields
        msg.set_Location(self.options_Location3D()[self.rng.randint(0, 2)]() if self.rng.randint(0, 1) else None)
        return msg

    def build_WaypointTransfer(self):
        msg = WaypointTransfer.WaypointTransfer()
        # WaypointTransfer fields
        msg.set_EntityID(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_Waypoints()
        for i in range(self.rng.randint(0, min(self.limit, 1024))):
            v.append(self.options_Waypoint()[self.rng.randint(0, 1)]())
        msg.set_TransferMode(WaypointTransferMode.get_WaypointTransferMode_str(WaypointTransferMode.get_WaypointTransferMode_int(self.rng.choice([0,1,2,3]))))
        return msg

    def build_PayloadStowAction(self):
        msg = PayloadStowAction.PayloadStowAction()
        # PayloadStowAction fields
        msg.set_PayloadID(self.rng.randint(-(2**63-1), 2**63-1))
        return msg

    def build_PowerConfiguration(self):
        msg = PowerConfiguration.PowerConfiguration()
        # PayloadConfiguration fields
        msg.set_PayloadID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_PayloadKind(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        v = msg.get_Parameters()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.build_KeyValuePair())
        # PowerConfiguration fields
        msg.set_NominalPowerConfiguration(PowerPlant.get_PowerPlant_str(PowerPlant.get_PowerPlant_int(self.rng.choice([0,1,2,3,4,5]))))
        return msg

    def build_RadioConfiguration(self):
        msg = RadioConfiguration.RadioConfiguration()
        # PayloadConfiguration fields
        msg.set_PayloadID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_PayloadKind(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        v = msg.get_Parameters()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.build_KeyValuePair())
        # RadioConfiguration fields
        msg.set_Range(self.rng.random())
        msg.set_RallyPoint(self.options_Location3D()[self.rng.randint(0, 2)]() if self.rng.randint(0, 1) else None)
        msg.set_Timeout(self.rng.randint(-(2**63-1), 2**63-1))
        return msg

    def build_RadioTowerConfiguration(self):
        msg = RadioTowerConfiguration.RadioTowerConfiguration()
        # EntityConfiguration fields
        msg.set_ID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Affiliation(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        msg.set_EntityType(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        msg.set_Label(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        msg.set_NominalSpeed(self.rng.random())
        msg.set_NominalAltitude(self.rng.random())
        msg.set_NominalAltitudeType(AltitudeType.get_AltitudeType_str(AltitudeType.get_AltitudeType_int(self.rng.choice([0,1]))))
        v = msg.get_PayloadConfigurationList()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.options_PayloadConfiguration()[self.rng.randint(0, 6)]())
        v = msg.get_Info()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.build_KeyValuePair())
        # RadioTowerConfiguration fields
        msg.set_Position(self.options_Location3D()[self.rng.randint(0, 2)]())
        msg.set_Range(self.rng.random())
        msg.set_Enabled(self.rng.choice([True,False]))
        return msg

    def build_RadioState(self):
        msg = RadioState.RadioState()
        # PayloadState fields
        msg.set_PayloadID(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_Parameters()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.build_KeyValuePair())
        # RadioState fields
        msg.set_Enabled(self.rng.choice([True,False]))
        msg.set_InRange(self.rng.choice([True,False]))
        return msg

    def build_RadioTowerState(self):
        msg = RadioTowerState.RadioTowerState()
        # EntityState fields
        msg.set_ID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_u(self.rng.random())
        msg.set_v(self.rng.random())
        msg.set_w(self.rng.random())
        msg.set_udot(self.rng.random())
        msg.set_vdot(self.rng.random())
        msg.set_wdot(self.rng.random())
        msg.set_Heading(self.rng.random())
        msg.set_Pitch(self.rng.random())
        msg.set_Roll(self.rng.random())
        msg.set_p(self.rng.random())
        msg.set_q(self.rng.random())
        msg.set_r(self.rng.random())
        msg.set_Course(self.rng.random())
        msg.set_Groundspeed(self.rng.random())
        msg.set_Location(self.options_Location3D()[self.rng.randint(0, 2)]())
        msg.set_EnergyAvailable(self.rng.random())
        msg.set_ActualEnergyRate(self.rng.random())
        v = msg.get_PayloadStateList()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.options_PayloadState()[self.rng.randint(0, 6)]())
        msg.set_CurrentWaypoint(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_CurrentCommand(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Mode(NavigationMode.get_NavigationMode_str(NavigationMode.get_NavigationMode_int(self.rng.choice([0,1,2,3,4,5]))))
        v = msg.get_AssociatedTasks()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Time(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_Info()
        for i in range(self.rng.randint(0, min(self.limit, 32))):
            v.append(self.build_KeyValuePair())
        # RadioTowerState fields
        msg.set_Enabled(self.rng.choice([True,False]))
        return msg

    def build_ImpactPayloadConfiguration(self):
        msg = ImpactPayloadConfiguration.ImpactPayloadConfiguration()
        # PayloadConfiguration fields
        msg.set_PayloadID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_PayloadKind(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        v = msg.get_Parameters()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.build_KeyValuePair())
        # ImpactPayloadConfiguration fields
        v = msg.get_AvailablePayloads()
        for i in range(self.rng.randint(0, min(self.limit, 16))):
            v.append(ImpactPayloadType.get_ImpactPayloadType_str(ImpactPayloadType.get_ImpactPayloadType_int(self.rng.choice([0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20]))))
        return msg

    def build_DeployImpactPayload(self):
        msg = DeployImpactPayload.DeployImpactPayload()
        # VehicleAction fields
        v = msg.get_AssociatedTaskList()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        # DeployImpactPayload fields
        msg.set_VehicleID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_DeployedPayload(ImpactPayloadType.get_ImpactPayloadType_str(ImpactPayloadType.get_ImpactPayloadType_int(self.rng.choice([0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20]))))
        msg.set_TargetEntityID(self.rng.randint(-(2**63-1), 2**63-1))
        return msg

    def build_PowerPlantState(self):
        msg = PowerPlantState.PowerPlantState()
        # PayloadState fields
        msg.set_PayloadID(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_Parameters()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.build_KeyValuePair())
        # PowerPlantState fields
        msg.set_ActivePowerPlant(PowerPlant.get_PowerPlant_str(PowerPlant.get_PowerPlant_int(self.rng.choice([0,1,2,3,4,5]))))
        return msg

    def build_BatchRoutePlanRequest(self):
        msg = BatchRoutePlanRequest.BatchRoutePlanRequest()
        # BatchRoutePlanRequest fields
        msg.set_RequestID(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_Vehicles()
        for i in range(self.rng.randint(0, min(self.limit, 16))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_TaskList()
        for i in range(self.rng.randint(0, min(self.limit, 16))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_OperatingRegion(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_ComputeTaskToTaskTiming(self.rng.choice([True,False]))
        msg.set_ComputeInterTaskToTaskTiming(self.rng.choice([True,False]))
        v = msg.get_InterTaskPercentage()
        for i in range(self.rng.randint(0, min(self.limit, 32))):
            v.append(self.rng.random())
        return msg

    def build_BatchRoutePlanResponse(self):
        msg = BatchRoutePlanResponse.BatchRoutePlanResponse()
        # BatchRoutePlanResponse fields
        msg.set_ResponseID(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_VehicleTiming()
        for i in range(self.rng.randint(0, min(self.limit, 8192))):
            v.append(self.build_TaskTimingPair())
        return msg

    def build_TaskTimingPair(self):
        msg = TaskTimingPair.TaskTimingPair()
        # TaskTimingPair fields
        msg.set_VehicleID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_InitialTaskID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_InitialTaskPercentage(self.rng.random())
        msg.set_DestinationTaskID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_TimeToGo(self.rng.randint(-(2**63-1), 2**63-1))
        return msg

    def build_BatchSummaryRequest(self):
        msg = BatchSummaryRequest.BatchSummaryRequest()
        # BatchSummaryRequest fields
        msg.set_RequestID(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_Vehicles()
        for i in range(self.rng.randint(0, min(self.limit, 16))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_TaskList()
        for i in range(self.rng.randint(0, min(self.limit, 16))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_TaskRelationships(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        v = msg.get_InterTaskPercentage()
        for i in range(self.rng.randint(0, min(self.limit, 32))):
            v.append(self.rng.random())
        msg.set_OperatingRegion(self.rng.randint(-(2**63-1), 2**63-1))
        return msg

    def build_BatchSummaryResponse(self):
        msg = BatchSummaryResponse.BatchSummaryResponse()
        # BatchSummaryResponse fields
        msg.set_ResponseID(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_Summaries()
        for i in range(self.rng.randint(0, min(self.limit, 512))):
            v.append(self.build_TaskSummary())
        return msg

    def build_TaskSummary(self):
        msg = TaskSummary.TaskSummary()
        # TaskSummary fields
        msg.set_TaskID(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_PerformingVehicles()
        for i in range(self.rng.randint(0, min(self.limit, 16))):
            v.append(self.build_VehicleSummary())
        msg.set_BestEffort(self.rng.random())
        return msg

    def build_VehicleSummary(self):
        msg = VehicleSummary.VehicleSummary()
        # VehicleSummary fields
        msg.set_VehicleID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_DestinationTaskID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_InitialTaskID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_InitialTaskPercentage(self.rng.random())
        msg.set_EstimateTimeToTaskPercentage(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_TimeToArrive(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_TimeOnTask(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_EnergyRemaining(self.rng.random())
        msg.set_BeyondCommRange(self.rng.choice([True,False]))
        msg.set_ConflictsWithROZ(self.rng.choice([True,False]))
        v = msg.get_ROZIDs()
        for i in range(self.rng.randint(0, min(self.limit, 64))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_WaypointList()
        for i in range(self.rng.randint(0, min(self.limit, 1024))):
            v.append(self.options_Waypoint()[self.rng.randint(0, 1)]())
        msg.set_FirstWaypoint(self.rng.randint(-(2**63-1), 2**63-1))
        return msg

    def build_SpeedAltPair(self):
        msg = SpeedAltPair.SpeedAltPair()
        # SpeedAltPair fields
        msg.set_VehicleID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_TaskID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Speed(self.rng.random())
        msg.set_Altitude(self.rng.random())
        msg.set_AltitudeType(AltitudeType.get_AltitudeType_str(AltitudeType.get_AltitudeType_int(self.rng.choice([0,1]))))
        return msg

    def build_ImpactAutomationRequest(self):
        msg = ImpactAutomationRequest.ImpactAutomationRequest()
        # ImpactAutomationRequest fields
        msg.set_RequestID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_TrialRequest(self.build_AutomationRequest())
        v = msg.get_OverridePlanningConditions()
        for i in range(self.rng.randint(0, min(self.limit, 16))):
            v.append(self.build_SpeedAltPair())
        msg.set_PlayID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_SolutionID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Sandbox(self.rng.choice([True,False]))
        return msg

    def build_ImpactAutomationResponse(self):
        msg = ImpactAutomationResponse.ImpactAutomationResponse()
        # ImpactAutomationResponse fields
        msg.set_ResponseID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_TrialResponse(self.build_AutomationResponse())
        msg.set_PlayID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_SolutionID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Sandbox(self.rng.choice([True,False]))
        v = msg.get_Summaries()
        for i in range(self.rng.randint(0, min(self.limit, 512))):
            v.append(self.build_TaskSummary())
        return msg

    def build_PointOfInterest(self):
        msg = PointOfInterest.PointOfInterest()
        # PointOfInterest fields
        msg.set_PointID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Location(self.options_Location3D()[self.rng.randint(0, 2)]())
        msg.set_PointAction(AreaActionOptions.get_AreaActionOptions_str(AreaActionOptions.get_AreaActionOptions_int(self.rng.choice([0,1,2]))))
        msg.set_PointLabel(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        msg.set_BackgroundBehaviorPoint(self.rng.choice([True,False]))
        return msg

    def build_LineOfInterest(self):
        msg = LineOfInterest.LineOfInterest()
        # LineOfInterest fields
        msg.set_LineID(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_Line()
        for i in range(self.rng.randint(0, min(self.limit, 1024))):
            v.append(self.options_Location3D()[self.rng.randint(0, 2)]())
        msg.set_LineAction(AreaActionOptions.get_AreaActionOptions_str(AreaActionOptions.get_AreaActionOptions_int(self.rng.choice([0,1,2]))))
        msg.set_LineLabel(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        msg.set_BackgroundBehaviorLine(self.rng.choice([True,False]))
        return msg

    def build_AreaOfInterest(self):
        msg = AreaOfInterest.AreaOfInterest()
        # AreaOfInterest fields
        msg.set_AreaID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Area(self.options_AbstractGeometry()[self.rng.randint(0, 3)]())
        msg.set_AreaAction(AreaActionOptions.get_AreaActionOptions_str(AreaActionOptions.get_AreaActionOptions_int(self.rng.choice([0,1,2]))))
        msg.set_AreaLabel(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        msg.set_BackgroundBehaviorArea(self.rng.choice([True,False]))
        return msg

    def build_ImpactPointSearchTask(self):
        msg = ImpactPointSearchTask.ImpactPointSearchTask()
        # Task fields
        msg.set_TaskID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Label(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        v = msg.get_EligibleEntities()
        for i in range(self.rng.randint(0, min(self.limit, 32))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_RevisitRate(self.rng.random())
        v = msg.get_Parameters()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.build_KeyValuePair())
        msg.set_Priority(self.rng.getrandbits(8))
        msg.set_Required(self.rng.choice([True,False]))
        # SearchTask fields
        v = msg.get_DesiredWavelengthBands()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(WavelengthBand.get_WavelengthBand_str(WavelengthBand.get_WavelengthBand_int(self.rng.choice([0,1,2,3,4,5]))))
        msg.set_DwellTime(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_GroundSampleDistance(self.rng.random())
        # ImpactPointSearchTask fields
        msg.set_SearchLocationID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_SearchLocation(self.options_Location3D()[self.rng.randint(0, 2)]() if self.rng.randint(0, 1) else None)
        msg.set_StandoffDistance(self.rng.random())
        v = msg.get_ViewAngleList()
        for i in range(self.rng.randint(0, min(self.limit, 64))):
            v.append(self.build_Wedge())
        msg.set_DesiredAction(self.build_LoiterAction() if self.rng.randint(0, 1) else None)
        return msg

    def build_PatternSearchTask(self):
        msg = PatternSearchTask.PatternSearchTask()
        # Task fields
        msg.set_TaskID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Label(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        v = msg.get_EligibleEntities()
        for i in range(self.rng.randint(0, min(self.limit, 32))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_RevisitRate(self.rng.random())
        v = msg.get_Parameters()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.build_KeyValuePair())
        msg.set_Priority(self.rng.getrandbits(8))
        msg.set_Required(self.rng.choice([True,False]))
        # SearchTask fields
        v = msg.get_DesiredWavelengthBands()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(WavelengthBand.get_WavelengthBand_str(WavelengthBand.get_WavelengthBand_int(self.rng.choice([0,1,2,3,4,5]))))
        msg.set_DwellTime(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_GroundSampleDistance(self.rng.random())
        # PatternSearchTask fields
        msg.set_SearchLocationID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_SearchLocation(self.options_Location3D()[self.rng.randint(0, 2)]() if self.rng.randint(0, 1) else None)
        msg.set_Pattern(AreaSearchPattern.get_AreaSearchPattern_str(AreaSearchPattern.get_AreaSearchPattern_int(self.rng.choice([0,1,2]))))
        msg.set_Extent(self.rng.random())
        return msg

    def build_AngledAreaSearchTask(self):
        msg = AngledAreaSearchTask.AngledAreaSearchTask()
        # Task fields
        msg.set_TaskID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Label(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        v = msg.get_EligibleEntities()
        for i in range(self.rng.randint(0, min(self.limit, 32))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_RevisitRate(self.rng.random())
        v = msg.get_Parameters()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.build_KeyValuePair())
        msg.set_Priority(self.rng.getrandbits(8))
        msg.set_Required(self.rng.choice([True,False]))
        # SearchTask fields
        v = msg.get_DesiredWavelengthBands()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(WavelengthBand.get_WavelengthBand_str(WavelengthBand.get_WavelengthBand_int(self.rng.choice([0,1,2,3,4,5]))))
        msg.set_DwellTime(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_GroundSampleDistance(self.rng.random())
        # AngledAreaSearchTask fields
        msg.set_SearchAreaID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_SweepAngle(self.rng.random())
        msg.set_StartPoint(self.options_Location3D()[self.rng.randint(0, 2)]() if self.rng.randint(0, 1) else None)
        return msg

    def build_ImpactLineSearchTask(self):
        msg = ImpactLineSearchTask.ImpactLineSearchTask()
        # Task fields
        msg.set_TaskID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Label(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        v = msg.get_EligibleEntities()
        for i in range(self.rng.randint(0, min(self.limit, 32))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_RevisitRate(self.rng.random())
        v = msg.get_Parameters()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.build_KeyValuePair())
        msg.set_Priority(self.rng.getrandbits(8))
        msg.set_Required(self.rng.choice([True,False]))
        # SearchTask fields
        v = msg.get_DesiredWavelengthBands()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(WavelengthBand.get_WavelengthBand_str(WavelengthBand.get_WavelengthBand_int(self.rng.choice([0,1,2,3,4,5]))))
        msg.set_DwellTime(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_GroundSampleDistance(self.rng.random())
        # ImpactLineSearchTask fields
        msg.set_LineID(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_ViewAngleList()
        for i in range(self.rng.randint(0, min(self.limit, 64))):
            v.append(self.build_Wedge())
        msg.set_UseInertialViewAngles(self.rng.choice([True,False]))
        return msg

    def build_WatchTask(self):
        msg = WatchTask.WatchTask()
        # Task fields
        msg.set_TaskID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Label(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        v = msg.get_EligibleEntities()
        for i in range(self.rng.randint(0, min(self.limit, 32))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_RevisitRate(self.rng.random())
        v = msg.get_Parameters()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.build_KeyValuePair())
        msg.set_Priority(self.rng.getrandbits(8))
        msg.set_Required(self.rng.choice([True,False]))
        # SearchTask fields
        v = msg.get_DesiredWavelengthBands()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(WavelengthBand.get_WavelengthBand_str(WavelengthBand.get_WavelengthBand_int(self.rng.choice([0,1,2,3,4,5]))))
        msg.set_DwellTime(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_GroundSampleDistance(self.rng.random())
        # WatchTask fields
        msg.set_WatchedEntityID(self.rng.randint(-(2**63-1), 2**63-1))
        return msg

    def build_MultiVehicleWatchTask(self):
        msg = MultiVehicleWatchTask.MultiVehicleWatchTask()
        # Task fields
        msg.set_TaskID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Label(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        v = msg.get_EligibleEntities()
        for i in range(self.rng.randint(0, min(self.limit, 32))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_RevisitRate(self.rng.random())
        v = msg.get_Parameters()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.build_KeyValuePair())
        msg.set_Priority(self.rng.getrandbits(8))
        msg.set_Required(self.rng.choice([True,False]))
        # SearchTask fields
        v = msg.get_DesiredWavelengthBands()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(WavelengthBand.get_WavelengthBand_str(WavelengthBand.get_WavelengthBand_int(self.rng.choice([0,1,2,3,4,5]))))
        msg.set_DwellTime(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_GroundSampleDistance(self.rng.random())
        # MultiVehicleWatchTask fields
        msg.set_WatchedEntityID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_NumberVehicles(self.rng.getrandbits(8))
        return msg

    def build_CommRelayTask(self):
        msg = CommRelayTask.CommRelayTask()
        # Task fields
        msg.set_TaskID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Label(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        v = msg.get_EligibleEntities()
        for i in range(self.rng.randint(0, min(self.limit, 32))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_RevisitRate(self.rng.random())
        v = msg.get_Parameters()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.build_KeyValuePair())
        msg.set_Priority(self.rng.getrandbits(8))
        msg.set_Required(self.rng.choice([True,False]))
        # CommRelayTask fields
        msg.set_SupportedEntityID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_DestinationLocation(self.options_Location3D()[self.rng.randint(0, 2)]() if self.rng.randint(0, 1) else None)
        msg.set_TowerID(self.rng.randint(-(2**63-1), 2**63-1))
        return msg

    def build_CordonTask(self):
        msg = CordonTask.CordonTask()
        # Task fields
        msg.set_TaskID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Label(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        v = msg.get_EligibleEntities()
        for i in range(self.rng.randint(0, min(self.limit, 32))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_RevisitRate(self.rng.random())
        v = msg.get_Parameters()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.build_KeyValuePair())
        msg.set_Priority(self.rng.getrandbits(8))
        msg.set_Required(self.rng.choice([True,False]))
        # CordonTask fields
        msg.set_CordonLocation(self.options_Location3D()[self.rng.randint(0, 2)]())
        msg.set_StandoffDistance(self.rng.random())
        return msg

    def build_BlockadeTask(self):
        msg = BlockadeTask.BlockadeTask()
        # Task fields
        msg.set_TaskID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Label(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        v = msg.get_EligibleEntities()
        for i in range(self.rng.randint(0, min(self.limit, 32))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_RevisitRate(self.rng.random())
        v = msg.get_Parameters()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.build_KeyValuePair())
        msg.set_Priority(self.rng.getrandbits(8))
        msg.set_Required(self.rng.choice([True,False]))
        # BlockadeTask fields
        msg.set_BlockedEntityID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_StandoffDistance(self.rng.random())
        msg.set_NumberVehicles(self.rng.getrandbits(8))
        msg.set_ProtectedLocation(self.options_Location3D()[self.rng.randint(0, 2)]() if self.rng.randint(0, 1) else None)
        return msg

    def build_EscortTask(self):
        msg = EscortTask.EscortTask()
        # Task fields
        msg.set_TaskID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Label(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        v = msg.get_EligibleEntities()
        for i in range(self.rng.randint(0, min(self.limit, 32))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_RevisitRate(self.rng.random())
        v = msg.get_Parameters()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.build_KeyValuePair())
        msg.set_Priority(self.rng.getrandbits(8))
        msg.set_Required(self.rng.choice([True,False]))
        # SearchTask fields
        v = msg.get_DesiredWavelengthBands()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(WavelengthBand.get_WavelengthBand_str(WavelengthBand.get_WavelengthBand_int(self.rng.choice([0,1,2,3,4,5]))))
        msg.set_DwellTime(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_GroundSampleDistance(self.rng.random())
        # EscortTask fields
        msg.set_SupportedEntityID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_RouteID(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_PrescribedWaypoints()
        for i in range(self.rng.randint(0, min(self.limit, 1024))):
            v.append(self.options_Waypoint()[self.rng.randint(0, 1)]())
        msg.set_StandoffDistance(self.rng.random())
        return msg

    def build_ConfigurationRequest(self):
        msg = ConfigurationRequest.ConfigurationRequest()
        # ConfigurationRequest fields
        v = msg.get_VehicleID()
        for i in range(self.rng.randint(0, min(self.limit, 16))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        return msg

    def build_WaterReport(self):
        msg = WaterReport.WaterReport()
        # WaterReport fields
        msg.set_Area(self.options_AbstractGeometry()[self.rng.randint(0, 3)]())
        msg.set_CurrentSpeed(self.rng.random())
        msg.set_CurrentDirection(self.rng.random())
        msg.set_WaveDirection(self.rng.random())
        msg.set_WaveHeight(self.rng.random())
        return msg

    def build_WaterZone(self):
        msg = WaterZone.WaterZone()
        # AbstractZone fields
        msg.set_ZoneID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_MinAltitude(self.rng.random())
        msg.set_MinAltitudeType(AltitudeType.get_AltitudeType_str(AltitudeType.get_AltitudeType_int(self.rng.choice([0,1]))))
        msg.set_MaxAltitude(self.rng.random())
        msg.set_MaxAltitudeType(AltitudeType.get_AltitudeType_str(AltitudeType.get_AltitudeType_int(self.rng.choice([0,1]))))
        v = msg.get_AffectedAircraft()
        for i in range(self.rng.randint(0, min(self.limit, 32))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_StartTime(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_EndTime(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Padding(self.rng.random())
        msg.set_Label(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        msg.set_Boundary(self.options_AbstractGeometry()[self.rng.randint(0, 3)]())
        # WaterZone fields
        return msg

    def build_PayloadDropTask(self):
        msg = PayloadDropTask.PayloadDropTask()
        # Task fields
        msg.set_TaskID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Label(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        v = msg.get_EligibleEntities()
        for i in range(self.rng.randint(0, min(self.limit, 32))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_RevisitRate(self.rng.random())
        v = msg.get_Parameters()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.build_KeyValuePair())
        msg.set_Priority(self.rng.getrandbits(8))
        msg.set_Required(self.rng.choice([True,False]))
        # PayloadDropTask fields
        msg.set_DropLocation(self.options_Location3D()[self.rng.randint(0, 2)]())
        msg.set_BDALocation(self.options_Location3D()[self.rng.randint(0, 2)]())
        return msg

    def build_EntityPerception(self):
        msg = EntityPerception.EntityPerception()
        # EntityPerception fields
        msg.set_PerceivedEntityID(self.rng.getrandbits(32))
        msg.set_PerceiverID(self.rng.getrandbits(32))
        v = msg.get_PerceiverPayloads()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.rng.getrandbits(32))
        v = msg.get_Velocity()
        for i in range(3):
            v[i] = self.rng.random()
        v = msg.get_VelocityError()
        for i in range(3):
            v[i] = self.rng.random()
        msg.set_VelocityValid(self.rng.choice([True,False]))
        v = msg.get_Attitude()
        for i in range(3):
            v[i] = self.rng.random()
        v = msg.get_AttitudeError()
        for i in range(3):
            v[i] = self.rng.random()
        msg.set_AttitudeValid(self.rng.choice([True,False]))
        msg.set_Location(self.options_Location3D()[self.rng.randint(0, 2)]())
        v = msg.get_LocationError()
        for i in range(3):
            v[i] = self.rng.random()
        msg.set_TimeLastSeen(self.rng.randint(-(2**63-1), 2**63-1))
        return msg

    def build_TrackEntityAction(self):
        msg = TrackEntityAction.TrackEntityAction()
        # VehicleAction fields
        v = msg.get_AssociatedTaskList()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        # TrackEntityAction fields
        msg.set_EntityID(self.rng.getrandbits(32))
        msg.set_SensorID(self.rng.getrandbits(32))
        msg.set_ReturnToWaypoint(self.rng.getrandbits(32))
        return msg

    def build_TrackEntityTask(self):
        msg = TrackEntityTask.TrackEntityTask()
        # Task fields
        msg.set_TaskID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Label(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        v = msg.get_EligibleEntities()
        for i in range(self.rng.randint(0, min(self.limit, 32))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_RevisitRate(self.rng.random())
        v = msg.get_Parameters()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.build_KeyValuePair())
        msg.set_Priority(self.rng.getrandbits(8))
        msg.set_Required(self.rng.choice([True,False]))
        # TrackEntityTask fields
        msg.set_EntityID(self.rng.getrandbits(32))
        msg.set_SensorModality(WavelengthBand.get_WavelengthBand_str(WavelengthBand.get_WavelengthBand_int(self.rng.choice([0,1,2,3,4,5]))))
        msg.set_GroundSampleDistance(self.rng.random())
        return msg

    def build_GraphNode(self):
        msg = GraphNode.GraphNode()
        # GraphNode fields
        msg.set_NodeID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Coordinates(self.options_Location3D()[self.rng.randint(0, 2)]())
        v = msg.get_AssociatedEdges()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        return msg

    def build_GraphEdge(self):
        msg = GraphEdge.GraphEdge()
        # GraphEdge fields
        msg.set_EdgeID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_StartNode(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_EndNode(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_Waypoints()
        for i in range(self.rng.randint(0, min(self.limit, 1024))):
            v.append(self.options_Location3D()[self.rng.randint(0, 2)]())
        return msg

    def build_GraphRegion(self):
        msg = GraphRegion.GraphRegion()
        # GraphRegion fields
        msg.set_ID(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_NodeList()
        for i in range(self.rng.randint(0, min(self.limit, 16777216))):
            v.append(self.build_GraphNode())
        v = msg.get_EdgeList()
        for i in range(self.rng.randint(0, min(self.limit, 16777216))):
            v.append(self.build_GraphEdge())
        return msg

    def build_RouteConstraints(self):
        msg = RouteConstraints.RouteConstraints()
        # RouteConstraints fields
        msg.set_RouteID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_StartLocation(self.options_Location3D()[self.rng.randint(0, 2)]())
        msg.set_StartHeading(self.rng.random())
        msg.set_UseStartHeading(self.rng.choice([True,False]))
        msg.set_EndLocation(self.options_Location3D()[self.rng.randint(0, 2)]())
        msg.set_EndHeading(self.rng.random())
        msg.set_UseEndHeading(self.rng.choice([True,False]))
        return msg

    def build_RouteRequest(self):
        msg = RouteRequest.RouteRequest()
        # RouteRequest fields
        msg.set_RequestID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_AssociatedTaskID(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_VehicleID()
        for i in range(self.rng.randint(0, min(self.limit, 16))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_OperatingRegion(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_RouteRequests()
        for i in range(self.rng.randint(0, min(self.limit, 256))):
            v.append(self.build_RouteConstraints())
        msg.set_IsCostOnlyRequest(self.rng.choice([True,False]))
        return msg

    def build_RoutePlanRequest(self):
        msg = RoutePlanRequest.RoutePlanRequest()
        # RoutePlanRequest fields
        msg.set_RequestID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_AssociatedTaskID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_VehicleID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_OperatingRegion(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_RouteRequests()
        for i in range(self.rng.randint(0, min(self.limit, 256))):
            v.append(self.build_RouteConstraints())
        msg.set_IsCostOnlyRequest(self.rng.choice([True,False]))
        return msg

    def build_RoutePlan(self):
        msg = RoutePlan.RoutePlan()
        # RoutePlan fields
        msg.set_RouteID(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_Waypoints()
        for i in range(self.rng.randint(0, min(self.limit, 1024))):
            v.append(self.options_Waypoint()[self.rng.randint(0, 1)]())
        msg.set_RouteCost(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_RouteError()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.build_KeyValuePair())
        return msg

    def build_RoutePlanResponse(self):
        msg = RoutePlanResponse.RoutePlanResponse()
        # RoutePlanResponse fields
        msg.set_ResponseID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_AssociatedTaskID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_VehicleID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_OperatingRegion(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_RouteResponses()
        for i in range(self.rng.randint(0, min(self.limit, 256))):
            v.append(self.build_RoutePlan())
        return msg

    def build_RouteResponse(self):
        msg = RouteResponse.RouteResponse()
        # RouteResponse fields
        msg.set_ResponseID(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_Routes()
        for i in range(self.rng.randint(0, min(self.limit, 4096))):
            v.append(self.build_RoutePlanResponse())
        return msg

    def build_EgressRouteRequest(self):
        msg = EgressRouteRequest.EgressRouteRequest()
        # EgressRouteRequest fields
        msg.set_RequestID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_StartLocation(self.options_Location3D()[self.rng.randint(0, 2)]())
        msg.set_Radius(self.rng.random())
        return msg

    def build_EgressRouteResponse(self):
        msg = EgressRouteResponse.EgressRouteResponse()
        # EgressRouteResponse fields
        msg.set_ResponseID(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_NodeLocations()
        for i in range(self.rng.randint(0, min(self.limit, 32))):
            v.append(self.options_Location3D()[self.rng.randint(0, 2)]())
        v = msg.get_Headings()
        for i in range(self.rng.randint(0, min(self.limit, 32))):
            v.append(self.rng.random())
        return msg

    def build_RoadPointsConstraints(self):
        msg = RoadPointsConstraints.RoadPointsConstraints()
        # RoadPointsConstraints fields
        msg.set_RoadPointsID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_StartLocation(self.options_Location3D()[self.rng.randint(0, 2)]())
        msg.set_EndLocation(self.options_Location3D()[self.rng.randint(0, 2)]())
        return msg

    def build_RoadPointsRequest(self):
        msg = RoadPointsRequest.RoadPointsRequest()
        # RoadPointsRequest fields
        msg.set_RequestID(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_RoadPointsRequests()
        for i in range(self.rng.randint(0, min(self.limit, 0))):
            v.append(self.build_RoadPointsConstraints())
        return msg

    def build_RoadPointsResponse(self):
        msg = RoadPointsResponse.RoadPointsResponse()
        # RoadPointsResponse fields
        msg.set_ResponseID(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_RoadPointsResponses()
        for i in range(self.rng.randint(0, min(self.limit, 0))):
            v.append(self.build_LineOfInterest())
        return msg

    def build_VideoRecord(self):
        msg = VideoRecord.VideoRecord()
        # VideoRecord fields
        msg.set_Record(self.rng.choice([True,False]))
        return msg

    def build_StartupComplete(self):
        msg = StartupComplete.StartupComplete()
        # StartupComplete fields
        return msg

    def build_CreateNewService(self):
        msg = CreateNewService.CreateNewService()
        # CreateNewService fields
        msg.set_ServiceID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_XmlConfiguration(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        v = msg.get_EntityConfigurations()
        for i in range(self.rng.randint(0, min(self.limit, 16))):
            v.append(self.options_EntityConfiguration()[self.rng.randint(0, 5)]())
        v = msg.get_EntityStates()
        for i in range(self.rng.randint(0, min(self.limit, 16))):
            v.append(self.options_EntityState()[self.rng.randint(0, 5)]())
        v = msg.get_MissionCommands()
        for i in range(self.rng.randint(0, min(self.limit, 16))):
            v.append(self.build_MissionCommand())
        v = msg.get_Areas()
        for i in range(self.rng.randint(0, min(self.limit, 64))):
            v.append(self.build_AreaOfInterest())
        v = msg.get_Lines()
        for i in range(self.rng.randint(0, min(self.limit, 64))):
            v.append(self.build_LineOfInterest())
        v = msg.get_Points()
        for i in range(self.rng.randint(0, min(self.limit, 64))):
            v.append(self.build_PointOfInterest())
        v = msg.get_KeepInZones()
        for i in range(self.rng.randint(0, min(self.limit, 64))):
            v.append(self.build_KeepInZone())
        v = msg.get_KeepOutZones()
        for i in range(self.rng.randint(0, min(self.limit, 64))):
            v.append(self.build_KeepOutZone())
        v = msg.get_OperatingRegions()
        for i in range(self.rng.randint(0, min(self.limit, 32))):
            v.append(self.build_OperatingRegion())
        return msg

    def build_KillService(self):
        msg = KillService.KillService()
        # KillService fields
        msg.set_ServiceID(self.rng.randint(-(2**63-1), 2**63-1))
        return msg

    def build_IncrementWaypoint(self):
        msg = IncrementWaypoint.IncrementWaypoint()
        # IncrementWaypoint fields
        msg.set_VehicleID(self.rng.randint(-(2**63-1), 2**63-1))
        return msg

    def build_SafeHeadingAction(self):
        msg = SafeHeadingAction.SafeHeadingAction()
        # VehicleAction fields
        v = msg.get_AssociatedTaskList()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        # SafeHeadingAction fields
        msg.set_VehicleID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_OperatingRegion(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_LeadAheadDistance(self.rng.random())
        msg.set_LoiterRadius(self.rng.random())
        msg.set_DesiredHeading(self.rng.random())
        msg.set_DesiredHeadingRate(self.rng.random())
        msg.set_UseHeadingRate(self.rng.choice([True,False]))
        msg.set_Altitude(self.rng.random())
        msg.set_AltitudeType(AltitudeType.get_AltitudeType_str(AltitudeType.get_AltitudeType_int(self.rng.choice([0,1]))))
        msg.set_UseAltitude(self.rng.choice([True,False]))
        msg.set_Speed(self.rng.random())
        msg.set_UseSpeed(self.rng.choice([True,False]))
        return msg

    def build_EntityLocation(self):
        msg = EntityLocation.EntityLocation()
        # EntityLocation fields
        msg.set_EntityID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Position(self.options_Location3D()[self.rng.randint(0, 2)]())
        msg.set_Time(self.rng.randint(-(2**63-1), 2**63-1))
        return msg

    def build_BandwidthTest(self):
        msg = BandwidthTest.BandwidthTest()
        # EntityLocation fields
        msg.set_EntityID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Position(self.options_Location3D()[self.rng.randint(0, 2)]())
        msg.set_Time(self.rng.randint(-(2**63-1), 2**63-1))
        # BandwidthTest fields
        msg.set_MessageID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Payload(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        return msg

    def build_BandwidthReceiveReport(self):
        msg = BandwidthReceiveReport.BandwidthReceiveReport()
        # BandwidthReceiveReport fields
        msg.set_EntitySender(self.options_EntityLocation()[self.rng.randint(0, 1)]())
        msg.set_EntityReceiver(self.options_EntityLocation()[self.rng.randint(0, 1)]())
        msg.set_TransferPayloadSize(self.rng.getrandbits(32))
        return msg

    def build_SubTaskExecution(self):
        msg = SubTaskExecution.SubTaskExecution()
        # SubTaskExecution fields
        v = msg.get_SubTasks()
        for i in range(self.rng.randint(0, min(self.limit, 16))):
            v.append(self.options_Task()[self.rng.randint(0, 20)]())
        msg.set_StrictOrder(self.rng.choice([True,False]))
        return msg

    def build_SubTaskAssignment(self):
        msg = SubTaskAssignment.SubTaskAssignment()
        # SubTaskAssignment fields
        v = msg.get_SubTasks()
        for i in range(self.rng.randint(0, min(self.limit, 16))):
            v.append(self.options_Task()[self.rng.randint(0, 20)]())
        v = msg.get_Neighbors()
        for i in range(self.rng.randint(0, min(self.limit, 16))):
            v.append(self.options_EntityState()[self.rng.randint(0, 5)]())
        return msg

    def build_AutopilotKeepAlive(self):
        msg = AutopilotKeepAlive.AutopilotKeepAlive()
        # AutopilotKeepAlive fields
        msg.set_AutopilotEnabled(self.rng.choice([True,False]))
        msg.set_SpeedAuthorized(self.rng.choice([True,False]))
        msg.set_GimbalEnabled(self.rng.choice([True,False]))
        msg.set_TimeSent(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_SpeedOverride(self.rng.random())
        msg.set_AltOverride(self.rng.random())
        return msg

    def build_OnboardStatusReport(self):
        msg = OnboardStatusReport.OnboardStatusReport()
        # OnboardStatusReport fields
        msg.set_VehicleID(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_ConnectedEntities()
        for i in range(self.rng.randint(0, min(self.limit, 0))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_CurrentTaskList()
        for i in range(self.rng.randint(0, min(self.limit, 0))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_ValidState(self.rng.choice([True,False]))
        msg.set_ValidAuthorization(self.rng.choice([True,False]))
        msg.set_SpeedAuthorization(self.rng.choice([True,False]))
        msg.set_GimbalAuthorization(self.rng.choice([True,False]))
        msg.set_VehicleTime(self.rng.randint(-(2**63-1), 2**63-1))
        return msg

    def build_EntityJoin(self):
        msg = EntityJoin.EntityJoin()
        # EntityJoin fields
        msg.set_EntityID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Label(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        return msg

    def build_EntityExit(self):
        msg = EntityExit.EntityExit()
        # EntityExit fields
        msg.set_EntityID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Label(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        return msg

    def build_SimulationTimeStepAcknowledgement(self):
        msg = SimulationTimeStepAcknowledgement.SimulationTimeStepAcknowledgement()
        # SimulationTimeStepAcknowledgement fields
        msg.set_VehicleID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_ReportedTime(self.rng.randint(-(2**63-1), 2**63-1))
        return msg

    def build_SpeedOverrideAction(self):
        msg = SpeedOverrideAction.SpeedOverrideAction()
        # VehicleAction fields
        v = msg.get_AssociatedTaskList()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        # SpeedOverrideAction fields
        msg.set_VehicleID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Speed(self.rng.random())
        return msg

    def build_AssignmentCoordinatorTask(self):
        msg = AssignmentCoordinatorTask.AssignmentCoordinatorTask()
        # Task fields
        msg.set_TaskID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Label(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        v = msg.get_EligibleEntities()
        for i in range(self.rng.randint(0, min(self.limit, 32))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_RevisitRate(self.rng.random())
        v = msg.get_Parameters()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.build_KeyValuePair())
        msg.set_Priority(self.rng.getrandbits(8))
        msg.set_Required(self.rng.choice([True,False]))
        # AssignmentCoordinatorTask fields
        return msg

    def build_RendezvousTask(self):
        msg = RendezvousTask.RendezvousTask()
        # Task fields
        msg.set_TaskID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Label(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        v = msg.get_EligibleEntities()
        for i in range(self.rng.randint(0, min(self.limit, 32))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_RevisitRate(self.rng.random())
        v = msg.get_Parameters()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.build_KeyValuePair())
        msg.set_Priority(self.rng.getrandbits(8))
        msg.set_Required(self.rng.choice([True,False]))
        # RendezvousTask fields
        msg.set_NumberOfParticipants(self.rng.getrandbits(8))
        v = msg.get_RendezvousStates()
        for i in range(self.rng.randint(0, min(self.limit, 16))):
            v.append(self.build_PlanningState())
        return msg

    def build_PlanningState(self):
        msg = PlanningState.PlanningState()
        # PlanningState fields
        msg.set_EntityID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_PlanningPosition(self.options_Location3D()[self.rng.randint(0, 2)]())
        msg.set_PlanningHeading(self.rng.random())
        return msg

    def build_AssignmentCoordination(self):
        msg = AssignmentCoordination.AssignmentCoordination()
        # AssignmentCoordination fields
        msg.set_CoordinatedAutomationRequestID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_PlanningState(self.build_PlanningState())
        return msg

    def build_CoordinatedAutomationRequest(self):
        msg = CoordinatedAutomationRequest.CoordinatedAutomationRequest()
        # CoordinatedAutomationRequest fields
        msg.set_RequestID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_MaximumResponseTime(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_OriginalRequest(self.build_AutomationRequest())
        v = msg.get_PlanningStates()
        for i in range(self.rng.randint(0, min(self.limit, 16))):
            v.append(self.build_PlanningState())
        return msg

    def build_TaskAutomationRequest(self):
        msg = TaskAutomationRequest.TaskAutomationRequest()
        # TaskAutomationRequest fields
        msg.set_RequestID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_OriginalRequest(self.build_AutomationRequest())
        msg.set_SandBoxRequest(self.rng.choice([True,False]))
        v = msg.get_PlanningStates()
        for i in range(self.rng.randint(0, min(self.limit, 16))):
            v.append(self.build_PlanningState())
        return msg

    def build_TaskAutomationResponse(self):
        msg = TaskAutomationResponse.TaskAutomationResponse()
        # TaskAutomationResponse fields
        msg.set_ResponseID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_OriginalResponse(self.build_AutomationResponse())
        v = msg.get_FinalStates()
        for i in range(self.rng.randint(0, min(self.limit, 16))):
            v.append(self.build_PlanningState())
        return msg

    def build_UniqueAutomationRequest(self):
        msg = UniqueAutomationRequest.UniqueAutomationRequest()
        # UniqueAutomationRequest fields
        msg.set_RequestID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_OriginalRequest(self.build_AutomationRequest())
        msg.set_SandBoxRequest(self.rng.choice([True,False]))
        v = msg.get_PlanningStates()
        for i in range(self.rng.randint(0, min(self.limit, 16))):
            v.append(self.build_PlanningState())
        return msg

    def build_UniqueAutomationResponse(self):
        msg = UniqueAutomationResponse.UniqueAutomationResponse()
        # UniqueAutomationResponse fields
        msg.set_ResponseID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_OriginalResponse(self.build_AutomationResponse())
        v = msg.get_FinalStates()
        for i in range(self.rng.randint(0, min(self.limit, 16))):
            v.append(self.build_PlanningState())
        return msg

    def build_SensorFootprintRequests(self):
        msg = SensorFootprintRequests.SensorFootprintRequests()
        # SensorFootprintRequests fields
        msg.set_RequestID(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_Footprints()
        for i in range(self.rng.randint(0, min(self.limit, 16))):
            v.append(self.build_FootprintRequest())
        return msg

    def build_FootprintRequest(self):
        msg = FootprintRequest.FootprintRequest()
        # FootprintRequest fields
        msg.set_FootprintRequestID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_VehicleID(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_EligibleWavelengths()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(WavelengthBand.get_WavelengthBand_str(WavelengthBand.get_WavelengthBand_int(self.rng.choice([0,1,2,3,4,5]))))
        v = msg.get_GroundSampleDistances()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.rng.random())
        v = msg.get_AglAltitudes()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.rng.random())
        v = msg.get_ElevationAngles()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.rng.random())
        return msg

    def build_SensorFootprint(self):
        msg = SensorFootprint.SensorFootprint()
        # SensorFootprint fields
        msg.set_FootprintResponseID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_VehicleID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_CameraID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_GimbalID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_HorizontalFOV(self.rng.random())
        msg.set_AglAltitude(self.rng.random())
        msg.set_GimbalElevation(self.rng.random())
        msg.set_AspectRatio(self.rng.random())
        msg.set_AchievedGSD(self.rng.random())
        msg.set_CameraWavelength(WavelengthBand.get_WavelengthBand_str(WavelengthBand.get_WavelengthBand_int(self.rng.choice([0,1,2,3,4,5]))))
        msg.set_HorizontalToLeadingEdge(self.rng.random())
        msg.set_HorizontalToTrailingEdge(self.rng.random())
        msg.set_HorizontalToCenter(self.rng.random())
        msg.set_WidthCenter(self.rng.random())
        msg.set_SlantRangeToCenter(self.rng.random())
        return msg

    def build_SensorFootprintResponse(self):
        msg = SensorFootprintResponse.SensorFootprintResponse()
        # SensorFootprintResponse fields
        msg.set_ResponseID(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_Footprints()
        for i in range(self.rng.randint(0, min(self.limit, 65535))):
            v.append(self.build_SensorFootprint())
        return msg

    def build_TaskImplementationRequest(self):
        msg = TaskImplementationRequest.TaskImplementationRequest()
        # TaskImplementationRequest fields
        msg.set_RequestID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_CorrespondingAutomationRequestID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_StartingWaypointID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_VehicleID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_StartPosition(self.options_Location3D()[self.rng.randint(0, 2)]())
        msg.set_StartHeading(self.rng.random())
        msg.set_StartTime(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_RegionID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_TaskID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_OptionID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_TimeThreshold(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_NeighborLocations()
        for i in range(self.rng.randint(0, min(self.limit, 65535))):
            v.append(self.build_PlanningState())
        return msg

    def build_TaskImplementationResponse(self):
        msg = TaskImplementationResponse.TaskImplementationResponse()
        # TaskImplementationResponse fields
        msg.set_ResponseID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_CorrespondingAutomationRequestID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_TaskID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_OptionID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_VehicleID(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_TaskWaypoints()
        for i in range(self.rng.randint(0, min(self.limit, 1024))):
            v.append(self.options_Waypoint()[self.rng.randint(0, 1)]())
        msg.set_FinalLocation(self.options_Location3D()[self.rng.randint(0, 2)]())
        msg.set_FinalHeading(self.rng.random())
        msg.set_FinalTime(self.rng.randint(-(2**63-1), 2**63-1))
        return msg

    def build_AssignmentCostMatrix(self):
        msg = AssignmentCostMatrix.AssignmentCostMatrix()
        # AssignmentCostMatrix fields
        msg.set_CorrespondingAutomationRequestID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_TaskLevelRelationship(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        v = msg.get_TaskList()
        for i in range(self.rng.randint(0, min(self.limit, 16))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_OperatingRegion(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_CostMatrix()
        for i in range(self.rng.randint(0, min(self.limit, 18432))):
            v.append(self.build_TaskOptionCost())
        return msg

    def build_TaskOptionCost(self):
        msg = TaskOptionCost.TaskOptionCost()
        # TaskOptionCost fields
        msg.set_VehicleID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_IntialTaskID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_IntialTaskOption(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_DestinationTaskID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_DestinationTaskOption(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_TimeToGo(self.rng.randint(-(2**63-1), 2**63-1))
        return msg

    def build_TaskAssignment(self):
        msg = TaskAssignment.TaskAssignment()
        # TaskAssignment fields
        msg.set_TaskID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_OptionID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_AssignedVehicle(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_TimeThreshold(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_TimeTaskCompleted(self.rng.randint(-(2**63-1), 2**63-1))
        return msg

    def build_TaskAssignmentSummary(self):
        msg = TaskAssignmentSummary.TaskAssignmentSummary()
        # TaskAssignmentSummary fields
        msg.set_CorrespondingAutomationRequestID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_OperatingRegion(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_TaskList()
        for i in range(self.rng.randint(0, min(self.limit, 256))):
            v.append(self.build_TaskAssignment())
        return msg

    def build_TaskOption(self):
        msg = TaskOption.TaskOption()
        # TaskOption fields
        msg.set_TaskID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_OptionID(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_EligibleEntities()
        for i in range(self.rng.randint(0, min(self.limit, 16))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Cost(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_StartLocation(self.options_Location3D()[self.rng.randint(0, 2)]())
        msg.set_StartHeading(self.rng.random())
        msg.set_EndLocation(self.options_Location3D()[self.rng.randint(0, 2)]())
        msg.set_EndHeading(self.rng.random())
        return msg

    def build_TaskPlanOptions(self):
        msg = TaskPlanOptions.TaskPlanOptions()
        # TaskPlanOptions fields
        msg.set_CorrespondingAutomationRequestID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_TaskID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Composition(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        v = msg.get_Options()
        for i in range(self.rng.randint(0, min(self.limit, 64))):
            v.append(self.build_TaskOption())
        return msg

    def build_TaskPause(self):
        msg = TaskPause.TaskPause()
        # TaskPause fields
        msg.set_TaskID(self.rng.randint(-(2**63-1), 2**63-1))
        return msg

    def build_TaskResume(self):
        msg = TaskResume.TaskResume()
        # TaskResume fields
        msg.set_TaskID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_RestartCompletely(self.rng.choice([True,False]))
        msg.set_ReAssign(self.build_TaskAssignment() if self.rng.randint(0, 1) else None)
        return msg

    def build_TaskProgress(self):
        msg = TaskProgress.TaskProgress()
        # TaskProgress fields
        msg.set_ResponseID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_TaskID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_PercentComplete(self.rng.random())
        v = msg.get_EntitiesEngaged()
        for i in range(self.rng.randint(0, min(self.limit, 16))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        return msg

    def build_TaskProgressRequest(self):
        msg = TaskProgressRequest.TaskProgressRequest()
        # TaskProgressRequest fields
        msg.set_RequestID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_TaskID(self.rng.randint(-(2**63-1), 2**63-1))
        return msg

    def build_TaskInitialized(self):
        msg = TaskInitialized.TaskInitialized()
        # TaskInitialized fields
        msg.set_TaskID(self.rng.randint(-(2**63-1), 2**63-1))
        return msg

    def build_TaskActive(self):
        msg = TaskActive.TaskActive()
        # TaskActive fields
        msg.set_TaskID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_EntityID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_TimeTaskActivated(self.rng.randint(-(2**63-1), 2**63-1))
        return msg

    def build_TaskComplete(self):
        msg = TaskComplete.TaskComplete()
        # TaskComplete fields
        msg.set_TaskID(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_EntitiesInvolved()
        for i in range(self.rng.randint(0, min(self.limit, 16))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_TimeTaskCompleted(self.rng.randint(-(2**63-1), 2**63-1))
        return msg

    def build_CancelTask(self):
        msg = CancelTask.CancelTask()
        # CancelTask fields
        v = msg.get_Vehicles()
        for i in range(self.rng.randint(0, min(self.limit, 16))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_CanceledTasks()
        for i in range(self.rng.randint(0, min(self.limit, 16))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        return msg

    def build_GroundVehicleConfiguration(self):
        msg = GroundVehicleConfiguration.GroundVehicleConfiguration()
        # EntityConfiguration fields
        msg.set_ID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Affiliation(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        msg.set_EntityType(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        msg.set_Label(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        msg.set_NominalSpeed(self.rng.random())
        msg.set_NominalAltitude(self.rng.random())
        msg.set_NominalAltitudeType(AltitudeType.get_AltitudeType_str(AltitudeType.get_AltitudeType_int(self.rng.choice([0,1]))))
        v = msg.get_PayloadConfigurationList()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.options_PayloadConfiguration()[self.rng.randint(0, 6)]())
        v = msg.get_Info()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.build_KeyValuePair())
        # GroundVehicleConfiguration fields
        msg.set_RoadGraphID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_MinimumSpeed(self.rng.random())
        msg.set_MaximumSpeed(self.rng.random())
        msg.set_EnergyRate(self.rng.random())
        return msg

    def build_GroundVehicleState(self):
        msg = GroundVehicleState.GroundVehicleState()
        # EntityState fields
        msg.set_ID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_u(self.rng.random())
        msg.set_v(self.rng.random())
        msg.set_w(self.rng.random())
        msg.set_udot(self.rng.random())
        msg.set_vdot(self.rng.random())
        msg.set_wdot(self.rng.random())
        msg.set_Heading(self.rng.random())
        msg.set_Pitch(self.rng.random())
        msg.set_Roll(self.rng.random())
        msg.set_p(self.rng.random())
        msg.set_q(self.rng.random())
        msg.set_r(self.rng.random())
        msg.set_Course(self.rng.random())
        msg.set_Groundspeed(self.rng.random())
        msg.set_Location(self.options_Location3D()[self.rng.randint(0, 2)]())
        msg.set_EnergyAvailable(self.rng.random())
        msg.set_ActualEnergyRate(self.rng.random())
        v = msg.get_PayloadStateList()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.options_PayloadState()[self.rng.randint(0, 6)]())
        msg.set_CurrentWaypoint(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_CurrentCommand(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Mode(NavigationMode.get_NavigationMode_str(NavigationMode.get_NavigationMode_int(self.rng.choice([0,1,2,3,4,5]))))
        v = msg.get_AssociatedTasks()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Time(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_Info()
        for i in range(self.rng.randint(0, min(self.limit, 32))):
            v.append(self.build_KeyValuePair())
        # GroundVehicleState fields
        return msg

    def build_SurfaceVehicleConfiguration(self):
        msg = SurfaceVehicleConfiguration.SurfaceVehicleConfiguration()
        # EntityConfiguration fields
        msg.set_ID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Affiliation(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        msg.set_EntityType(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        msg.set_Label(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        msg.set_NominalSpeed(self.rng.random())
        msg.set_NominalAltitude(self.rng.random())
        msg.set_NominalAltitudeType(AltitudeType.get_AltitudeType_str(AltitudeType.get_AltitudeType_int(self.rng.choice([0,1]))))
        v = msg.get_PayloadConfigurationList()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.options_PayloadConfiguration()[self.rng.randint(0, 6)]())
        v = msg.get_Info()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.build_KeyValuePair())
        # SurfaceVehicleConfiguration fields
        msg.set_WaterArea(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_MinimumSpeed(self.rng.random())
        msg.set_MaximumSpeed(self.rng.random())
        msg.set_EnergyRate(self.rng.random())
        msg.set_MaxBankAngle(self.rng.random())
        msg.set_MaxBankRate(self.rng.random())
        return msg

    def build_SurfaceVehicleState(self):
        msg = SurfaceVehicleState.SurfaceVehicleState()
        # EntityState fields
        msg.set_ID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_u(self.rng.random())
        msg.set_v(self.rng.random())
        msg.set_w(self.rng.random())
        msg.set_udot(self.rng.random())
        msg.set_vdot(self.rng.random())
        msg.set_wdot(self.rng.random())
        msg.set_Heading(self.rng.random())
        msg.set_Pitch(self.rng.random())
        msg.set_Roll(self.rng.random())
        msg.set_p(self.rng.random())
        msg.set_q(self.rng.random())
        msg.set_r(self.rng.random())
        msg.set_Course(self.rng.random())
        msg.set_Groundspeed(self.rng.random())
        msg.set_Location(self.options_Location3D()[self.rng.randint(0, 2)]())
        msg.set_EnergyAvailable(self.rng.random())
        msg.set_ActualEnergyRate(self.rng.random())
        v = msg.get_PayloadStateList()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.options_PayloadState()[self.rng.randint(0, 6)]())
        msg.set_CurrentWaypoint(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_CurrentCommand(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Mode(NavigationMode.get_NavigationMode_str(NavigationMode.get_NavigationMode_int(self.rng.choice([0,1,2,3,4,5]))))
        v = msg.get_AssociatedTasks()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Time(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_Info()
        for i in range(self.rng.randint(0, min(self.limit, 32))):
            v.append(self.build_KeyValuePair())
        # SurfaceVehicleState fields
        msg.set_BankAngle(self.rng.random())
        msg.set_Speed(self.rng.random())
        return msg

    def build_StationarySensorConfiguration(self):
        msg = StationarySensorConfiguration.StationarySensorConfiguration()
        # EntityConfiguration fields
        msg.set_ID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Affiliation(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        msg.set_EntityType(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        msg.set_Label(''.join(self.rng.sample(string.ascii_letters, self.rng.randint(0, self.limit))))
        msg.set_NominalSpeed(self.rng.random())
        msg.set_NominalAltitude(self.rng.random())
        msg.set_NominalAltitudeType(AltitudeType.get_AltitudeType_str(AltitudeType.get_AltitudeType_int(self.rng.choice([0,1]))))
        v = msg.get_PayloadConfigurationList()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.options_PayloadConfiguration()[self.rng.randint(0, 6)]())
        v = msg.get_Info()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.build_KeyValuePair())
        # StationarySensorConfiguration fields
        return msg

    def build_StationarySensorState(self):
        msg = StationarySensorState.StationarySensorState()
        # EntityState fields
        msg.set_ID(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_u(self.rng.random())
        msg.set_v(self.rng.random())
        msg.set_w(self.rng.random())
        msg.set_udot(self.rng.random())
        msg.set_vdot(self.rng.random())
        msg.set_wdot(self.rng.random())
        msg.set_Heading(self.rng.random())
        msg.set_Pitch(self.rng.random())
        msg.set_Roll(self.rng.random())
        msg.set_p(self.rng.random())
        msg.set_q(self.rng.random())
        msg.set_r(self.rng.random())
        msg.set_Course(self.rng.random())
        msg.set_Groundspeed(self.rng.random())
        msg.set_Location(self.options_Location3D()[self.rng.randint(0, 2)]())
        msg.set_EnergyAvailable(self.rng.random())
        msg.set_ActualEnergyRate(self.rng.random())
        v = msg.get_PayloadStateList()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.options_PayloadState()[self.rng.randint(0, 6)]())
        msg.set_CurrentWaypoint(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_CurrentCommand(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Mode(NavigationMode.get_NavigationMode_str(NavigationMode.get_NavigationMode_int(self.rng.choice([0,1,2,3,4,5]))))
        v = msg.get_AssociatedTasks()
        for i in range(self.rng.randint(0, min(self.limit, 8))):
            v.append(self.rng.randint(-(2**63-1), 2**63-1))
        msg.set_Time(self.rng.randint(-(2**63-1), 2**63-1))
        v = msg.get_Info()
        for i in range(self.rng.randint(0, min(self.limit, 32))):
            v.append(self.build_KeyValuePair())
        # StationarySensorState fields
        return msg



    def write(self):
        print("seed={}".format(self.seed))

        # NOTE: roundtrip pack/unpack to ensure serialized text reflects underlying precision of types

        # afrl/cmasi messages
        path = self.dir.joinpath("afrl/cmasi")
        if not os.path.exists(str(path)):
            os.makedirs(str(path), exist_ok=True)

        print("{:>03}/{:>3}: writing {}...".format(1, 164, "AbstractGeometry"))
        msg = self.build_AbstractGeometry()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("AbstractGeometry")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(2, 164, "KeyValuePair"))
        msg = self.build_KeyValuePair()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("KeyValuePair")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(3, 164, "Location3D"))
        msg = self.build_Location3D()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("Location3D")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(4, 164, "PayloadAction"))
        msg = self.build_PayloadAction()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("PayloadAction")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(5, 164, "PayloadConfiguration"))
        msg = self.build_PayloadConfiguration()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("PayloadConfiguration")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(6, 164, "PayloadState"))
        msg = self.build_PayloadState()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("PayloadState")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(7, 164, "VehicleAction"))
        msg = self.build_VehicleAction()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("VehicleAction")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(8, 164, "Task"))
        msg = self.build_Task()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("Task")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(9, 164, "SearchTask"))
        msg = self.build_SearchTask()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("SearchTask")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(10, 164, "AbstractZone"))
        msg = self.build_AbstractZone()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("AbstractZone")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(11, 164, "EntityConfiguration"))
        msg = self.build_EntityConfiguration()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("EntityConfiguration")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(12, 164, "FlightProfile"))
        msg = self.build_FlightProfile()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("FlightProfile")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(13, 164, "AirVehicleConfiguration"))
        msg = self.build_AirVehicleConfiguration()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("AirVehicleConfiguration")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(14, 164, "EntityState"))
        msg = self.build_EntityState()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("EntityState")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(15, 164, "AirVehicleState"))
        msg = self.build_AirVehicleState()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("AirVehicleState")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(16, 164, "Wedge"))
        msg = self.build_Wedge()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("Wedge")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(17, 164, "AreaSearchTask"))
        msg = self.build_AreaSearchTask()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("AreaSearchTask")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(18, 164, "CameraAction"))
        msg = self.build_CameraAction()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("CameraAction")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(19, 164, "CameraConfiguration"))
        msg = self.build_CameraConfiguration()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("CameraConfiguration")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(20, 164, "GimballedPayloadState"))
        msg = self.build_GimballedPayloadState()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("GimballedPayloadState")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(21, 164, "CameraState"))
        msg = self.build_CameraState()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("CameraState")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(22, 164, "Circle"))
        msg = self.build_Circle()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("Circle")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(23, 164, "GimbalAngleAction"))
        msg = self.build_GimbalAngleAction()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("GimbalAngleAction")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(24, 164, "GimbalConfiguration"))
        msg = self.build_GimbalConfiguration()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("GimbalConfiguration")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(25, 164, "GimbalScanAction"))
        msg = self.build_GimbalScanAction()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("GimbalScanAction")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(26, 164, "GimbalStareAction"))
        msg = self.build_GimbalStareAction()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("GimbalStareAction")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(27, 164, "GimbalState"))
        msg = self.build_GimbalState()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("GimbalState")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(28, 164, "GoToWaypointAction"))
        msg = self.build_GoToWaypointAction()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("GoToWaypointAction")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(29, 164, "KeepInZone"))
        msg = self.build_KeepInZone()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("KeepInZone")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(30, 164, "KeepOutZone"))
        msg = self.build_KeepOutZone()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("KeepOutZone")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(31, 164, "LineSearchTask"))
        msg = self.build_LineSearchTask()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("LineSearchTask")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(32, 164, "NavigationAction"))
        msg = self.build_NavigationAction()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("NavigationAction")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(33, 164, "LoiterAction"))
        msg = self.build_LoiterAction()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("LoiterAction")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(34, 164, "LoiterTask"))
        msg = self.build_LoiterTask()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("LoiterTask")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(35, 164, "Waypoint"))
        msg = self.build_Waypoint()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("Waypoint")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(36, 164, "MissionCommand"))
        msg = self.build_MissionCommand()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("MissionCommand")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(37, 164, "MustFlyTask"))
        msg = self.build_MustFlyTask()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("MustFlyTask")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(38, 164, "OperatorSignal"))
        msg = self.build_OperatorSignal()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("OperatorSignal")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(39, 164, "OperatingRegion"))
        msg = self.build_OperatingRegion()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("OperatingRegion")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(40, 164, "AutomationRequest"))
        msg = self.build_AutomationRequest()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("AutomationRequest")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(41, 164, "PointSearchTask"))
        msg = self.build_PointSearchTask()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("PointSearchTask")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(42, 164, "Polygon"))
        msg = self.build_Polygon()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("Polygon")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(43, 164, "Rectangle"))
        msg = self.build_Rectangle()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("Rectangle")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(44, 164, "RemoveTasks"))
        msg = self.build_RemoveTasks()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("RemoveTasks")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(45, 164, "ServiceStatus"))
        msg = self.build_ServiceStatus()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("ServiceStatus")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(46, 164, "SessionStatus"))
        msg = self.build_SessionStatus()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("SessionStatus")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(47, 164, "VehicleActionCommand"))
        msg = self.build_VehicleActionCommand()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("VehicleActionCommand")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(48, 164, "VideoStreamAction"))
        msg = self.build_VideoStreamAction()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("VideoStreamAction")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(49, 164, "VideoStreamConfiguration"))
        msg = self.build_VideoStreamConfiguration()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("VideoStreamConfiguration")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(50, 164, "VideoStreamState"))
        msg = self.build_VideoStreamState()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("VideoStreamState")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(51, 164, "AutomationResponse"))
        msg = self.build_AutomationResponse()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("AutomationResponse")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(52, 164, "RemoveZones"))
        msg = self.build_RemoveZones()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("RemoveZones")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(53, 164, "RemoveEntities"))
        msg = self.build_RemoveEntities()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("RemoveEntities")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(54, 164, "FlightDirectorAction"))
        msg = self.build_FlightDirectorAction()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("FlightDirectorAction")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(55, 164, "WeatherReport"))
        msg = self.build_WeatherReport()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("WeatherReport")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(56, 164, "FollowPathCommand"))
        msg = self.build_FollowPathCommand()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("FollowPathCommand")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(57, 164, "PathWaypoint"))
        msg = self.build_PathWaypoint()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("PathWaypoint")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(58, 164, "StopMovementAction"))
        msg = self.build_StopMovementAction()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("StopMovementAction")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(59, 164, "WaypointTransfer"))
        msg = self.build_WaypointTransfer()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("WaypointTransfer")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(60, 164, "PayloadStowAction"))
        msg = self.build_PayloadStowAction()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("PayloadStowAction")), "wb") as f:
            f.write(bytes)

        # afrl/impact messages
        path = self.dir.joinpath("afrl/impact")
        if not os.path.exists(str(path)):
            os.makedirs(str(path), exist_ok=True)

        print("{:>03}/{:>3}: writing {}...".format(61, 164, "PowerConfiguration"))
        msg = self.build_PowerConfiguration()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("PowerConfiguration")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(62, 164, "RadioConfiguration"))
        msg = self.build_RadioConfiguration()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("RadioConfiguration")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(63, 164, "RadioTowerConfiguration"))
        msg = self.build_RadioTowerConfiguration()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("RadioTowerConfiguration")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(64, 164, "RadioState"))
        msg = self.build_RadioState()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("RadioState")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(65, 164, "RadioTowerState"))
        msg = self.build_RadioTowerState()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("RadioTowerState")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(66, 164, "ImpactPayloadConfiguration"))
        msg = self.build_ImpactPayloadConfiguration()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("ImpactPayloadConfiguration")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(67, 164, "DeployImpactPayload"))
        msg = self.build_DeployImpactPayload()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("DeployImpactPayload")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(68, 164, "PowerPlantState"))
        msg = self.build_PowerPlantState()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("PowerPlantState")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(69, 164, "BatchRoutePlanRequest"))
        msg = self.build_BatchRoutePlanRequest()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("BatchRoutePlanRequest")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(70, 164, "BatchRoutePlanResponse"))
        msg = self.build_BatchRoutePlanResponse()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("BatchRoutePlanResponse")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(71, 164, "TaskTimingPair"))
        msg = self.build_TaskTimingPair()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("TaskTimingPair")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(72, 164, "BatchSummaryRequest"))
        msg = self.build_BatchSummaryRequest()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("BatchSummaryRequest")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(73, 164, "BatchSummaryResponse"))
        msg = self.build_BatchSummaryResponse()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("BatchSummaryResponse")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(74, 164, "TaskSummary"))
        msg = self.build_TaskSummary()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("TaskSummary")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(75, 164, "VehicleSummary"))
        msg = self.build_VehicleSummary()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("VehicleSummary")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(76, 164, "SpeedAltPair"))
        msg = self.build_SpeedAltPair()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("SpeedAltPair")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(77, 164, "ImpactAutomationRequest"))
        msg = self.build_ImpactAutomationRequest()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("ImpactAutomationRequest")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(78, 164, "ImpactAutomationResponse"))
        msg = self.build_ImpactAutomationResponse()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("ImpactAutomationResponse")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(79, 164, "PointOfInterest"))
        msg = self.build_PointOfInterest()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("PointOfInterest")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(80, 164, "LineOfInterest"))
        msg = self.build_LineOfInterest()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("LineOfInterest")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(81, 164, "AreaOfInterest"))
        msg = self.build_AreaOfInterest()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("AreaOfInterest")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(82, 164, "ImpactPointSearchTask"))
        msg = self.build_ImpactPointSearchTask()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("ImpactPointSearchTask")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(83, 164, "PatternSearchTask"))
        msg = self.build_PatternSearchTask()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("PatternSearchTask")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(84, 164, "AngledAreaSearchTask"))
        msg = self.build_AngledAreaSearchTask()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("AngledAreaSearchTask")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(85, 164, "ImpactLineSearchTask"))
        msg = self.build_ImpactLineSearchTask()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("ImpactLineSearchTask")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(86, 164, "WatchTask"))
        msg = self.build_WatchTask()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("WatchTask")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(87, 164, "MultiVehicleWatchTask"))
        msg = self.build_MultiVehicleWatchTask()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("MultiVehicleWatchTask")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(88, 164, "CommRelayTask"))
        msg = self.build_CommRelayTask()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("CommRelayTask")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(89, 164, "CordonTask"))
        msg = self.build_CordonTask()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("CordonTask")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(90, 164, "BlockadeTask"))
        msg = self.build_BlockadeTask()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("BlockadeTask")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(91, 164, "EscortTask"))
        msg = self.build_EscortTask()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("EscortTask")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(92, 164, "ConfigurationRequest"))
        msg = self.build_ConfigurationRequest()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("ConfigurationRequest")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(93, 164, "WaterReport"))
        msg = self.build_WaterReport()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("WaterReport")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(94, 164, "WaterZone"))
        msg = self.build_WaterZone()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("WaterZone")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(95, 164, "PayloadDropTask"))
        msg = self.build_PayloadDropTask()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("PayloadDropTask")), "wb") as f:
            f.write(bytes)

        # afrl/cmasi/perceive messages
        path = self.dir.joinpath("afrl/cmasi/perceive")
        if not os.path.exists(str(path)):
            os.makedirs(str(path), exist_ok=True)

        print("{:>03}/{:>3}: writing {}...".format(96, 164, "EntityPerception"))
        msg = self.build_EntityPerception()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("EntityPerception")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(97, 164, "TrackEntityAction"))
        msg = self.build_TrackEntityAction()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("TrackEntityAction")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(98, 164, "TrackEntityTask"))
        msg = self.build_TrackEntityTask()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("TrackEntityTask")), "wb") as f:
            f.write(bytes)

        # uxas/messages/route messages
        path = self.dir.joinpath("uxas/messages/route")
        if not os.path.exists(str(path)):
            os.makedirs(str(path), exist_ok=True)

        print("{:>03}/{:>3}: writing {}...".format(99, 164, "GraphNode"))
        msg = self.build_GraphNode()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("GraphNode")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(100, 164, "GraphEdge"))
        msg = self.build_GraphEdge()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("GraphEdge")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(101, 164, "GraphRegion"))
        msg = self.build_GraphRegion()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("GraphRegion")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(102, 164, "RouteConstraints"))
        msg = self.build_RouteConstraints()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("RouteConstraints")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(103, 164, "RouteRequest"))
        msg = self.build_RouteRequest()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("RouteRequest")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(104, 164, "RoutePlanRequest"))
        msg = self.build_RoutePlanRequest()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("RoutePlanRequest")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(105, 164, "RoutePlan"))
        msg = self.build_RoutePlan()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("RoutePlan")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(106, 164, "RoutePlanResponse"))
        msg = self.build_RoutePlanResponse()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("RoutePlanResponse")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(107, 164, "RouteResponse"))
        msg = self.build_RouteResponse()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("RouteResponse")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(108, 164, "EgressRouteRequest"))
        msg = self.build_EgressRouteRequest()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("EgressRouteRequest")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(109, 164, "EgressRouteResponse"))
        msg = self.build_EgressRouteResponse()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("EgressRouteResponse")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(110, 164, "RoadPointsConstraints"))
        msg = self.build_RoadPointsConstraints()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("RoadPointsConstraints")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(111, 164, "RoadPointsRequest"))
        msg = self.build_RoadPointsRequest()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("RoadPointsRequest")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(112, 164, "RoadPointsResponse"))
        msg = self.build_RoadPointsResponse()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("RoadPointsResponse")), "wb") as f:
            f.write(bytes)

        # uxas/messages/uxnative messages
        path = self.dir.joinpath("uxas/messages/uxnative")
        if not os.path.exists(str(path)):
            os.makedirs(str(path), exist_ok=True)

        print("{:>03}/{:>3}: writing {}...".format(113, 164, "VideoRecord"))
        msg = self.build_VideoRecord()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("VideoRecord")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(114, 164, "StartupComplete"))
        msg = self.build_StartupComplete()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("StartupComplete")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(115, 164, "CreateNewService"))
        msg = self.build_CreateNewService()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("CreateNewService")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(116, 164, "KillService"))
        msg = self.build_KillService()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("KillService")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(117, 164, "IncrementWaypoint"))
        msg = self.build_IncrementWaypoint()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("IncrementWaypoint")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(118, 164, "SafeHeadingAction"))
        msg = self.build_SafeHeadingAction()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("SafeHeadingAction")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(119, 164, "EntityLocation"))
        msg = self.build_EntityLocation()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("EntityLocation")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(120, 164, "BandwidthTest"))
        msg = self.build_BandwidthTest()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("BandwidthTest")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(121, 164, "BandwidthReceiveReport"))
        msg = self.build_BandwidthReceiveReport()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("BandwidthReceiveReport")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(122, 164, "SubTaskExecution"))
        msg = self.build_SubTaskExecution()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("SubTaskExecution")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(123, 164, "SubTaskAssignment"))
        msg = self.build_SubTaskAssignment()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("SubTaskAssignment")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(124, 164, "AutopilotKeepAlive"))
        msg = self.build_AutopilotKeepAlive()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("AutopilotKeepAlive")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(125, 164, "OnboardStatusReport"))
        msg = self.build_OnboardStatusReport()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("OnboardStatusReport")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(126, 164, "EntityJoin"))
        msg = self.build_EntityJoin()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("EntityJoin")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(127, 164, "EntityExit"))
        msg = self.build_EntityExit()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("EntityExit")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(128, 164, "SimulationTimeStepAcknowledgement"))
        msg = self.build_SimulationTimeStepAcknowledgement()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("SimulationTimeStepAcknowledgement")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(129, 164, "SpeedOverrideAction"))
        msg = self.build_SpeedOverrideAction()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("SpeedOverrideAction")), "wb") as f:
            f.write(bytes)

        # uxas/messages/task messages
        path = self.dir.joinpath("uxas/messages/task")
        if not os.path.exists(str(path)):
            os.makedirs(str(path), exist_ok=True)

        print("{:>03}/{:>3}: writing {}...".format(130, 164, "AssignmentCoordinatorTask"))
        msg = self.build_AssignmentCoordinatorTask()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("AssignmentCoordinatorTask")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(131, 164, "RendezvousTask"))
        msg = self.build_RendezvousTask()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("RendezvousTask")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(132, 164, "PlanningState"))
        msg = self.build_PlanningState()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("PlanningState")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(133, 164, "AssignmentCoordination"))
        msg = self.build_AssignmentCoordination()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("AssignmentCoordination")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(134, 164, "CoordinatedAutomationRequest"))
        msg = self.build_CoordinatedAutomationRequest()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("CoordinatedAutomationRequest")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(135, 164, "TaskAutomationRequest"))
        msg = self.build_TaskAutomationRequest()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("TaskAutomationRequest")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(136, 164, "TaskAutomationResponse"))
        msg = self.build_TaskAutomationResponse()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("TaskAutomationResponse")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(137, 164, "UniqueAutomationRequest"))
        msg = self.build_UniqueAutomationRequest()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("UniqueAutomationRequest")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(138, 164, "UniqueAutomationResponse"))
        msg = self.build_UniqueAutomationResponse()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("UniqueAutomationResponse")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(139, 164, "SensorFootprintRequests"))
        msg = self.build_SensorFootprintRequests()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("SensorFootprintRequests")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(140, 164, "FootprintRequest"))
        msg = self.build_FootprintRequest()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("FootprintRequest")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(141, 164, "SensorFootprint"))
        msg = self.build_SensorFootprint()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("SensorFootprint")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(142, 164, "SensorFootprintResponse"))
        msg = self.build_SensorFootprintResponse()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("SensorFootprintResponse")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(143, 164, "TaskImplementationRequest"))
        msg = self.build_TaskImplementationRequest()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("TaskImplementationRequest")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(144, 164, "TaskImplementationResponse"))
        msg = self.build_TaskImplementationResponse()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("TaskImplementationResponse")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(145, 164, "AssignmentCostMatrix"))
        msg = self.build_AssignmentCostMatrix()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("AssignmentCostMatrix")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(146, 164, "TaskOptionCost"))
        msg = self.build_TaskOptionCost()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("TaskOptionCost")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(147, 164, "TaskAssignment"))
        msg = self.build_TaskAssignment()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("TaskAssignment")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(148, 164, "TaskAssignmentSummary"))
        msg = self.build_TaskAssignmentSummary()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("TaskAssignmentSummary")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(149, 164, "TaskOption"))
        msg = self.build_TaskOption()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("TaskOption")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(150, 164, "TaskPlanOptions"))
        msg = self.build_TaskPlanOptions()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("TaskPlanOptions")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(151, 164, "TaskPause"))
        msg = self.build_TaskPause()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("TaskPause")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(152, 164, "TaskResume"))
        msg = self.build_TaskResume()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("TaskResume")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(153, 164, "TaskProgress"))
        msg = self.build_TaskProgress()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("TaskProgress")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(154, 164, "TaskProgressRequest"))
        msg = self.build_TaskProgressRequest()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("TaskProgressRequest")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(155, 164, "TaskInitialized"))
        msg = self.build_TaskInitialized()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("TaskInitialized")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(156, 164, "TaskActive"))
        msg = self.build_TaskActive()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("TaskActive")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(157, 164, "TaskComplete"))
        msg = self.build_TaskComplete()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("TaskComplete")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(158, 164, "CancelTask"))
        msg = self.build_CancelTask()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("CancelTask")), "wb") as f:
            f.write(bytes)

        # afrl/vehicles messages
        path = self.dir.joinpath("afrl/vehicles")
        if not os.path.exists(str(path)):
            os.makedirs(str(path), exist_ok=True)

        print("{:>03}/{:>3}: writing {}...".format(159, 164, "GroundVehicleConfiguration"))
        msg = self.build_GroundVehicleConfiguration()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("GroundVehicleConfiguration")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(160, 164, "GroundVehicleState"))
        msg = self.build_GroundVehicleState()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("GroundVehicleState")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(161, 164, "SurfaceVehicleConfiguration"))
        msg = self.build_SurfaceVehicleConfiguration()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("SurfaceVehicleConfiguration")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(162, 164, "SurfaceVehicleState"))
        msg = self.build_SurfaceVehicleState()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("SurfaceVehicleState")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(163, 164, "StationarySensorConfiguration"))
        msg = self.build_StationarySensorConfiguration()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("StationarySensorConfiguration")), "wb") as f:
            f.write(bytes)

        print("{:>03}/{:>3}: writing {}...".format(164, 164, "StationarySensorState"))
        msg = self.build_StationarySensorState()
        msg.unpack(msg.pack(), 0)
        bytes = LMCPFactory.packMessage(msg, self.checksum)
        with open(str(path.joinpath("StationarySensorState")), "wb") as f:
            f.write(bytes)



def valid_directory(string):
    dir_ = Path(string)
    if dir_.is_dir():
        return dir_
    else:
        raise argparse.ArgumentTypeError("\'{}\' does not exist".format(string))

def to_bool(string):
    if string.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif string.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')

def main():
    parser = argparse.ArgumentParser(description="Generates reference messages (xml and corresponding binary) for use by implementations to verify correctness of serialization/deserialization functions")
    parser.add_argument("-dir", help="path to root output directory (default=cwd)", type=valid_directory, default=os.getcwd())
    parser.add_argument("-seed", help="random seed to use for number generator (default=random)", type=int, default=int.from_bytes(os.urandom(8), byteorder="big"))
    parser.add_argument("-limit", help="limit variable length arrays, including strings, to specified length (default=32)", type=int, default=32)
    parser.add_argument("-checksum", help="compute checksum (default", type=to_bool, default=False)
    args = parser.parse_args()

    generator = Generator(args.dir.joinpath("reference"), args.seed, args.limit, args.checksum)
    generator.write()

    return 0


if __name__ == "__main__":
    sys.exit(main())
