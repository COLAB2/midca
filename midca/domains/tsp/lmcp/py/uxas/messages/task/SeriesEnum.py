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


SERIES_NAME = "UXTASK"
#Series Name turned into a long for quick comparisons.
SERIES_NAME_ID = 6149757930721443840
SERIES_VERSION = 8


class SeriesEnum:

    def getName(self, type_):
        if(type_ ==  1): return "AssignmentCoordinatorTask"
        if(type_ ==  2): return "RendezvousTask"
        if(type_ ==  3): return "PlanningState"
        if(type_ ==  4): return "AssignmentCoordination"
        if(type_ ==  5): return "CoordinatedAutomationRequest"
        if(type_ ==  6): return "TaskAutomationRequest"
        if(type_ ==  7): return "TaskAutomationResponse"
        if(type_ ==  8): return "UniqueAutomationRequest"
        if(type_ ==  9): return "UniqueAutomationResponse"
        if(type_ ==  10): return "SensorFootprintRequests"
        if(type_ ==  11): return "FootprintRequest"
        if(type_ ==  12): return "SensorFootprint"
        if(type_ ==  13): return "SensorFootprintResponse"
        if(type_ ==  14): return "TaskImplementationRequest"
        if(type_ ==  15): return "TaskImplementationResponse"
        if(type_ ==  16): return "AssignmentCostMatrix"
        if(type_ ==  17): return "TaskOptionCost"
        if(type_ ==  18): return "TaskAssignment"
        if(type_ ==  19): return "TaskAssignmentSummary"
        if(type_ ==  20): return "TaskOption"
        if(type_ ==  21): return "TaskPlanOptions"
        if(type_ ==  22): return "TaskPause"
        if(type_ ==  23): return "TaskResume"
        if(type_ ==  24): return "TaskProgress"
        if(type_ ==  25): return "TaskProgressRequest"
        if(type_ ==  26): return "TaskInitialized"
        if(type_ ==  27): return "TaskActive"
        if(type_ ==  28): return "TaskComplete"
        if(type_ ==  29): return "CancelTask"


    def getType(self, name):
        if ( name == "AssignmentCoordinatorTask"): return 1
        if ( name == "RendezvousTask"): return 2
        if ( name == "PlanningState"): return 3
        if ( name == "AssignmentCoordination"): return 4
        if ( name == "CoordinatedAutomationRequest"): return 5
        if ( name == "TaskAutomationRequest"): return 6
        if ( name == "TaskAutomationResponse"): return 7
        if ( name == "UniqueAutomationRequest"): return 8
        if ( name == "UniqueAutomationResponse"): return 9
        if ( name == "SensorFootprintRequests"): return 10
        if ( name == "FootprintRequest"): return 11
        if ( name == "SensorFootprint"): return 12
        if ( name == "SensorFootprintResponse"): return 13
        if ( name == "TaskImplementationRequest"): return 14
        if ( name == "TaskImplementationResponse"): return 15
        if ( name == "AssignmentCostMatrix"): return 16
        if ( name == "TaskOptionCost"): return 17
        if ( name == "TaskAssignment"): return 18
        if ( name == "TaskAssignmentSummary"): return 19
        if ( name == "TaskOption"): return 20
        if ( name == "TaskPlanOptions"): return 21
        if ( name == "TaskPause"): return 22
        if ( name == "TaskResume"): return 23
        if ( name == "TaskProgress"): return 24
        if ( name == "TaskProgressRequest"): return 25
        if ( name == "TaskInitialized"): return 26
        if ( name == "TaskActive"): return 27
        if ( name == "TaskComplete"): return 28
        if ( name == "CancelTask"): return 29

        return -1

    def getInstance(self, type_):
        if(type_ ==  1): return AssignmentCoordinatorTask.AssignmentCoordinatorTask()
        if(type_ ==  2): return RendezvousTask.RendezvousTask()
        if(type_ ==  3): return PlanningState.PlanningState()
        if(type_ ==  4): return AssignmentCoordination.AssignmentCoordination()
        if(type_ ==  5): return CoordinatedAutomationRequest.CoordinatedAutomationRequest()
        if(type_ ==  6): return TaskAutomationRequest.TaskAutomationRequest()
        if(type_ ==  7): return TaskAutomationResponse.TaskAutomationResponse()
        if(type_ ==  8): return UniqueAutomationRequest.UniqueAutomationRequest()
        if(type_ ==  9): return UniqueAutomationResponse.UniqueAutomationResponse()
        if(type_ ==  10): return SensorFootprintRequests.SensorFootprintRequests()
        if(type_ ==  11): return FootprintRequest.FootprintRequest()
        if(type_ ==  12): return SensorFootprint.SensorFootprint()
        if(type_ ==  13): return SensorFootprintResponse.SensorFootprintResponse()
        if(type_ ==  14): return TaskImplementationRequest.TaskImplementationRequest()
        if(type_ ==  15): return TaskImplementationResponse.TaskImplementationResponse()
        if(type_ ==  16): return AssignmentCostMatrix.AssignmentCostMatrix()
        if(type_ ==  17): return TaskOptionCost.TaskOptionCost()
        if(type_ ==  18): return TaskAssignment.TaskAssignment()
        if(type_ ==  19): return TaskAssignmentSummary.TaskAssignmentSummary()
        if(type_ ==  20): return TaskOption.TaskOption()
        if(type_ ==  21): return TaskPlanOptions.TaskPlanOptions()
        if(type_ ==  22): return TaskPause.TaskPause()
        if(type_ ==  23): return TaskResume.TaskResume()
        if(type_ ==  24): return TaskProgress.TaskProgress()
        if(type_ ==  25): return TaskProgressRequest.TaskProgressRequest()
        if(type_ ==  26): return TaskInitialized.TaskInitialized()
        if(type_ ==  27): return TaskActive.TaskActive()
        if(type_ ==  28): return TaskComplete.TaskComplete()
        if(type_ ==  29): return CancelTask.CancelTask()

        return None
