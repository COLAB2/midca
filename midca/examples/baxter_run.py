#!/usr/bin/env python 
from midca import base, rosrun
from midca.modules import simulator, perceive, note, guide, evaluate, intend, planning, act
from midca.modules._plan.asynch import asynch, operators, methods
from midca.logging import Logger
import inspect, os
from std_msgs.msg import String

OBJ_LOC_TOPIC = "quad_pos"
UTTERANCE_TOPIC = "cmds_received"

def ros_style_midca():
	myMidca = base.MIDCA(None, verbose = 2)
	for phase in ["Perceive", "Interpret", "Eval", "Intend", "Plan", "Act"]:
		myMidca.append_phase(phase)

	myMidca.append_module("Perceive", perceive.ROSObserver())
	myMidca.append_module("Interpret", guide.InstructionReceiver())
	myMidca.append_module("Eval", evaluate.EvalPointingFromFeedback())
	myMidca.append_module("Intend", intend.SimpleIntend())
	myMidca.append_module("Plan", planning.AsynchPyhopPlanner(methods.declare_methods, 
	operators.declare_ops))
	myMidca.append_module("Act", act.AsynchronousAct())
	return myMidca
	
thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))

MIDCA_ROOT = thisDir + "/../"

myMidca = ros_style_midca()

myMidca.logger.logOutput()
myMidca.mem.enableLogging(myMidca.logger)

rosMidca = rosrun.RosMidca(myMidca, incomingMsgHandlers = [
	rosrun.FixedObjectLocationHandler(OBJ_LOC_TOPIC, "quad", myMidca),
	rosrun.UtteranceHandler(UTTERANCE_TOPIC, myMidca),
	rosrun.FeedbackHandler(rosrun.FEEDBACK_TOPIC, myMidca)],
	outgoingMsgHandlers = [rosrun.OutgoingMsgHandler(asynch.POINT_TOPIC, String)])
rosMidca.ros_connect()
rosMidca.run_midca()
