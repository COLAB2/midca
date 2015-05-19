#!/usr/bin/env python 
from MIDCA import base, rosrun
from MIDCA.modules import simulator, perceive, note, guide, evaluate, intend, planning, act
from MIDCA.modules._plan.asynch import asynch, operators, methods
from MIDCA.logging import Logger
import inspect, os, rospy

def RosMidca():
	myMidca = base.MIDCA(None, verbose = 2)
	for phase in ["Perceive", "Interpret", "Eval", "Intend", "Plan", "Act"]:
		myMidca.append_phase(phase)

	myMidca.append_module("Perceive", perceive.ROSObserver())
	myMidca.append_module("Interpret", guide.InstructionReceiver())
	#myMidca.append_module("Eval", evaluate.SimpleEval())
	myMidca.append_module("Intend", intend.SimpleIntend())
	myMidca.append_module("Plan", planning.AsynchPyhopPlanner(methods.declare_methods, 
	operators.declare_operators))
	myMidca.append_module("Act", act.AsynchronousAct())
	return myMidca
	
thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))

MIDCA_ROOT = thisDir + "/../"

myMidca = RosMidca()

myMidca.logger.logOutput()
myMidca.mem.enableLogging(myMidca.logger)

rosMidca = RosMidca(incomingMsgHandlers = [
	FixedObjectLocationHandler("quad_position", "quad", myMidca),
	UtteranceHandler("cmds_received", myMidca),
	FeedbackHandler("feedback", myMidca)],
	outgoingMsgHandlers = [OutgoingMsgHandler(asynch.POINT_TOPIC, rospy.String)])
rosMidca.ros_connect()
rosMidca.run_midca()