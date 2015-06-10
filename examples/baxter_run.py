#!/usr/bin/env python 
from MIDCA import base, rosrun
from MIDCA.modules import simulator, perceive, note, guide, evaluate, intend, planning, act
from MIDCA.modules._plan.asynch import asynch, operators, methods
from MIDCA.logging import Logger
import inspect, os
from std_msgs.msg import String

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
	rosrun.FixedObjectLocationHandler("quad_pos", "quad", myMidca),
	rosrun.UtteranceHandler("cmds_received", myMidca),
	rosrun.FeedbackHandler(rosrun.FEEDBACK_TOPIC, myMidca)],
	outgoingMsgHandlers = [rosrun.OutgoingMsgHandler(asynch.POINT_TOPIC, String)])
rosMidca.ros_connect()
rosMidca.run_midca()
