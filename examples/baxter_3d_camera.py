#!/usr/bin/env python 
from MIDCA import base, rosrun
from MIDCA.modules import simulator, perceive, note, guide, evaluate, intend, planning, act
from MIDCA.modules._plan.asynch import asynch, operators_sr, methods_sr, monitors_sr
from MIDCA.logging import Logger
import inspect, os
import rospy
from std_msgs.msg import String
from MIDCA.examples import Calibrate
from geometry_msgs.msg import Point, PointStamped
from baxter import *


def ros_style_midca():
	myMidca = base.MIDCA(None, verbose = 2 , logenabled = False)
	for phase in ["Perceive", "Interpret", "Eval", "Intend", "Plan", "Act"]:
		myMidca.append_phase(phase)

	myMidca.append_module("Perceive", perceive.ROSObserver())
	myMidca.append_module("Interpret", guide.InstructionReceiver_sr())
	myMidca.append_module("Eval", evaluate.EvalPointingFromFeedback())
	myMidca.append_module("Intend", intend.SimpleIntend())
	myMidca.append_module("Plan", planning.AsynchPyhopPlanner_3d_camera(methods_sr.declare_methods, 
	operators_sr.declare_ops,monitors_sr.declare_monitors
	
	))
	myMidca.append_module("Act", act.AsynchronousAct())
	return myMidca
	
thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))

MIDCA_ROOT = thisDir + "/../"

myMidca = ros_style_midca()






rosMidca = rosrun.RosMidca(myMidca, incomingMsgHandlers = [
	#rosrun.CalibrationHandler("calibrate_done", myMidca),
	rosrun.MultipleObjectsLocationHandler("obj_pos", myMidca),
	rosrun.UtteranceHandler("cmds_received", myMidca),
	rosrun.FeedbackHandler(rosrun.FEEDBACK_TOPIC, myMidca)],
	outgoingMsgHandlers = [rosrun.OutgoingMsgHandler(asynch.LOC_TOPIC, String), 
						rosrun.OutgoingMsgHandler(asynch.GRAB_TOPIC, String),
						rosrun.OutgoingMsgHandler(asynch.RELEASE_TOPIC, String),
						rosrun.OutgoingMsgHandler(asynch.RAISE_TOPIC, String)])

p = Point(x = 0.6480168766398825, y =  0.4782503847940384, z = 0.289534050209461)
myMidca.mem.set(myMidca.mem.RAISING_POINT, p)
q = Point(0.7445881227726235, 0.13967133208903987, -0.15821251824917773)
myMidca.mem.set(myMidca.mem.PUTTING_POINT, q)
rosMidca.ros_connect()

# calibrate baxter's left gripper
baxter = Baxter()
baxter.enable()
baxter.calibrateLeftGripper()



raw_input('Enter ...')
rosMidca.run_midca()

