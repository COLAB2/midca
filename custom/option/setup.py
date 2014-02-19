import sys
from custom import customsetup, memconstants
import phasemanager
import socket, subprocess, time

def initialize(options):	
		
	midca = phasemanager.MIDCA(customsetup.read_world(), customsetup.get_world_sim(options), options, memconstants)
		
	modules = customsetup.get_modules(options)
	
	for phase in customsetup.PHASES:
		if phase in modules:
			midca.add_module(phase, modules[phase])
	midca.init()
	customsetup.custom_init(midca)
	return midca