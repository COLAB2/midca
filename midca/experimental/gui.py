import Tkinter as tk
import tkFont
import sys

class StdoutDirector:
    
    def __init__(self,textArea):
        self.textArea = textArea
        
    def write(self,str):
        self.textArea.insert(tk.END,str)
        self.textArea.see(tk.END)
    
    def flush(self):
        pass

class MidcaDisplay(tk.Frame):
	
	def __init__(self, master, phaseManager):
		tk.Frame.__init__(self, master)
		self.phaseManager = phaseManager
		self.textArea = tk.Text(self, height=30, width=80,bg='light cyan')
		self.textScroll = tk.Scrollbar(self)
		self.textArea.config(yscrollcommand = self.textScroll.set)
		self.textScroll.config(command = self.textArea.yview)
		self.textArea.grid(row = 2, column = 0, columnspan = 4)
		self.textScroll.grid(row = 2, column = 4)
		
		self.nextPhaseButton = tk.Button(self, text = "Next Phase", command = self.phaseManager.next_phase)
		self.nextPhaseButton.grid(row = 0, column = 0)
		
		#buffer = tk.Frame(self, padx = 5)
		#buffer.grid(row = 0, column = 1)
		
		self.numSkipEntry = tk.Entry(self, width = 10)
		self.numSkipEntry.insert(tk.INSERT, '1')
		self.skipButton = tk.Button(self, text = "Skip Cycles", command = self.skipCycles)
		self.skipButton.grid(row = 0, column = 1)
		self.numSkipEntry.grid(row = 1, column = 1)
		self.errorLabel = tk.Label(self, font = tkFont.Font(family = "helvetica", size = 9), width = 30)
		self.errorLabel.grid(row = 1, column = 2)
		
		sys.stdout = StdoutDirector(self.textArea)
		self.phaseManager.init()
	
	#modify this method to send output to a file.
	def skipCycles(self):
		try:
			num = int(self.numSkipEntry.get())
			if num <= 0:
				raise ValueError()
		except ValueError:
			self.errorLabel.config(text = "num cycles must be an integer > 0")
			return
		self.phaseManager.several_cycles(num, verbose = 0)

import midca
from midca.examples import predicateworld
import inspect, os
import threading

thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))

MIDCA_ROOT = thisDir + "/../"

myMidca = predicateworld.guiMidca(domainFile = MIDCA_ROOT + "worldsim/domains/arsonist.sim", stateFile = MIDCA_ROOT + "worldsim/states/defstate_fire.sim")

root = tk.Tk()
display = MidcaDisplay(root, myMidca)
display.pack()

root.mainloop()

