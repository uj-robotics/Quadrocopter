
import visual.graph
from visual import *
import time
import thread
from numpy import *


x_axis = [1,0,0]
y_axis = [0,1,0]
z_axis = [0,0,1]


# Konwencja XYZ #
eulers = [0,0,0]
current_eulers = [0,0,0]

# How often refresh 
DRAW_MS = 100
# How many signals plot back
PLOT_BACK = 100 
# Sampling frequency
SIGNAL_MS = 100

class CyclicBuffer(object):
	"""
		Simple class representing cyclic buffer
	"""
	def __init__(self, len):
		self.buf = [0]*len
		self.index = 0
		self.len = len
	def retrieve(self):
		""" Retrieves buffer """
		return self.buf[(self.index+1):self.len] + self.buf[0:(self.index+1)]
	def insert(self,val):
		self.index = (self.index+1)%self.len
		self.buf[self.index] = val
	def at(self, index):
		return self.buf[ (self.index+index)% self.len]
	def __len__(self):
		return self.len

def draw_angle_error():
	global eulers
	angle_buffer = CyclicBuffer(100)
	scene3 = display(title='Graph of position',
    width=600, height=200,
    center=(5,0,0), background=(0,1,1))		
	scene3.select()
	
	MAX_T = 5
	
	gd = visual.graph.gdisplay(x=0, y=0, width=900, height=250, title='N vs. t', xtitle='t', ytitle='N', 
		foreground=color.black, background=color.black, xmax=MAX_T, xmin=0, ymax=90, ymin=-90)
		
	
	f1 = visual.graph.gcurve(color=color.green )	 # a graphics curve	
	n = 0
	while True:
		n += 0.01
		
		
		scene3.select()
		angle_buffer.insert(eulers[0])
		
		if(n>MAX_T):
			f1.gcurve.visible = False
			f1 = visual.graph.gcurve(color=color.green )	 # a graphics curve	
			n=0
		
		buffer = angle_buffer.retrieve()
		angles = get_angles()
		#for n,angle in enumerate(buffer):
		f1.plot(pos=(n,angles[1]))	# plot
		
		time.sleep(0.01)
				
		
def draw_board():
	
	global current_eulers, eulers
	
	
	
	scene2 = display(title='Rotating the board', x=0, y=0, width=600, height=600, center=(5,0,0), background=(255,228,196))
	scene2.select()
	
	lamp = local_light(pos=(1,1,1), color=color.yellow)	
	scene2.select()
	board = box(pos=(0,0,0), size=(4,2,3), color=color.red)
	
	scene2.select()
	
	#arrow(pos=vector(5,0,0), axis=vector(3,0,0), color=color.yellow)
	#arrow(pos=vector(5,0,0), axis=vector(0,3,0), color=color.yellow)
	#arrow(pos=vector(5,0,0), axis=vector(0,0,3), color=color.yellow)
	
	
	scene2.select()
	x = 0
	while True:
		scene2.select()
		board.rotate(angle = eulers[0] - current_eulers[0] , axis = x_axis)
		board.rotate(angle = -(eulers[1] - current_eulers[1]) , axis = z_axis)
		board.rotate(angle = eulers[2] - current_eulers[2] , axis = y_axis)
	
		current_eulers = list(eulers)

		time.sleep(0.1)
		


def test_draw_board():
	x = 0.01
	while True:
		x += 0.01
		angles = get_angles()
		angles[0] = 90 *sin(x)
		set_angles(angles)
		time.sleep(0.01)
		
def monitor_serial_angles():
	import serial
	ser = serial.Serial('COM3', 9600)	
	while True:
		angles = ser.readline().split(";")[0:3]
		set_angles([float(x) for x in angles])
		time.sleep(0.1)

def get_angles():
	global eulers
	return [180*(x/3.14) for x in eulers]
	
def set_angles(angles):
	global eulers
	eulers = [3.14*(x/180.0) for x in angles]	


	
def main():
	global eulers, current_eulers

	print eulers
	eulers[0] = 0
	eulers[1] = 0
	eulers[2] = 0
	current_eulers[0] = 0
	current_eulers[1] = 0
	current_eulers[2] = 0


	
	
	thread.start_new_thread(draw_board,())
	thread.start_new_thread(draw_angle_error,())
	thread.start_new_thread(monitor_serial_angles())
	#thread.start_new_thread(test_draw_board,())
	while True:
		pass
	
if __name__=="__main__":
	main()
