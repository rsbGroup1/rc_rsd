# Imports
from socket import *
from socket import error as SocketError
import errno
import thread
import time

# Defines
MOBILE_ROBOT = 1
ROBOT_CELL = 2
BUFF = 1024
ADR = ('10.115.253.233', 21240)
MAX_BRICKS = 100

# Global variables
redBricks = -1
blueBricks = -1
yellowBricks = -1
mobileRobot = 0
cell = 0
numberOfConnections = 0
sendMsg = ""
mrSend = False
rcSend = False
rcToMrOrderDone = 0 # -1 = Error, 0 = Idle, 1 = Ok
mrToRcRobot = 0 # -1 = Error, 0 = Idle, 1 = Ok 
mrToRcConveyer = 0 # -1 = Error, 0 = Idle, 1 = Ok 
mrDone = False
rcDone = False
mrConnected = False
rcConnected = False

# Keyboard order handler
def keyboardHandler(dummy):
    while True:
	global redBricks 
	global blueBricks
	global yellowBricks 
	global mobileRobot
	global cell
	global sendMsg
	global mrSend
	global rcSend
	global mrDone
	global rcDone
	global mrConnected
	global rcConnected

	# Print
	if(mrConnected == False or rcConnected == False):
		print "\nWaiting for RC and MR connection!"

	# Wait for connection
	while(mrConnected == False or rcConnected == False):
		time.sleep(2)

	# Print
	print "\nOrder handling:"

	# Get input
	while mobileRobot < 1 or mobileRobot > 3:
		try:
		    	mobileRobot = int(raw_input('Enter mobile robot number(1-3): '))
		except ValueError:
		    	mobileRobot = 0

	# Check connection
	if(mrConnected == False or rcConnected == False):
		continue

	# Get input
	while cell < 1 or cell > 3:
		try:
			cell = int(raw_input('Enter cell number(1-3): '))
		except ValueError:
		    	cell = 0

	# Check connection
	if(mrConnected == False or rcConnected == False):
		continue

	# Get input
	while redBricks < 0 or redBricks > MAX_BRICKS:
		try:
	   		redBricks = int(raw_input('Enter number of red bricks(0-' + str(MAX_BRICKS) + '): '))
		except ValueError:
	    		redBricks = -1

	# Check connection
	if(mrConnected == False or rcConnected == False):
		continue

	# Get input
	while blueBricks < 0 or blueBricks > MAX_BRICKS:
		try:
		    	blueBricks = int(raw_input('Enter number of blue bricks(0-' + str(MAX_BRICKS) + '): '))
		except ValueError:
		    	blueBricks = -1

	# Check connection
	if(mrConnected == False or rcConnected == False):
		continue

	# Get input
	while yellowBricks < 0 or yellowBricks > MAX_BRICKS:
		try:
		    	yellowBricks = int(raw_input('Enter number of yellow bricks(0-' + str(MAX_BRICKS) + '): '))
		except ValueError:
		    	yellowBricks = -1

	# Check connection
	if(mrConnected == False or rcConnected == False):
		continue

	# Create MES order message
	sendMsg = '<MESServer>' + '<MobileRobot>' + str(mobileRobot) + '</MobileRobot>' + '<Cell>' + str(cell) + '</Cell>' + '<Red>' + str(redBricks) + '</Red>' + '<Blue>' + str(blueBricks) + '</Blue>' + '<Yellow>' + str(yellowBricks) + '</Yellow>' + '</MESServer>' 

	if(blueBricks == redBricks == yellowBricks == 0):
		redBricks = -1
		blueBricks = -1
		yellowBricks = -1
		mobileRobot = 0
		cell = 0
		continue

	#print '\nOrder msg: ' + sendMsg + '\n'
	mrSend = True
	rcSend = True
	
	# Wait for messages to be sent
	while(mrSend == True or rcSend == True):
		time.sleep(2)

	# Print
	print "\nMES: Orders sent!\n\nWaiting for MR to reach the conveyer belt!"

	# Wait for robot cell to be done
	while(rcDone == False):
		time.sleep(2)

	# Print
	#print "Robot cell done!"

	# Wait for mobile robot to be done
	while(mrDone == False):
		time.sleep(2)

	# Print
	#print "Mobile robot done!"

	# Reset 
	redBricks = -1
	blueBricks = -1
	yellowBricks = -1
	mobileRobot = 0
	cell = 0
	mrDone = False
	rcDone = False
	mrSend = False
	rcSend = False

# Client handler
def clientHandler(clientsock, addr):

	global sendMsg
	global rcSend
	global mrSend
	global rcToMrOrderDone
	global mrToRcRobot 
	global mrToRcConveyer
	global mrDone
	global rcDone
	global mrConnected
	global rcConnected

	# Figure if RC or MR and which cell/robot
	header = clientsock.recv(100)
	robotType = 0
	cellNumber = 0
	mobileRobotNumber = 0

	# Get type
	if "MR" in header:
		robotType = MOBILE_ROBOT
	elif "RC" in header:
	    	robotType = ROBOT_CELL

	# Get number
	if "1" in header:
	    	cellNumber = 1
	    	mobileRobotNumber = 1
	elif "2" in header:
	    	cellNumber = 2
	   	mobileRobotNumber = 2
	elif "3" in header:
	    	cellNumber = 3
	    	mobileRobotNumber = 3

	if robotType == MOBILE_ROBOT:
		# Print
		print "\nConnection from: Mobile Robot " + str(mobileRobotNumber)
		mrConnected = True

		while True:
			# ------------------------------------------------------------------------------------ #
			# Wait for message to send
			while(mrSend == False):
				time.sleep(1)
			
			# Send order
			try:
				clientsock.send(sendMsg)
			except SocketError as e:
				mrConnected = False
				break 

			# Reset
			mrSend = False
			# ------------------------------------------------------------------------------------ #



			# ------------------------------------------------------------------------------------ #
			# Wait for at-conveyer-belt message
			feedback = clientsock.recv(100)
			if "Ok" in feedback:
				mrToRcConveyer = 1
				print "\nMR: At conveyer!\n\nRC: Starting conveyer!"
			else:
				mrToRcConveyer = -1 
				print "\nMR: Error!"
			# ------------------------------------------------------------------------------------ #




			# ------------------------------------------------------------------------------------ #
			# Wait for at-robot message
			feedback = clientsock.recv(100)
			if "Ok" in feedback:
				mrToRcRobot = 1
				print "\nMR: At robot!\n\nWaiting for RC to finish order!"
			else:
				mrToRcRobot = -1 
				print "\nMR: Error!"
			# ------------------------------------------------------------------------------------ #



			# ------------------------------------------------------------------------------------ #
			# Wait for robot-cell-done message
			while(rcToMrOrderDone == 0):
				time.sleep(2)
			
			# Create message
			msg = ""
			if(rcToMrOrderDone == 1):
				msg = '<MESServer>' + '<Status>' + str(1) + '</Status>' + '</MESServer>' 
			else:# if(rcToMrOrderDone == -1):
				msg = '<MESServer>' + '<Status>' + str(2) + '</Status>' + '</MESServer>' 
				
			# Send msg
			try:
				clientsock.send(msg)
			except SocketError as e:
				mrConnected = False
				break 	

			# Reset
			rcToMrOrderDone = 0
			# ------------------------------------------------------------------------------------ #




			# ------------------------------------------------------------------------------------ #
			# Wait for done message
			feedback = clientsock.recv(100)
			if "Ok" in feedback:
				print "\nMR: Home!"
			else:
				print "MR: Error!"

			# Signal to keyboard handler
			mrDone = True 		
			# ------------------------------------------------------------------------------------ #

	elif robotType == ROBOT_CELL:
		# Print
		print "Connection from: Robot Cell " + str(cellNumber)
		rcConnected = True

		while True:	
			# ------------------------------------------------------------------------------------ #
			# Wait for message to send
			while(rcSend == False):
				time.sleep(2)
			
			# Send order
			try:
				clientsock.send(sendMsg)
			except SocketError as e:
				rcConnected = False
				break 

			# Reset
			rcSend = False
			# ------------------------------------------------------------------------------------ #



			# ------------------------------------------------------------------------------------ #
			# Wait for GO message (start conveyer)
			while(mrToRcConveyer == 0):
				time.sleep(2)
			
			# Create message
			msg = ""
			if(mrToRcConveyer == 1):
				msg = '<MESServer>' + '<Status>' + str(1) + '</Status>' + '</MESServer>' 
			else:# if(mrToRcConveyer == -1):
				msg = '<MESServer>' + '<Status>' + str(2) + '</Status>' + '</MESServer>' 

			# Send msg
			try:
				clientsock.send(msg)
			except SocketError as e:
				mrConnected = False
				break 	

			# Reset
			mrToRcConveyer = 0
			# ------------------------------------------------------------------------------------ #



			# ------------------------------------------------------------------------------------ #
			# Wait for GO message (start sorting)
			while(mrToRcRobot == 0):
				time.sleep(2)

			# Create message
			msg = ""
			if(mrToRcRobot == 1):
				msg = '<MESServer>' + '<Status>' + str(1) + '</Status>' + '</MESServer>' 
			else:# if(mrToRcRobot == -1):
				msg = '<MESServer>' + '<Status>' + str(2) + '</Status>' + '</MESServer>' 
				
			# Send msg
			try:
				clientsock.send(msg)
			except SocketError as e:
				mrConnected = False
				break 	

			# Reset
			mrToRcRobot = 0
			# ------------------------------------------------------------------------------------ #




			# ------------------------------------------------------------------------------------ #
			# Wait for done message
			feedback = clientsock.recv(100)
			if "Ok" in feedback:
				rcToMrOrderDone = 1
				print "\nRC: Order done!\n\nWaiting for MR to go home!"
			else:
				rcToMrOrderDone = -1 
				print "\nRC: Error!"

			# Signal to keyboard handler
			rcDone = True 	
			# ------------------------------------------------------------------------------------ #
	
	# Close
	clientsock.close()
	print addr, "- closed connection" #log on console

if __name__=='__main__':
    # Create TCP server
    serversock = socket(AF_INET, SOCK_STREAM)
    serversock.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
    serversock.bind(ADR)
    serversock.listen(5)

    # Create keyboard handler
    thread.start_new_thread(keyboardHandler, (0,))

    # Wait for connection
    while 1:
        print 'Waiting for connection... listening on port', ADR[1]
        clientsock, addr = serversock.accept()
        print '...connected from:', addr
        thread.start_new_thread(clientHandler, (clientsock, addr))

