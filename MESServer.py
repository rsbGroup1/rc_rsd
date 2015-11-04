from socket import *
from socket import error as SocketError
import errno
import thread

BUFF = 1024
ADR = ('10.115.253.233', 21240)
MAX_BRICKS = 10

def response(key):
    return 'Server response: ' + key

def handler(clientsock, addr):
    while True:
	redBricks = -1
	blueBricks = -1
	yellowBricks = -1
	mobileRobot = 0
	cell = 0

	while mobileRobot < 1 or mobileRobot > 3:
		try:
		    	mobileRobot = int(raw_input('Enter mobile robot number(1-3): '))
		except ValueError:
		    	mobileRobot = 0

	while cell < 1 or cell > 3:
		try:
			cell = int(raw_input('Enter cell number(1-3): '))
		except ValueError:
		    	cell = 0

	while redBricks < 0 or redBricks > MAX_BRICKS:
		try:
	   		redBricks = int(raw_input('Enter number of red bricks(0-' + str(MAX_BRICKS) + '): '))
		except ValueError:
	    		redBricks = -1

	while blueBricks < 0 or blueBricks > MAX_BRICKS:
		try:
		    	blueBricks = int(raw_input('Enter number of blue bricks(0-' + str(MAX_BRICKS) + '): '))
		except ValueError:
		    	blueBricks = -1

	while yellowBricks < 0 or yellowBricks > MAX_BRICKS:
		try:
		    	yellowBricks = int(raw_input('Enter number of yellow bricks(0-' + str(MAX_BRICKS) + '): '))
		except ValueError:
		    	yellowBricks = -1

	sendMsg = '<MESServer>' + '<MobileRobot>' + str(mobileRobot) + '</MobileRobot>' + '<Cell>' + str(cell) + '</Cell>' + '<Red>' + str(redBricks) + '</Red>' + '<Blue>' + str(blueBricks) + '</Blue>' + '<Yellow>' + str(yellowBricks) + '</Yellow>' + '</MESServer>' 

	try:
		clientsock.send(sendMsg)
	except SocketError as e:
    		break 

	print 'Sent: ' + sendMsg + '\n\n'

    clientsock.close()
    print addr, "- closed connection" #log on console

if __name__=='__main__':
    serversock = socket(AF_INET, SOCK_STREAM)
    serversock.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
    serversock.bind(ADR)
    serversock.listen(5)
    while 1:
        print 'Waiting for connection... listening on port', ADR[1]
        clientsock, addr = serversock.accept()
        print '...connected from:', addr
        thread.start_new_thread(handler, (clientsock, addr))

