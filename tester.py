import random
import liblo, time


debugserver=liblo.Address(12000)

while 1:
	x = random.randrange(0, 40)
	y = random.randrange(0, 40)
	z = random.randrange(0, 40)
	axis1 = random.randrange(-600, 600)
	axis2 = random.randrange(-600, 600)

	liblo.send(debugserver, "/Front", x)
	liblo.send(debugserver, "/Left", y)
	liblo.send(debugserver, "/Right", z)
	liblo.send(debugserver, "/X", axis1)
	liblo.send(debugserver, "/Y", axis2)
	time.sleep(0.01)
