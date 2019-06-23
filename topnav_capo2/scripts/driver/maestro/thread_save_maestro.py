from threading import Lock

import maestro
from driver.maestro.singleton_metaclass import Singleton


class ThreadSaveController:
    __metaclass__ = Singleton

    def __init__(self, ttyStr='/dev/ttyACM0', device=0x0c):
        print '[ThreadSaveController] init called'
        self.lock = Lock()
        self._controller = maestro.Controller(ttyStr, device)

    # Cleanup by closing USB serial port
    def close(self):
        self.lock.acquire()
        self._controller.usb.close()
        self.lock.release()

    # Send a Pololu command out the serial port
    def sendCmd(self, cmd):
        self.lock.acquire()
        self._controller.sendCmd(cmd)

    def setRange(self, chan, min, max):
        self.lock.acquire()
        self._controller.setRange(chan, min, max)

    def getMin(self, chan):
        self.lock.acquire()
        min = self._controller.getMin(chan)
        self.lock.release()
        return min

    def getMax(self, chan):
        self.lock.acquire()
        max = self._controller.getMax(chan)
        self.lock.release()
        return max

    def setTarget(self, chan, target):
        self.lock.acquire()
        self._controller.setTarget(chan, target)

    def setSpeed(self, chan, speed):
        self.lock.acquire()
        self._controller.setSpeed(chan, speed)

    def setAccel(self, chan, accel):
        self.lock.acquire()
        accel = self._controller.setAccel(chan, accel)
        self.lock.release()
        return accel

    def getPosition(self, chan):
        self.lock.acquire()
        position = self._controller.getPosition(chan)
        self.lock.release()
        return position

    def isMoving(self, chan):
        self.lock.acquire()
        return self._controller.isMoving(chan)

    def getMovingState(self):
        self.lock.acquire()
        state = self._controller.getMovingState()
        self.lock.release()
        return state


    def runScriptSub(self, subNumber):
        self.lock.acquire()
        self._controller.runScriptSub(subNumber)
        self.lock.release()

    def stopScript(self):
        self.lock.acquire()
        self._controller.stopScript()
        self.lock.release()
