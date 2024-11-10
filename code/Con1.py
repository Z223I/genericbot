import threading
import Queue
import time

class Con1():
    """
    Consumes two streams of data.
    """

    def __init__(self):
        self._running = True

    def terminate(self):
        self._running = False

    def run1(self, _q1):
        while self._running:
            name = threading.currentThread().getName()
            print "Consumer thread 1:  ", name
            number = _q1.get();
            print "Number: ", number
            print
#            time.sleep(3)
            _q1.task_done()
 

    def run2(self, _q2):
        while self._running:
            name = threading.currentThread().getName()
            print "Consumer thread 2:  ", name
            command = _q2.get();
            print "Here is the command... ", command
            print
#            time.sleep(4)
            _q2.task_done()
