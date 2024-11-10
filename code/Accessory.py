#/usr/bin/python

import sys
sys.path.append('/home/pi/pythondev/RelayPiPy/RelayPiPy')
from RelayPiPy import RelayPiPy

#import threading

import time
import queue
import pdb


class Accessory:
    """Accessory """

    def __init__(self):
        """__init__ initializes class variables."""
        self._running = True
        self.time_on = 2
        self.time_off = 4
        self.auto_water = False
        #self._init_relay()

    def terminate(self):
        """terminate triggers the thread to stop."""

        self._running = False

    def _init_relay(self):
        """_init_relay initializes the relay."""

        self.relay4 = RelayPiPy()

        # init list with pin numbers
        pinList = [6, 13, 19, 26]
        self.relay4.init(pinList)

        for i in range(len(pinList)):
            self.relay4.off(i)

    def is_running(self):
        """is_running returns true if the thread is running, otherwise false"""

        return self._running

    def _water_pump(self, on):
        """_water_pump turns the water pump on/off as specified by the input
        parameter.

        @type: boolean
        @param on

        @rtype: boolean
        @rparam: on """

        # Set water pump to on/off.
        relay_no = 1

        if on:
            self.relay4.on(relay_no)
        else:
            self.relay4.off(relay_no)

        return on

    def _w_p_init(self):
        pass

    def _w_p_loop(self, q_w_p):
        """_w_p_loop cycles the water pump on and off iff auto_water = True.
        It is used in the water_pump method."""

        # Process any commands from the queue.
        self._w_p_queue_check(q_w_p)

        if self.auto_water:
            on = True
            self._water_pump(on)
            time.sleep(self.time_on)

            on = False
            self._water_pump(on)
            time.sleep(self.time_off)
        else:
            time.sleep(1)

    def _w_p_queue_check(self, q_w_p):
        """_w_p_queue_check processes commands from the water pump queue.

        @type: Queue
        @param: q_w_p

        @rtype: boolean
        @rparam: on"""

        # Looking for 'won' or 'woff'.
        if not q_w_p.empty():
            command = q_w_p.get()
            command = command.lower()

            if command == 'won':
                 self.auto_water = True
            elif command == 'woff':
                 self.auto_water = False
            else:
                # Log the error
                print('water_pump invalid command: {}'.format(command))
                # Leave the auto_water attribute unchanged.

        return self.auto_water

    def water_pump(self, q_w_p):
        """water_pump is the thread which controls the water pump."""
        # pdb.set_trace()
        self._w_p_init()

        while self.is_running():
            self._w_p_loop(q_w_p)
