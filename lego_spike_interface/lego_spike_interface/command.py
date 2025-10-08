#!/usr/bin/env python3
"""
Commands sent over the serial interface to the Lego hub
"""

from threading import Lock
from json import dumps


class CommandList:
    ACTION_LIGHTS = "lights"
    ACTION_MOTORS = "motors"

    def __init__(self, actions=[], parameters=[]):
        self.actions = actions
        self.parameters = parameters
        self.mutex = Lock()

    def __str__(self):
        if len(self) > 0:

            resp = dict({"actions": self.actions, "parameters": self.parameters})
            resp = dumps(resp)
        else:
            resp = ""
        return resp

    def __len__(self):
        return len(self.actions)

    def append(self, action, parameter):
        print(action, parameter)
        self.mutex.acquire()
        self.actions.append(action)
        self.parameters.append(parameter)
        self.mutex.release()

    def clear(self):
        self.mutex.acquire()
        self.actions = []
        self.parameters = []
        self.mutex.release()

    def transmit(self, interface):
        self.mutex.acquire()
        if len(self) > 0:

            interface.write_line(str(self))
            # clear, but we already have the lock!
            self.actions = []
            self.parameters = []
        else:
            interface.write_line(
                ""
            )  # write an empty line so there's something to receive on the other end
        self.mutex.release()
