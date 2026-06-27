"""status_fetcher.py

Thread Which Handles Receiving Status Data with ZMQ

"""

import zmq
from threading import Thread, Event
from time import sleep
import json
import logging
from typing import Any, Callable, Dict, Optional, Tuple
from ...daemon.telescope_types import DaemonStatus, RotorState


class StatusThread(Thread):
    """
    Thread Which Handles Receiving Status Data
    """

    def __init__(self, group=None, target=None, name=None, port=5555):
        """Initializer for StatusThread

        Parameters
        ----------
        group : NoneType
            The ThreadGroup the Thread Belongs to (Currently Unimplemented in Python 3.8)
        target : callable
            Function that the Thread Should Run (Leave This Be For Command Sending)
        name : str
            Name of the Thread
        port : int
            Port of the Status Data ZMQ PUB/SUB Socket
        """
        super().__init__(group=group, target=target, name=name, daemon=True)
        self.status: Optional[DaemonStatus] = None
        self.port = port
        self.exit_event = Event()

    def run(self):
        """Grabs Most Recent Status From ZMQ and Stores

        Returns
        -------

        """
        context = zmq.Context()
        socket = context.socket(zmq.SUB)

        socket.connect("tcp://localhost:%s" % self.port)
        socket.subscribe("")

        logging.warning("StatusThread connected to port %s", self.port)
        
        while not self.exit_event.is_set():
            try:
                rec = socket.recv()
                dump = json.loads(rec)
                self.status = DaemonStatus.from_dict(dump)
            except zmq.error.ZMQError as e:
                logging.error("StatusThread recv error: %s", str(e))
                sleep(0.1)

    def get_status(self) -> DaemonStatus | None:
        """Return Most Recent Status Dictionary

        Returns
        -------
        dict
            Status Dictionary
        """
        return self.status


if __name__ == "__main__":
    thread = StatusThread()
    thread.start()
    sleep(1)
    print(thread.get_status())
