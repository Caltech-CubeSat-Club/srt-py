"""command_dispatcher.py

Thread Which Handles Sending Commands to the Daemon

"""

import zmq
from threading import Thread
from queue import Queue
import logging


class CommandThread(Thread):
    """
    Thread Which Handles Sending Commands to the Daemon
    """

    def __init__(self, group=None, target=None, name=None, port=5556):
        """Initializer for the Command Thread

        Parameters
        ----------
        group : NoneType
            The ThreadGroup the Thread Belongs to (Currently Unimplemented in Python 3.8)
        target : callable
            Function that the Thread Should Run (Leave This Be For Command Sending)
        name : str
            Name of the Thread
        port : int
            Port to Access for Sending Commands
        """
        super().__init__(group=group, target=target, name=name, daemon=True)
        self.queue = Queue()
        self.port = port

    def run(self):
        """Grabs Commands from the Queue and Sends the to the Daemon

        Returns
        -------
        None
        """
        context = zmq.Context()
        socket = context.socket(zmq.PUSH)

        socket.connect("tcp://localhost:%s" % self.port)
        logging.warning("CommandThread connected to port %s", self.port)
        
        while self.is_alive():
            command = self.queue.get()
            socket.send_string(command)

    def add_to_queue(self, cmd):
        """Adds a New Item to the Queue

        Parameters
        ----------
        cmd : str
            New Command to Add to the Queue

        Returns
        -------
        None
        """
        self.queue.put(cmd)

    def get_queue_empty(self):
        """Returns if the Queue is Empty

        Returns
        -------
        bool
            If the Queue is Empty
        """
        return self.queue.empty()

    def emergency_stop(self):
        """Sends an emergency stop (SPA) command with high priority, bypassing the queue.
        
        Creates a direct PUSH socket connection to send the SPA (stop all) command immediately,
        ensuring it is not delayed by queued commands.

        Returns
        -------
        bool
            True if SPA sent successfully, False otherwise
        """
        try:
            from time import sleep
            context = zmq.Context()
            estop_socket = context.socket(zmq.PUSH)
            estop_socket.setsockopt(zmq.LINGER, 0)  # No waiting on close
            estop_socket.setsockopt(zmq.SNDHWM, 1)  # Small send buffer

            estop_socket.connect("tcp://localhost:%s" % self.port)
            sleep(0.05)  # Brief pause to ensure connection established
            estop_socket.send_string("spa")
            estop_socket.close()
            context.term()
            return True
        except zmq.error.ZMQError as e:
            return False



def test_cmd_send():  # TODO: Look into PyTest Fixtures - Having Multiple Threads in PyTest Causes Copious Warnings
    from time import sleep

    port = 5556
    num_dispatchers = 10
    num_in_queue = 5

    context = zmq.Context()
    socket = context.socket(zmq.PULL)
    socket.bind("tcp://*:%s" % port)

    def receive_func():
        while True:
            received.append(socket.recv_string())

    threads = [CommandThread(port=port) for _ in range(num_dispatchers)]
    for i, thread in enumerate(threads):
        for _ in range(num_in_queue):
            thread.add_to_queue(str(i))
    for thread in threads:
        thread.start()

    received = []
    receive_thread = Thread(target=receive_func, daemon=True)
    receive_thread.start()
    sleep(1)
    print(received)
    assert len(received) == num_in_queue * num_dispatchers
