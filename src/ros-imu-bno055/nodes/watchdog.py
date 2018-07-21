#!/usr/bin/env python3
import os
import signal
import threading



class Watchdog():

    on_expire = None

    def __init__(self, timeout=10, on_expire = None, restart=False):
        self.timeout = timeout
        self.on_expire = on_expire
        self.restart = restart
        self._t = None

    def do_expire(self):
        if self.on_expire:
          self.on_expire()
        else:
          os.kill(os.getpid(),signal.SIGKILL)
        if self.restart:
          self._t = threading.Timer(self.timeout, self._expire)
          self.start()

    def _expire(self):
        print("\nWatchdog expire")
        self.do_expire()

    def start(self):
        if self._t is None:
            self._t = threading.Timer(self.timeout, self._expire)
            self._t.start()

    def stop(self):
        if self._t is not None:
            self._t.cancel()
            self._t = None

    def refresh(self):
        if self._t is not None:
             self.stop()
             self.start()

