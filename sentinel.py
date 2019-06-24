import cherrypy
import os, os.path
import time

from threading import Thread
from queue import Queue, Empty

from subprocess import Popen, PIPE

class UnexpectedEndOfStream(Exception): pass

class NonBlockingStreamReader:

    def __init__(self, stream):
        '''
        stream: the stream to read from.
                Usually a process' stdout or stderr.
        '''

        self._s = stream
        self._q = Queue()

        def _populateQueue(stream, queue):
            '''
            Collect lines from 'stream' and put them in 'quque'.
            '''

            while True:
                line = stream.readline()
                print(line)
                if line:
                    n = line.find('=')
                    if n >= 0:
                        queue.put(line[n+1:-1])
                else:
                    raise UnexpectedEndOfStream

        self._t = Thread(target = _populateQueue,
                args = (self._s, self._q))
        self._t.daemon = True
        self._t.start() #start collecting lines from the stream

    def readline(self, timeout = None):
        try:
            return self._q.get(block = timeout is not None,
                    timeout = timeout)
        except Empty:
            return None

class SentinelServer(object):

    def __init__(self):
        self.sentinelProcess = None
    
    def connectToSentinel(self):
        self.sentinelProcess = Popen(['./sentinel.bin', '-i'], stdin=PIPE, stdout=PIPE, shell=False, universal_newlines=True)
        self.nsbr = NonBlockingStreamReader(self.sentinelProcess.stdout)

    @cherrypy.expose
    def index(self):
        return open('public/index.html')

    @cherrypy.expose
    @cherrypy.tools.json_out()
    def start(self):
        self.sentinelProcess.stdin.write("start\n")
        self.sentinelProcess.stdin.flush()
        response = self.nsbr.readline(1.0)
        return {"response": response}

    @cherrypy.expose
    @cherrypy.tools.json_out()
    def stop(self):
        self.sentinelProcess.stdin.write("stop\n")
        self.sentinelProcess.stdin.flush()
        response = self.nsbr.readline(1.0)
        return {"response": response}

    @cherrypy.expose
    @cherrypy.tools.json_out()
    def get_running(self):
        print("Get_running")
        self.sentinelProcess.stdin.write("get_running\n")
        self.sentinelProcess.stdin.flush()
        response = self.nsbr.readline(1.0)
        return {"response": response}

    @cherrypy.expose
    @cherrypy.tools.json_in()
    @cherrypy.tools.json_out()
    def cmd(self):
        data = cherrypy.request.json
        print(data)
        cmd = data["cmd"]
        if cmd == "start":
            return {"running": True}

        if cmd == "stop":
            return {"running": False}

        return {"running": False}


conf =  { '/': {
            'tools.staticdir.root': os.path.abspath(os.getcwd()),
            'tools.staticdir.on': True,
            'tools.staticdir.dir': 'static'
            }
        }


if __name__ == '__main__':
    server = SentinelServer()
    server.connectToSentinel()
    cherrypy.config.update({'server.socket_host': '0.0.0.0', 'server.socket_port': 9090})
    cherrypy.quickstart(server, '/', conf)
