import cherrypy
import os, os.path
import time
import glob
import subprocess
import select

from threading import Thread
from queue import Queue, Empty

from subprocess import Popen, PIPE, run

class SentinelServer(object):

    def __init__(self):
        self.sentinelProcess = None
        self.cmdQueue = Queue()
        self.startTime = "21:30"
        self.stopTime  = "05:00"
        self.sentinelRunning = False

        if not os.path.exists("events"):
            os.mkdir("events")
        if not os.path.exists("saved"):
            os.mkdir("saved")
        if not os.path.exists("trash"):
            os.mkdir("trash")

    def runStartSequence(self):
        queue = Queue()
        run(['v4l2-ctl', '-d', '/dev/video2', '-c', 'exposure_auto=3'])
        self.handleSentinelCommand({"cmd": "start\n", "queue": queue})

        time.sleep(10)
        run(['v4l2-ctl', '-d', '/dev/video2', '-c', 'exposure_auto=1'])
        self.sentinelRunning = True

    def runStopSequence(self):
        queue = Queue()
        run(['v4l2-ctl', '-d', '/dev/video2', '-c', 'exposure_auto=3'])
        time.sleep(10)

        self.handleSentinelCommand({"cmd": "stop\n", "queue": queue})
        self.sentinelRunning = False

    def handleSentinelCommand(self,obj):
        cmd = obj['cmd']
        queue = obj['queue']

        #Send command to Sentinel Process
        self.sentinelProcess.stdin.write(cmd)
        self.sentinelProcess.stdin.flush()

        #Wait for response from Sentinel Process
        while True:
            if not self.y.poll(500):
                return

            line = self.sentinelProcess.stdout.readline()
            if not line:
                return

            #True responses start with a '='
            n = line.find('=')
            if n >= 0:
                queue.put({'response': line[n+1:-1]})
                break
            else:
                print(line,end='')

    def backgroundProcess(self):
        #Wait for engine to start up
        while cherrypy.engine.state != cherrypy.engine.states.STARTED:
            time.sleep(1)

        while cherrypy.engine.state == cherrypy.engine.states.STARTED:
            #Dispose of miscellaneous output from Sentinel Process
            while self.y.poll(1):
                print(self.sentinelProcess.stdout.readline(),end='')

            #Check for timed actions
            tnow = time.strftime('%H:%M')
            if not self.sentinelRunning and tnow == self.startTime:
                self.runStartSequence()
            elif self.sentinelRunning and tnow == self.stopTime:
                self.runStopSequence()

            #Check for commands from web
            try:
                obj = self.cmdQueue.get(timeout=1)
                self.handleSentinelCommand(obj)
            except:
                pass

    def funnelCmd( self, cmd ):
        responseQueue = Queue()
        self.cmdQueue.put( {"cmd": cmd, "queue": responseQueue})
        try:
            response = responseQueue.get(timeout=1.0)
        except:
            response = {"response": None}
        return response
        
    def connectToSentinel(self):
        self.sentinelProcess = Popen(['./sentinel.bin', '-i'], stdin=PIPE, stdout=PIPE, shell=False, universal_newlines=True)
        self.y = select.poll()
        self.y.register(self.sentinelProcess.stdout, select.POLLIN)
        self.background = Thread(target = self.backgroundProcess)
        self.background.start()

    @cherrypy.expose
    def index(self):
        return open('public/index.html')

    @cherrypy.expose
    @cherrypy.tools.json_out()
    def start(self):
        print("start")
        return self.funnelCmd("start\n")

    @cherrypy.expose
    @cherrypy.tools.json_out()
    def stop(self):
        print("stop")
        return self.funnelCmd("stop\n")

    @cherrypy.expose
    @cherrypy.tools.json_out()
    def get_running(self):
        print("get_running")
        return self.funnelCmd("get_running\n")

    def relocateCurrent(self,currentData):
        for item in currentData:
            dfrom = item["from"]
            dto = item["to"]

            if dfrom != dto:
                root = os.path.splitext(item["event"])[0]
                fromPath = os.path.join(dfrom, root)
                toPath = os.path.join(dto, root)
                os.rename( fromPath+".mp4",  toPath+".mp4" )
                os.rename( fromPath+".h264", toPath+".h264")
                os.rename( fromPath+".txt",  toPath+".txt")

    @cherrypy.expose
    @cherrypy.tools.json_in()
    @cherrypy.tools.json_out()
    def events(self):
        print("events")
        data = cherrypy.request.json
        self.relocateCurrent(data)

        mp4files = glob.glob("events/s*.mp4")
        mp4files = list(map(os.path.basename,mp4files))
        return mp4files

    @cherrypy.expose
    @cherrypy.tools.json_in()
    @cherrypy.tools.json_out()
    def saved(self):
        print("saved")
        data = cherrypy.request.json
        self.relocateCurrent(data)

        mp4files = glob.glob("saved/s*.mp4")
        mp4files = list(map(os.path.basename,mp4files))
        return mp4files

    @cherrypy.expose
    @cherrypy.tools.json_in()
    @cherrypy.tools.json_out()
    def trash(self):
        print("trash")
        data = cherrypy.request.json
        self.relocateCurrent(data)

        mp4files = glob.glob("trash/s*.mp4")
        mp4files = list(map(os.path.basename,mp4files))
        return mp4files

conf =  { 
            '/': {
                'tools.staticdir.root': os.path.abspath(os.getcwd()),
                'tools.staticdir.on': True,
                'tools.staticdir.dir': 'public'
            },
            '/events': {
                'tools.staticdir.on': True,
                'tools.staticdir.dir': 'events'
            },
            '/saved': {
                'tools.staticdir.on': True,
                'tools.staticdir.dir': 'saved'
            },
            '/trash': {
                'tools.staticdir.on': True,
                'tools.staticdir.dir': 'trash'
            }
        }


if __name__ == '__main__':
    server = SentinelServer()
    server.connectToSentinel()
    cherrypy.config.update({'server.socket_host': '0.0.0.0', 'server.socket_port': 9090})
    cherrypy.quickstart(server, '/', conf)
