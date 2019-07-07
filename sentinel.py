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

        if not os.path.exists("events"):
            os.mkdir("events")
        if not os.path.exists("saved"):
            os.mkdir("saved")
        if not os.path.exists("trash"):
            os.mkdir("trash")

    def runStartSequence(self):
        response = self.funnelCmd("get_running")
        if response["response"] == "Yes":
            return

        response = self.funnelCmd("start")
        if response["response"] != "OK":
            print("Start failed")
            return

    def runStopSequence(self):
        response = self.funnelCmd("get_running")
        if response["response"] == "No":
            return
            
        response = self.funnelCmd("stop")
        if response["response"] != "OK":
            print("Stop failed")
            return

    def toggleStartStop(self):
        response = self.funnelCmd("get_running")
        if response["response"] == "Yes":
            self.runStopSequence()
        elif response["response"] == "No":
            self.runStartSequence()
        else:
            print("Toggle Start/Stop failed")

    def checkExposure(self):
        response = self.funnelCmd("get_running")
        if response["response"] != "Yes":
            return

        response = self.funnelCmd("get_frame_rate")
        if float(response["response"]) < 19.0:
            run(['v4l2-ctl', '-d', '/dev/video2', '-c', 'exposure_auto=1'])
            run(['v4l2-ctl', '-d', '/dev/video2', '-c', 'exposure_absolute=333'])
            return

        response = self.funnelCmd("get_zenith_amplitude")
        if float(response["response"]) > 230.0:
            run(['v4l2-ctl', '-d', '/dev/video2', '-c', 'exposure_auto=3'])

    def handleSentinelCommand(self,obj):
        cmd = obj['cmd']+"\n"
        queue = obj['queue']

        #Send command to Sentinel Process
        self.sentinelProcess.stdin.write(cmd)
        self.sentinelProcess.stdin.flush()

        #Wait for response from Sentinel Process
        while True:
            if not self.y.poll(500):
                print("handleSentinelCommand: poll failed")
                return

            line = self.sentinelProcess.stdout.readline()
            if not line:
                print("handleSentinelCommand: readline failed")
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

        lastTime = 'XX:XX'
        while cherrypy.engine.state == cherrypy.engine.states.STARTED:
            #Check for timed actions
            tnow = time.strftime('%H:%M')
            if tnow != lastTime:
                if tnow == self.startTime:
                    self.runStartSequence()
                elif tnow == self.stopTime:
                    self.runStopSequence()
                else:
                    self.checkExposure()

            lastTime = tnow
            time.sleep(10)

    def commProcess(self):
        #Wait for engine to start up
        while cherrypy.engine.state != cherrypy.engine.states.STARTED:
            time.sleep(1)

        while cherrypy.engine.state == cherrypy.engine.states.STARTED:
            #Dispose of miscellaneous output from Sentinel Process
            while self.y.poll(1):
                print(self.sentinelProcess.stdout.readline(),end='')

            #Check for commands from web
            try:
                obj = self.cmdQueue.get(timeout=10)
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
        self.comm = Thread(target = self.commProcess)
        self.comm.start()

    @cherrypy.expose
    def index(self):
        return open('public/index.html')

    @cherrypy.expose
    @cherrypy.tools.json_out()
    def start(self):
        print("start")
        return self.funnelCmd("start")

    @cherrypy.expose
    @cherrypy.tools.json_out()
    def stop(self):
        print("stop")
        return self.funnelCmd("stop")

    @cherrypy.expose
    @cherrypy.tools.json_out()
    def get_running(self):
        print("get_running")
        return self.funnelCmd("get_running")

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

    @cherrypy.expose
    @cherrypy.tools.json_out()
    def get_state(self):
        print("get_state")
        response = {}
        r = self.funnelCmd("get_frame_rate")
        response["frameRate"] = float(r["response"])
        r = self.funnelCmd("get_zenith_amplitude")
        response["zenithAmplitude"] = float(r["response"])
        r = self.funnelCmd("get_noise")
        response["noiseThreshold"] = int(r["response"])
        r = self.funnelCmd("get_sum_threshold")
        response["sumThreshold"] = int(r["response"])
        r = self.funnelCmd("get_max_events_per_hour")
        response["eventsPerHour"] = int(r["response"])
        r = self.funnelCmd("get_running")
        response["running"] = r["response"]
        response["startTime"] = {"h": int(self.startTime[0:2]), "m": int(self.startTime[3:5])}
        response["stopTime"]  = {"h": int(self.stopTime[0:2]),  "m": int(self.stopTime[3:5])}
        return response

    @cherrypy.expose
    @cherrypy.tools.json_in()
    def set_state(self):
        print("set_state")
        data = cherrypy.request.json
        r = self.funnelCmd("set_noise %d" % data["noiseThreshold"])
        if r["response"] != "OK":
            return r
        r = self.funnelCmd("set_sum_threshold %d" % data["sumThreshold"])
        if r["response"] != "OK":
            return r
        r = self.funnelCmd("set_max_events_per_hour %d" % data["eventsPerHour"])
        if r["response"] != "OK":
            return r
        start = data["startTime"]
        stop = data["stopTime"]
        self.startTime = "%02d:%02d" % (start["h"],start["m"])
        self.stopTime  = "%02d:%02d" % (stop["h"], stop["m"])

        return { "response": "OK" }

    @cherrypy.expose
    def toggle(self):
        self.toggleStartStop()
        return { "response": "OK"}

    @cherrypy.expose
    def force_trigger(self):
        r = self.funnelCmd("get_running")
        if r["response"] != "Yes":
            return { "response": "No"}

        return self.funnelCmd("force_trigger")

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
