import cherrypy
import os, os.path
import time
import glob
import subprocess
import select
import datetime

from threading import Thread
from queue import Queue, Empty

from subprocess import Popen, PIPE, run

class SentinelServer(object):

    def __init__(self):
        self.sentinelProcess = None
        self.cmdQueue = Queue()
        self.startTime = "21:30"
        self.stopTime  = "05:00"
        self.devName = "/dev/video2"
        self.archivePath = "none"

        if not os.path.exists("new"):
            os.mkdir("new")
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
            run(['v4l2-ctl', '-d', self.devName, '-c', 'exposure_auto=1'])
            run(['v4l2-ctl', '-d', self.devName, '-c', 'exposure_absolute=333'])
            return

        response = self.funnelCmd("get_zenith_amplitude")
        if float(response["response"]) > 230.0:
            run(['v4l2-ctl', '-d', self.devName, '-c', 'exposure_auto=3'])

    def handleSentinelCommand(self,obj):
        cmd = obj['cmd']+"\n"
        queue = obj['queue']

        #Send command to Sentinel Process
        self.sentinelProcess.stdin.write(cmd)
        self.sentinelProcess.stdin.flush()

        #Wait for response from Sentinel Process
        while True:
            if not self.y.poll(1000):
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
                if tnow == self.startTime and self.startTime != self.stopTime:
                    self.runStartSequence()
                elif tnow == self.stopTime and self.startTime != self.stopTime:
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
        self.sentinelProcess = Popen(['./sentinel.bin', '-s'], stdin=PIPE, stdout=PIPE, shell=False, universal_newlines=True)
        self.y = select.poll()
        self.y.register(self.sentinelProcess.stdout, select.POLLIN)
        self.background = Thread(target = self.backgroundProcess)
        self.background.start()
        self.comm = Thread(target = self.commProcess)
        self.comm.start()

    def testThread(self):
        for i in range(20):
            time.sleep(69)
            response = self.funnelCmd("start")
            if response["response"] != "OK":
                print("Start failed")
                return

            time.sleep(61)
            self.funnelCmd("force_trigger")
            if response["response"] != "OK":
                print("Force Trigger failed")
                return

            time.sleep(53)
            response = self.funnelCmd("stop")
            if response["response"] != "OK":
                print("Stop failed")
                return

        print("End of Test")

    @cherrypy.expose
    @cherrypy.tools.json_out()
    def test(self):
        print("test")
        self.test = Thread(target = self.testThread)
        self.test.start();
        return { "response": "OK" }

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
                if os.path.exists( fromPath+".jpg"):
                    os.rename( fromPath+".jpg", toPath+".jpg" )
                if os.path.exists( fromPath+"m.jpeg"):
                    os.rename( fromPath+"m.jpeg", toPath+"m.jpg")

    @cherrypy.expose
    @cherrypy.tools.json_in()
    @cherrypy.tools.json_out()
    def new(self):
        print("new")
        data = cherrypy.request.json
        self.relocateCurrent(data)

        mp4files = glob.glob("new/s*.mp4")
        mp4files = sorted(list(map(os.path.basename,mp4files)))
        return mp4files

    @cherrypy.expose
    @cherrypy.tools.json_in()
    @cherrypy.tools.json_out()
    def saved(self):
        print("saved")
        data = cherrypy.request.json
        self.relocateCurrent(data)

        mp4files = glob.glob("saved/s*.mp4")
        mp4files = sorted(list(map(os.path.basename,mp4files)))
        return mp4files

    @cherrypy.expose
    @cherrypy.tools.json_in()
    @cherrypy.tools.json_out()
    def trash(self):
        print("trash")
        data = cherrypy.request.json
        self.relocateCurrent(data)

        mp4files = glob.glob("trash/s*.mp4")
        mp4files = sorted(list(map(os.path.basename,mp4files)))
        return mp4files

    @cherrypy.expose
    @cherrypy.tools.json_out()
    def get_state(self):
        print("get_state")
        response = {}
        try:
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
            r = self.funnelCmd("get_dev_name")
            response["devName"] = r["response"]
            r = self.funnelCmd("get_archive_path")
            response["archivePath"] = r["response"]
            response["startTime"] = {"h": int(self.startTime[0:2]), "m": int(self.startTime[3:5])}
            response["stopTime"]  = {"h": int(self.stopTime[0:2]),  "m": int(self.stopTime[3:5])}
            response["numNew"]     = len(glob.glob("new/*.mp4"))
            response["numSaved"]   = len(glob.glob("saved/*.mp4"))
            response["numTrashed"] = len(glob.glob("trash/*.mp4"))
            self.devName = response["devName"]
        except Exception as inst:
            print(inst)
            return {}

        return response

    @cherrypy.expose
    @cherrypy.tools.json_in()
    @cherrypy.tools.json_out()
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
        r = self.funnelCmd("set_dev_name %s" % data["devName"])
        if r["response"] != "OK":
            return r
        r = self.funnelCmd("set_archive_path %s" % data["archivePath"])
        if r["response"] != "OK":
            return r
        start = data["startTime"]
        stop = data["stopTime"]
        self.startTime = "%02d:%02d" % (start["h"],start["m"])
        self.stopTime  = "%02d:%02d" % (stop["h"], stop["m"])
        self.devName = data["devName"]
        self.archivePath = data["archivePath"]

        return { "response": "OK" }

    @cherrypy.expose
    @cherrypy.tools.json_in()
    @cherrypy.tools.json_out()
    def file_exists(self):
        data = cherrypy.request.json
        path = data["path"]
        if os.path.exists(path):
            return { "response": "Yes" }
        return { "response": "No" }

    @cherrypy.expose
    @cherrypy.tools.json_in()
    @cherrypy.tools.json_out()
    def compose(self):
        data = cherrypy.request.json
        path = data["path"]
        r = self.funnelCmd("compose %s" % data["path"])
        return r

    def checkForSync( self, frame ):
        word = 0xffffffff
        for c in frame:
            if word == 1:
                c = c & 0x0f
                if c == 1:
                    return False
                if c == 5:
                    return True

            word = ((word << 8) & 0xffffffff) | c

        return False

    @cherrypy.expose
    @cherrypy.tools.json_in()
    @cherrypy.tools.json_out()
    def playback(self):
        if self.archivePath == "none":
            return { "response": "No archive path"}

        data = cherrypy.request.json
        localTime = datetime.datetime(data["year"], data["month"], data["day"],
                               data["hour"], data["minute"], data["second"])
        dt = localTime.astimezone()

        t0 = dt.timestamp()
        tm = t0 - 2
        tp = t0 + data["duration"]
        path0 = f"{self.archivePath}/s{int(t0/3600)}/s{int(t0/60)}.h264"
        pathm = f"{self.archivePath}/s{int(tm/3600)}/s{int(tm/60)}.h264"
        pathp = f"{self.archivePath}/s{int(tp/3600)}/s{int(tp/60)}.h264"

        l = []

        if os.path.exists(pathm) and os.path.exists(pathm.replace(".h264",".txt")):
            l.append(pathm)
        if path0 not in l and os.path.exists(path0) and os.path.exists(path0.replace(".h264",".txt")):
            l.append(path0)
        if pathp not in l and os.path.exists(pathp) and os.path.exists(pathp.replace(".h264",".txt")):
            l.append(pathp)

        if len(l) == 0:
            return { "response": "Archive file not found"}

        utc = datetime.datetime.utcfromtimestamp(t0)
        playbackPath = utc.strftime("new/s%Y%m%d_%H%M%S.h264")

        pbv = None
        pbt = None
        syncFound = False
        count = 0

        for path in l:
            fv = open(path,"rb")
            ft = open(path.replace(".h264",".txt"),"r")

            offset = 0

            while True:
                line = ft.readline()
                if not line:
                    break

                items = line.split()
                ftime = float(items[0])
                fsize = int(items[1])

                if ftime > tp:
                    break

                if ftime < tm:
                    fv.seek(fsize,1)
                    continue

                frame = fv.read(fsize)
                if not syncFound and not self.checkForSync(frame):
                    continue

                syncFound = True
                if pbv == None:
                    pbv = open(playbackPath,"wb")
                    pbt = open(playbackPath.replace(".h264",".txt"),"w")

                count = count+1
                pbv.write(frame)
                pbt.write("%3d %7.3f\n" % (count,ftime-t0))

            fv.close()
            ft.close()

        if pbv == None:
            return { "result": "Time period not in archive file"}

        pbv.close()
        pbt.close()

        mp4File = playbackPath.replace(".h264",".mp4")
        subprocess.run(["MP4Box", "-add",  playbackPath, "-fps", "30", "-quiet", mp4File]) 

        return { "response": "OK"}

    @cherrypy.expose
    @cherrypy.tools.json_out()
    def toggle(self):
        self.toggleStartStop()
        return { "response": "OK"}

    @cherrypy.expose
    @cherrypy.tools.json_out()
    def force_trigger(self):
        r = self.funnelCmd("get_running")
        if r["response"] != "Yes":
            return { "response": "No"}

        return self.funnelCmd("force_trigger")

    @cherrypy.expose
    def shutdown(self):
        self.funnelCmd("quit")
        time.sleep(2)
        cherrypy.engine.exit()

conf =  { 
            '/': {
                'tools.staticdir.root': os.path.abspath(os.getcwd()),
                'tools.staticdir.on': True,
                'tools.staticdir.dir': 'public'
            },
            '/new': {
                'tools.staticdir.on': True,
                'tools.staticdir.dir': 'new'
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
