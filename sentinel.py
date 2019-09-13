import cherrypy
import os, os.path
import time
import glob
import subprocess
import select
import datetime
import json
import ephem
import math

from shutil import rmtree
from threading import Thread
from queue import Queue, Empty
from time import localtime, strftime

from subprocess import Popen, PIPE, run

class SentinelServer(object):

    def __init__(self):
        self.sentinelProcess = None
        self.cmdQueue = Queue()
        self.startTime = "21:30"
        self.stopTime  = "05:00"
        self.devName = "/dev/video2"
        self.archivePath = "none"
        self.maxPercentUsage = 90.0

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
            
        print("Auto exposure at: ", strftime("%Y-%m-%d %H:%M:%S", localtime()))
        run(['v4l2-ctl', '-d', self.devName, '-c', 'exposure_auto=3'])
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
            print("Fixed exposure at: ", strftime("%Y-%m-%d %H:%M:%S", localtime()))
            run(['v4l2-ctl', '-d', self.devName, '-c', 'exposure_auto=1'])
            run(['v4l2-ctl', '-d', self.devName, '-c', 'exposure_absolute=333'])
            return

        response = self.funnelCmd("get_zenith_amplitude")
        if float(response["response"]) > 230.0:
            print("Auto exposure at: ", strftime("%Y-%m-%d %H:%M:%S", localtime()))
            run(['v4l2-ctl', '-d', self.devName, '-c', 'exposure_auto=3'])

    def handleSentinelCommand(self,obj):
        cmd = obj['cmd']+"\n"
        queue = obj['queue']

        #Send command to Sentinel Process
        self.sentinelProcess.stdin.write(cmd)
        self.sentinelProcess.stdin.flush()

        #Wait for response from Sentinel Process
        while True:
            fdlst = self.y.poll(1000)
            for fd,flags in fdlst:
                if flags & (select.POLLERR|select.POLLHUP|select.POLLNVAL):
                    print("Sentinel communications error")
                    queue.put({'response': 'Sentinel comm error'})
                    return

            if not fdlst:
                print("handleSentinelCommand: timeout")
                queue.put({'response': 'Sentinel comm timeout'})
                return

            line = self.sentinelProcess.stdout.readline()
            if not line:
                print("handleSentinelCommand: readline failed")
                queue.put({'response': 'Sentinel comm read failure'})
                return

            #True responses start with a '='
            n = line.find('=')
            if n >= 0:
                queue.put({'response': line[n+1:-1]})
                break
            else:
                print(line,end='')

    def pruneArchive(self):
        stat = os.statvfs(self.archivePath)
        usage = 100.0 * (1.0 - stat.f_bavail/stat.f_blocks)
        # print("Percent usage: %7.1f" % usage)
        if usage > self.maxPercentUsage:
            lst = os.listdir(self.archivePath)
            lst.sort()
            if len(lst) >= 2:
                rmtree(os.path.join(self.archivePath,lst[0]))

    def backgroundProcess(self):
        #Wait for engine to start up
        while cherrypy.engine.state != cherrypy.engine.states.STARTED:
            time.sleep(1)

        lastTime = 'XX:XX'
        f = open("state.json")
        if f:
            s = f.read()
            data = json.loads(s)
            self.handle_set_state(data)

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

                if self.archivePath != "none":
                    self.pruneArchive()

            lastTime = tnow
            time.sleep(10)

    def commProcess(self):
        #Wait for engine to start up
        while cherrypy.engine.state != cherrypy.engine.states.STARTED:
            time.sleep(1)

        while cherrypy.engine.state == cherrypy.engine.states.STARTED:
            #Dispose of miscellaneous output from Sentinel Process
            fdlist = self.y.poll(1)
            for fd, flags in fdlist:
                if flags & (select.POLLERR|select.POLLHUP):
                    print("Sentinel communications error")
                    return

                if flags & select.POLLIN:
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
        self.y.register(self.sentinelProcess.stdout, select.POLLIN|select.POLLERR|select.POLLHUP|select.POLLNVAL)
        background = Thread(target = self.backgroundProcess)
        background.daemon = True
        background.start()
        comm = Thread(target = self.commProcess)
        comm.daemon = True
        comm.start()            

    def testThread(self):
        for i in range(10):
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
        thread = Thread(target = self.testThread)
        thread.daemon = True
        thread.start();
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
                if os.path.exists( fromPath+".mp4"):
                    os.rename( fromPath+".mp4",  toPath+".mp4" )
                if os.path.exists( fromPath+".h264"):
                    os.rename( fromPath+".h264", toPath+".h264")
                if os.path.exists( fromPath+".txt"):
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

    def handle_set_state(self,data):
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

    @cherrypy.expose
    @cherrypy.tools.json_in()
    @cherrypy.tools.json_out()
    def set_state(self):
        print("set_state")
        data = cherrypy.request.json
        self.handle_set_state(data)

        f = open("state.json","w")
        if f:
            s = json.dumps(data,sort_keys=True, indent=4)
            f.write(s)
            f.close()

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

    @cherrypy.expose
    @cherrypy.tools.json_in()
    @cherrypy.tools.json_out()
    def average(self):
        data = cherrypy.request.json
        path = data["path"]
        tpath = path.replace(".h264",".txt")
        f = open(tpath)
        if not f:
            return { "response": "Text file not found"}

        lines = f.readlines()
        frames = len(lines)
        r = self.funnelCmd("set_star_movie_frame_count %d" % frames)
        if r["response"] != "OK":
            return r

        r = self.funnelCmd("average %s" % data["path"])
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

        firstMinute = int( tm / 60 )
        lastMinute  = int( tp / 60 )

        l = []

        for minute in range(firstMinute,lastMinute+1):
            path0 = f"{self.archivePath}/s{minute//60}/s{minute}.h264"
            if path0 not in l and os.path.exists(path0) and os.path.exists(path0.replace(".h264",".txt")):
                l.append(path0)

        if len(l) == 0:
            return { "response": "Archive file not found"}

        utc = datetime.datetime.utcfromtimestamp(t0)
        playbackPath = utc.strftime("new/s%Y%m%d_%H%M%S.h264")

        pbv = None
        pbt = None
        syncFound = False
        count = 0
        firstFrameTime = 0.0
        lastFrameTime = 0.0

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

                if count == 1:
                    firstFrameTime = ftime
                lastFrameTime = ftime

            fv.close()
            ft.close()

        if pbv == None:
            return { "result": "Time period not in archive file"}

        pbv.close()
        pbt.close()

        framesPerSecond = 30
        if lastFrameTime != firstFrameTime:
            framesPerSecond = int(round((count-1)/(lastFrameTime-firstFrameTime)))

        mp4File = playbackPath.replace(".h264",".mp4")
        subprocess.run(["MP4Box", "-add",  playbackPath, "-fps", "%d" % framesPerSecond, "-quiet", "-new", mp4File]) 

        return { "response": "OK"}

    @cherrypy.expose
    @cherrypy.tools.json_in()
    @cherrypy.tools.json_out()
    def stars(self):
        starList = [ "Sirius", "Canopus", "Rigil Kentaurus", "Arcturus", "Vega", "Capella", "Rigel", "Procyon",
                     "Achernar", "Betelgeuse", "Hadar", "Altair", "Acrux", "Aldebaran", "Antares", "Spica", "Pollux", "Fomalhaut",
                     "Deneb", "Mimosa", "Regulus", "Adhara", "Shaula", "Castor", "Gacrux" ]
        planetList = ["Mars", "Venus", "Jupiter", "Saturn"]

        data = cherrypy.request.json
        observer = ephem.Observer()
        observer.lon = str(data["cameraLongitude"])
        observer.lat = str(data["cameraLatitude"])
        observer.elevation = data["cameraElevation"]

        path = os.path.basename(data["path"])

        year   = int(path[ 1: 5])
        month  = int(path[ 5: 7])
        day    = int(path[ 7: 9])
        hour   = int(path[10:12])
        minute = int(path[12:14])
        second = int(path[14:16])

        result = []

        date = datetime.datetime(year,month,day,hour,minute,second)
        observer.date = date

        for name in starList:
            star = ephem.star(name)
            star.compute(observer)
            if star.alt > 0.0:
                azim = star.az*180.0/math.pi
                elev = star.alt*180.0/math.pi
                result.append( {"name": name, "azim": azim, "elev": elev} )

        mars = ephem.Mars(date)
        mars.compute(observer)
        if mars.alt > 0.0:
            azim = mars.az*180.0/math.pi
            elev = mars.alt*180.0/math.pi
            result.append( {"name": "Mars", "azim": azim, "elev": elev} )

        jupiter = ephem.Jupiter(date)
        jupiter.compute(observer)
        if jupiter.alt > 0.0:
            azim = jupiter.az*180.0/math.pi
            elev = jupiter.alt*180.0/math.pi
            result.append( {"name": "Jupiter", "azim": azim, "elev": elev} )

        saturn = ephem.Saturn(date)
        saturn.compute(observer)
        if saturn.alt > 0.0:
            azim = saturn.az*180.0/math.pi
            elev = saturn.alt*180.0/math.pi
            result.append( {"name": "Saturn", "azim": azim, "elev": elev} )

        return result

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
