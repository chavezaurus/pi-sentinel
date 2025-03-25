#!/usr/bin/env python3

import cherrypy
import os, os.path
import time
import glob
import socket
import select
import datetime
import json
import ephem
import numpy

from math import pi, cos, sin, atan2, sqrt, acos, exp

from shutil import rmtree
from threading import Thread
from queue import Queue, Empty
from time import localtime, strftime
from scipy.optimize import minimize

from subprocess import Popen, PIPE, run

def CalibrationVector(obj):
    v = [ obj["V"], obj["S"], obj["D"], obj["a0"], obj["E"],
            obj["eps"], obj["COPx"], obj["COPy"], obj["alpha"], obj["flat"] ]
    return v

def CalibrationObject(v):
    obj = { "V": v[0], "S": v[1], "D": v[2], "a0": v[3], "E": v[4],
            "eps": v[5], "COPx": v[6], "COPy": v[7], "alpha": v[8], "flat": v[9] }
    return obj

def CalibrationTruncate(obj):
    obj["V"]     = round(obj["V"],6)
    obj["S"]     = round(obj["S"],6)
    obj["D"]     = round(obj["D"],6)
    obj["a0"]    = round(obj["a0"],3)
    obj["E"]     = round(obj["E"],3)
    obj["eps"]   = round(obj["eps"],3)
    obj["COPx"]  = round(obj["COPx"],3)
    obj["COPy"]  = round(obj["COPy"],3)
    obj["alpha"] = round(obj["alpha"],6)
    obj["flat"]  = round(obj["flat"],6)

def CalibrationToRadians(obj):
    obj["a0"]  = obj["a0"]  * pi / 180.0
    obj["E"]   = obj["E"]   * pi / 180.0
    obj["eps"] = obj["eps"] * pi / 180.0

def CalibrationToDegrees(obj):
    obj["a0"]  = obj["a0"]  * 180.0 / pi
    obj["E"]   = obj["E"]   * 180.0 / pi
    obj["eps"] = obj["eps"] * 180.0 / pi

def CalibrationError(params, skyThing):
    V     = params["V"]
    S     = params["S"]
    D     = params["D"]
    a0    = params["a0"]
    E     = params["E"]
    eps   = params["eps"]
    COPx  = params["COPx"]
    COPy  = params["COPy"]
    alpha = params["alpha"]
    flat  = params["flat"]

    px = skyThing["px"]
    py = skyThing["py"]
    azim = skyThing["azim"]
    elev = skyThing["elev"]

    if flat >= 1.0:
        flat = 0.999999

    dilation = sqrt(1-flat)
    K = COPx*sin(alpha) + COPy*cos(alpha)
    L = COPy*sin(alpha) - COPx*cos(alpha)
    c = cos(alpha)*cos(alpha)*dilation + sin(alpha)*sin(alpha)/dilation
    d = sin(alpha)*cos(alpha)*dilation - sin(alpha)*cos(alpha)/dilation;
    e = -(K*cos(alpha)*dilation*dilation - COPy*dilation + L*sin(alpha))/dilation
    f = sin(alpha)*cos(alpha)*dilation - sin(alpha)*cos(alpha)/ dilation
    g = sin(alpha)*sin(alpha)*dilation + cos(alpha)*cos(alpha)/dilation
    h = -(K*sin(alpha)*dilation*dilation - COPx*dilation - L*cos(alpha))/dilation
    
    # Apply the affine transformation to the input coordinates
    # PURPOSE: Correct elliptical image distortion

    pxt = g*px + f*py + h
    pyt = d*px + c*py + e
    # print( "pxt: %f pyt: %f" % (pxt,pyt))

    x = pxt - COPx
    y = pyt - COPy

    # print( "x: %f y: %f" % (x,y))
    r = sqrt( x*x + y*y )
    u = V*r + S*(exp(D*r) - 1)
    b = a0 - E + atan2(x, y)

    # print( "b: %f" % b)
    angle = b
    z = u
    if eps != 0.0:
        z = acos(cos(u)*cos(eps)-sin(u)*sin(eps)*cos(b))
        sinAngle = sin(b)*sin(u)/sin(z)
        cosAngle = (cos(u)-cos(eps)*cos(z))/(sin(eps)*sin(z))
        angle = atan2(sinAngle,cosAngle)

    elev2 = pi/2 - z
    azim2 = angle + E - pi # Measured from cardinal NORTH.

    x2 = cos(azim2)*cos(elev2)
    y2 = sin(azim2)*cos(elev2)

    # These are in degrees so need to be converted to radians
    x1 = cos(azim*pi/180.0)*cos(elev*pi/180.0)
    y1 = sin(azim*pi/180.0)*cos(elev*pi/180.0)

    dx = x1-x2
    dy = y1-y2

    esq = dx*dx+dy*dy
    return esq

def TotalCalibrationError(calVector, skyList ):
    obj = CalibrationObject(calVector)
    sum = 0.0
    for skyThing in skyList:
        sum = sum + CalibrationError(obj,skyThing)

    # print("Sum: %f" % sum)
    return sum

def round_to_nearest_5_minutes(dt, round_down=True):
    minutes = dt.minute
    rounded_minutes = (minutes // 5) * 5 if round_down else ((minutes + 4) // 5) * 5
    return dt.replace(minute=rounded_minutes, second=0, microsecond=0)

class SentinelServer(object):

    def __init__(self):
        self.sentinelSocket = None
        self.cmdQueue = Queue()
        self.startTime = {"h": 21, "m": 30 }
        self.stopTime  = {"h": 5, "m": 0 }
        self.startUTC = {"h": 21, "m": 30 }
        self.stopUTC = {"h": 5, "m": 0 }
        self.archivePath = "none"
        self.devName = "/dev/video2"
        self.maxPercentUsage = 90.0
        self.gps_latitude = 0.0
        self.gps_longitude = 0.0
        self.gps_time_offset = 0.0
        self.analyze_and_compose = False

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
        run(['v4l2-ctl', '-d', self.devName, '-c', 'auto_exposure=3'])
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
        if float(response["response"]) < 17.0:
            print("Fixed exposure at: ", strftime("%Y-%m-%d %H:%M:%S", localtime()))
            run(['v4l2-ctl', '-d', self.devName, '-c', 'auto_exposure=1'])
            run(['v4l2-ctl', '-d', self.devName, '-c', 'exposure_time_absolute=333'])
            run(['v4l2-ctl', '-d', self.devName, '-c', 'gain=0'])
            return

        response = self.funnelCmd("get_zenith_amplitude")
        if float(response["response"]) > 230.0:
            print("Auto exposure at: ", strftime("%Y-%m-%d %H:%M:%S", localtime()))
            run(['v4l2-ctl', '-d', self.devName, '-c', 'auto_exposure=3'])

    def handleSentinelCommand(self,obj):
        cmd = obj['cmd']+"\n"
        queue = obj['queue']

        #Send command to Sentinel Process
        self.sentinelSocket.send(cmd.encode())

        #Wait for response from Sentinel Process
        response = ""
        self.sentinelSocket.settimeout(3.0)
        while not response or response[-1] != '\n':
            try:
                data = self.sentinelSocket.recv(80)
                response += data.decode()
            except socket.timeout as e:
                print(e)
                queue.put({'response': 'Sentinel comm timeout'})
                return

            if not response:
                print("Sentinel communications error")
                queue.put({'response': 'Sentinel comm error'})
                return

        queue.put({'response': response[:-1]})

    def pruneArchive(self):
        stat = os.statvfs(self.archivePath)
        usage = 100.0 * (1.0 - stat.f_bavail/stat.f_blocks)
        # print("Percent usage: %7.1f" % usage)
        if usage > self.maxPercentUsage:
            lst = [ d for d in os.listdir(self.archivePath) if d.startswith('s')]
            lst.sort()
            if len(lst) >= 2:
                rmtree(os.path.join(self.archivePath,lst[0]))
    
    def moonPosition(self, file_path):
        f = open("calibration.json")
        if not f:
            return

        s = f.read()
        calibration_data = json.loads(s)

        observer = ephem.Observer()
        observer.lon = str(calibration_data["cameraLongitude"])
        observer.lat = str(calibration_data["cameraLatitude"])
        observer.elevation = calibration_data["cameraElevation"]

        path = os.path.basename(file_path)

        year   = int(path[ 1: 5])
        month  = int(path[ 5: 7])
        day    = int(path[ 7: 9])
        hour   = int(path[10:12])
        minute = int(path[12:14])
        second = int(path[14:16])

        date = datetime.datetime(year,month,day,hour,minute,second)
        observer.date = date

        moon = ephem.Moon(date)
        moon.compute(observer)
        azim = 0.0
        elev = 0.0

        if moon.alt > 0.0:
            azim = moon.az*180.0/pi
            elev = moon.alt*180.0/pi

        return azim, elev

    def backgroundAnalyzeAndCompose(self):
                # Only do this if we are not busy
        r = self.funnelCmd("get_running")
        if r["response"] == "Yes":
            return

        self.analyze_and_compose = False
        for file in os.listdir("new"):
            if file.endswith(".h264"):
                base_name = os.path.splitext(file)[0]
                csv_path = f"new/{base_name}.csv"
                jpg_path = f"new/{base_name}.jpg"
                if not os.path.exists(csv_path):
                    azim,elev = self.moonPosition( file )
                    print( f"analyze new/{file} {azim} {elev}" )
                    r = self.funnelCmd( f"analyze new/{file} {azim} {elev}")
                    self.analyze_and_compose = True
                    return
                elif not os.path.exists(jpg_path):
                    print( f"compose new/{file}" )
                    r = self.funnelCmd( f"compose new/{file}" )
                    self.analyze_and_compose = True
                    return

    def backgroundProcess(self):
        #Wait for engine to start up
        while cherrypy.engine.state != cherrypy.engine.states.STARTED:
            time.sleep(1)

        last_time = datetime.time(hour=0,minute=0)
        try:
            f = open("state.json")
            s = f.read()
            data = json.loads(s)
            self.handle_set_state(data)
        except:
            print("state.json does not exist")

        while cherrypy.engine.state == cherrypy.engine.states.STARTED:
            #Check for timed actions
            utc_now = datetime.datetime.now(datetime.timezone.utc).time()
            utc_now = utc_now.replace(second=0, microsecond=0)
            if utc_now != last_time:
                tstart = datetime.time(hour=self.startUTC["h"], minute=self.startUTC["m"])
                tstop = datetime.time(hour=self.stopUTC["h"], minute=self.stopUTC["m"])
                
                if tstart == utc_now and tstart != tstop:
                    self.runStartSequence()
                elif tstop == utc_now and tstart != tstop:
                    self.runStopSequence()
                    self.analyze_and_compose = True
                else:
                    self.checkExposure()

                last_time = utc_now

                if self.archivePath != "none":
                    self.pruneArchive()

            if self.analyze_and_compose:
                self.backgroundAnalyzeAndCompose()

            time.sleep(10)

    def commProcess(self):
        #Wait for engine to start up
        while cherrypy.engine.state != cherrypy.engine.states.STARTED:
            time.sleep(1)

        while cherrypy.engine.state == cherrypy.engine.states.STARTED:
            #Check for commands from web
            try:
                obj = self.cmdQueue.get(timeout=10)
                self.handleSentinelCommand(obj)
            except:
                pass

    def GPS_Process(self):
        while cherrypy.engine.state != cherrypy.engine.states.STARTED:
            time.sleep(1)

        self.gpsProcess = Popen(['python3', './gpsparse.py'])
        time.sleep(1)

        while cherrypy.engine.state == cherrypy.engine.states.STARTED:
            # Create a UDS socket
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)

            # Connect the socket to the port where the server is listening
            server_address = '/tmp/sentinel_time.sock'
            try:
                sock.connect(server_address)
            except:
                return      

            try:
                amount_received = 0
                amount_expected = 52
                line = ""
                
                while amount_received < amount_expected:
                    line += sock.recv(52).decode()
                    amount_received += len(line)
            finally:
                sock.close()          
            
            items = line.split()
            self.gps_latitude = float(items[3])
            self.gps_longitude = float(items[4])
            self.gps_time_offset = float(items[2])
            self.funnelCmd("set_gps_time_offset %7.3f" % self.gps_time_offset)
            print(line)
            time.sleep(60)

    def funnelCmd( self, cmd ):
        responseQueue = Queue()
        self.cmdQueue.put( {"cmd": cmd, "queue": responseQueue})
        try:
            response = responseQueue.get(timeout=1.0)
        except:
            response = {"response": None}
        return response
        
    def connectToSentinel(self,socketName):
        pid = Popen(["./pi-sentinel.bin", "-s", socketName]).pid
        time.sleep(2.0)
        self.sentinelSocket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sentinelSocket.connect(socketName)
        background = Thread(target = self.backgroundProcess)
        background.daemon = True
        background.start()
        comm = Thread(target = self.commProcess)
        comm.daemon = True
        comm.start()
        gps = Thread(target = self.GPS_Process)
        gps.daemon = True
        gps.start()

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
                if os.path.exists( fromPath+"m.jpg"):
                    os.rename( fromPath+"m.jpg", toPath+"m.jpg")
                if os.path.exists( fromPath+".csv"):
                    os.rename( fromPath+".csv", toPath+".csv")

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
            response["startTime"] = self.startTime
            response["stopTime"]  = self.stopTime
            response["startUTC"] = self.startUTC
            response["stopUTC"]  = self.stopUTC
            response["numNew"]     = len(glob.glob("new/*.mp4"))
            response["numSaved"]   = len(glob.glob("saved/*.mp4"))
            response["numTrashed"] = len(glob.glob("trash/*.mp4"))
            response["gpsLatitude"] = self.gps_latitude
            response["gpsLongitude"] = self.gps_longitude
            response["gpsTimeOffset"] = self.gps_time_offset
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
        self.startTime = data["startTime"]
        self.stopTime  = data["stopTime"]
        self.startUTC = data.get("startUTC", self.startTime)
        self.stopUTC = data.get("stopUTC", self.stopTime)
        self.devName = data["devName"]
        self.archivePath = data["archivePath"]

    @cherrypy.expose
    @cherrypy.tools.json_out()
    def get_calibration(self):
        response = {}
        f = open("calibration.json")
        if f:
            s = f.read()
            response = json.loads(s)
        return response

    @cherrypy.expose
    @cherrypy.tools.json_out()
    def get_sky_objects(self):
        response = {}
        f = open("sky_list.json")
        if f:
            s = f.read()
            response = json.loads(s)
        return response

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
        else:
            print("Cannot open f")

        return { "response": "OK" }

    @cherrypy.expose
    @cherrypy.tools.json_in()
    @cherrypy.tools.json_out()
    def save_calibration(self):
        data = cherrypy.request.json

        f = open("calibration.json","w")
        if f:
            s = json.dumps(data,indent=4)
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

        # lines = f.readlines()
        # frames = len(lines)
        # r = self.funnelCmd("set_star_movie_frame_count %d" % frames)
        # if r["response"] != "OK":
        #     return r

        r = self.funnelCmd("average %s" % data["path"])
        return r

    @cherrypy.expose
    @cherrypy.tools.json_in()
    @cherrypy.tools.json_out()
    def analyze(self):
        data = cherrypy.request.json

        path = data["path"]
        azim,elev = self.moonPosition( path )
        cmd = f"analyze {path} {azim} {elev}"
        print( cmd )
        r = self.funnelCmd( f"analyze {path} {azim} {elev}")
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
        utcTime = datetime.datetime(data["year"], data["month"], data["day"],
                               data["hour"], data["minute"], data["second"],
                               tzinfo=datetime.timezone.utc)

        tm = utcTime - datetime.timedelta(seconds=2)
        tp = utcTime + datetime.timedelta(seconds=data["duration"] + 1)

        tm_string = tm.strftime("%Y%m%d_%H%M%S_000")
        tp_string = tp.strftime("%Y%m%d_%H%M%S_000")

        l = []

        while tm < tp+datetime.timedelta(seconds=60):
            path0 = self.archivePath + "/s" + tm.strftime("%Y%m%d_%H") 
            path0 += "/s" + tm.strftime("%Y%m%d_%H%M") + ".h264"
            if path0 not in l and os.path.exists(path0) and os.path.exists(path0.replace(".h264",".txt")):
                l.append(path0)
            tm = tm + datetime.timedelta(seconds=60)

        if len(l) == 0:
            return { "response": "Archive file not found"}

        playbackPath = utcTime.strftime("new/s%Y%m%d_%H%M%S_000.h264")

        pbv = None
        pbt = None
        syncFound = False
        count = 0
        firstFrameTime = 0.0
        lastFrameTime = 0.0
        gpsTimeOffset = 0.0

        for path in l:
            fv = open(path,"rb")
            ft = open(path.replace(".h264",".txt"),"r")

            offset = 0

            while True:
                line = ft.readline()
                if not line:
                    break

                items = line.split()
                ftime = items[0]
                fsize = int(items[1])

                if ftime > tp_string:
                    break

                if ftime < tm_string:
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
                if count == 1:
                    firstFrameTime = ftime

                pbt.write("%3d %s %7d 0\n" % (count,ftime,fsize))

                lastFrameTime = ftime

            fv.close()
            ft.close()

        if pbv == None:
            return { "result": "Time period not in archive file"}

        pbv.close()
        pbt.close()

        framesPerSecond = 30
        if lastFrameTime != firstFrameTime:
            dt_last  = datetime.datetime.strptime(lastFrameTime+"000","%Y%m%d_%H%M%S_%f")
            dt_first = datetime.datetime.strptime(firstFrameTime+"000","%Y%m%d_%H%M%S_%f")
            duration = (dt_last - dt_first).total_seconds()
            framesPerSecond = int(round((count-1)/duration))

        mp4File = playbackPath.replace(".h264",".mp4")
        # pid = Popen(["MP4Box", "-add", playbackPath, "-fps", "%d" % framesPerSecond, "-quiet", "-new", mp4File]).pid
        Popen(["ffmpeg", "-hide_banner", "-loglevel", "error", "-r", f"{framesPerSecond}", "-i", f"{playbackPath}", "-vcodec", 
                "copy", mp4File])
        return { "response": "OK"}

    @cherrypy.expose
    @cherrypy.tools.json_in()
    @cherrypy.tools.json_out()
    def recalc_start_stop_times(self):
        data = cherrypy.request.json
        
        # Create an observer object
        observer = ephem.Observer()

        # Set the observer's location (latitude and longitude)
        observer.lat = str(data["lat"])
        observer.lon = str(data["lon"])

        # Set the date to the current UTC time
        observer.date = ephem.now()

        # Create a Sun object
        sun = ephem.Sun()

        twilight = data["twilight"]

        sunset = observer.next_setting(sun)
        sunset_datetime = sunset.datetime()
        time_on = sunset_datetime + datetime.timedelta(minutes=twilight)
        time_on = round_to_nearest_5_minutes( time_on, False )

        sunrise = observer.next_rising(sun)
        sunrise_datetime = sunrise.datetime()
        time_off = sunrise_datetime - datetime.timedelta(minutes=twilight)
        time_off = round_to_nearest_5_minutes( time_off, True )

        self.startUTC = { "h": time_on.hour, "m": time_on.minute }
        self.stopUTC = { "h": time_off.hour, "m": time_off.minute }

        return { "response": "OK", "startUTC": self.startUTC, "stopUTC": self.stopUTC }

    @cherrypy.expose
    @cherrypy.tools.json_in()
    @cherrypy.tools.json_out()
    def stars(self):
        starList = [ "Sirius", "Canopus", "Rigil Kentaurus", "Arcturus", "Vega", "Capella", "Rigel", "Procyon",
                     "Achernar", "Betelgeuse", "Hadar", "Altair", "Acrux", "Aldebaran", "Antares", "Spica", "Pollux", "Fomalhaut",
                     "Deneb", "Mimosa", "Regulus", "Adhara", "Shaula", "Castor", "Gacrux" ]
        planetList = ["Mars", "Venus", "Jupiter", "Saturn", "Moon"]

        data = cherrypy.request.json
        observer = ephem.Observer()
        observer.lon = str(data["cameraLongitude"])
        observer.lat = str(data["cameraLatitude"])
        observer.elevation = data["cameraElevation"]

        if not data["path"]:
            return { "response": "OK", "sky_objects": [] }

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
                azim = star.az*180.0/pi
                elev = star.alt*180.0/pi
                result.append( {"name": name, "azim": azim, "elev": elev} )

        mars = ephem.Mars(date)
        mars.compute(observer)
        if mars.alt > 0.0:
            azim = mars.az*180.0/pi
            elev = mars.alt*180.0/pi
            result.append( {"name": "Mars", "azim": azim, "elev": elev} )

        venus = ephem.Venus(date)
        venus.compute(observer)
        if venus.alt > 0.0:
            azim = venus.az*180.0/pi
            elev = venus.alt*180.0/pi
            result.append( {"name": "Venus", "azim": azim, "elev": elev} )

        jupiter = ephem.Jupiter(date)
        jupiter.compute(observer)
        if jupiter.alt > 0.0:
            azim = jupiter.az*180.0/pi
            elev = jupiter.alt*180.0/pi
            result.append( {"name": "Jupiter", "azim": azim, "elev": elev} )

        saturn = ephem.Saturn(date)
        saturn.compute(observer)
        if saturn.alt > 0.0:
            azim = saturn.az*180.0/pi
            elev = saturn.alt*180.0/pi
            result.append( {"name": "Saturn", "azim": azim, "elev": elev} )

        moon = ephem.Moon(date)
        moon.compute(observer)
        if moon.alt > 0.0:
            azim = moon.az*180.0/pi
            elev = moon.alt*180.0/pi
            result.append( {"name": "Moon", "azim": azim, "elev": elev} )

        return { "response": "OK", "sky_objects": result }

    @cherrypy.expose
    @cherrypy.tools.json_in()
    @cherrypy.tools.json_out()
    def save_sky_objects(self):
        print( "save_sky_objects" )
        data = cherrypy.request.json
        sky_object_list = data["sky_object_list"]
        f = open("sky_list.json","w")
        if f:
            s = json.dumps(sky_object_list,indent=4)
            f.write(s)
            f.close()

        return { "response": "OK" }

    @cherrypy.expose
    @cherrypy.tools.json_in()
    @cherrypy.tools.json_out()
    def do_calibration(self):
        data = cherrypy.request.json
        sky_object_list = data["sky_object_list"]
        calibration_state = data["calibration_state"]
        CalibrationToRadians( calibration_state )

        calVector = CalibrationVector(calibration_state)

        result = minimize(fun=TotalCalibrationError, x0=calVector, method="Nelder-Mead", args=(sky_object_list,), options={'maxiter': 10000, 'disp': False} )
        print("Iterations: %f Error: %f" % (result.nfev, result.fun) )

        calObject = CalibrationObject(result["x"])
        CalibrationToDegrees( calObject )
        CalibrationTruncate( calObject )

        calObject["cameraLatitude"]  = calibration_state["cameraLatitude"]
        calObject["cameraLongitude"] = calibration_state["cameraLongitude"]
        calObject["cameraElevation"] = calibration_state["cameraElevation"]

        f = open("calibration.json","w")
        if f:
            s = json.dumps(calObject,indent=4)
            f.write(s)
            f.close()

        f = open("sky_list.json","w")
        if f:
            s = json.dumps(sky_object_list,indent=4)
            f.write(s)
            f.close()

        return { "response": "OK", "calibration_state": calObject }
        

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
    import sys
    port = 9090
    socketName = "/tmp/sentinel.sock"

    server = SentinelServer()
    if len(sys.argv) > 1 and argv[1] in ["1","2","3","4"]:
        port += int(argv[1])
        socketName += argv[1]

    server.connectToSentinel(socketName)
    cherrypy.config.update({'server.socket_host': '0.0.0.0', 'server.socket_port': port})
    cherrypy.quickstart(server, '/', conf)
    # server.sentinelSocket.send("quit\n".encode())
    print("pi-sentinel end")
