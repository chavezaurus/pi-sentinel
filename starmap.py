import datetime
import requests
import pytz
import json
import os.path
import time

def process():
    mountain = pytz.timezone("US/Mountain")
    utc = pytz.timezone("UTC")
    start = mountain.localize( datetime.datetime(2019,8,26,22,0,0))
    end   = mountain.localize( datetime.datetime(2019,8,27, 5,0,0))
    headers = {'Content-type': 'application/json'}
    address = "http://localhost:9090"

    while start <= end:
        url = address + "/playback"
        obj = { "year": start.year, "month": start.month, "day": start.day,
                "hour": start.hour, "minute": start.minute, "second": start.second,
                "duration": 300 }

        r = requests.post(url, json=obj, headers=headers)
        j = r.json()
        if j["response"] != "OK":
            print(j)
            break;

        utime = start.astimezone(utc)
        path = utime.strftime("new/s%Y%m%d_%H%M%S.mp4")
        print(path)

        url = address + "/average"
        obj = {"path": path.replace(".mp4",".h264")}

        r = requests.post(url, json=obj, headers=headers)
        j = r.json()
        if j["response"] != "OK":
            print(j)
            break;

        while not os.path.exists(path.replace(".mp4",".jpg")):
            time.sleep(5)

        start = start + datetime.timedelta(minutes=60)


if __name__ == "__main__":
    process()