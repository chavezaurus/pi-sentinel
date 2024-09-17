from datetime import datetime, timedelta
from requests import post
from pytz import timezone
from os.path import exists
from time import sleep
from os import remove

def process():
    mountain = timezone("US/Mountain")
    utc = timezone("UTC")
    start = mountain.localize( datetime(2024,8,27,22,0,0))
    end   = start + timedelta(hours=4)
    delta = timedelta(minutes=60)

    headers = {'Content-type': 'application/json'}
    address = "http://localhost:9090"
    url1 = address + "/playback"
    url2 = address + "/average"

    while start <= end:
        obj1 = { "year": start.year, "month": start.month, "day": start.day,
                 "hour": start.hour, "minute": start.minute, "second": start.second,
                 "duration": 300 }

        # Make long duration playback
        r = post(url1, json=obj1, headers=headers)
        j = r.json()
        if j["response"] != "OK":
            print(j)
            break;

        utime = start.astimezone(utc)
        path = utime.strftime("new/s%Y%m%d_%H%M%S_000.mp4")
        if exists(path):
            remove(path)
        print(path)

        obj2 = {"path": path.replace(".mp4",".h264")}

        # Make star chart from long duration playback
        r = post(url2, json=obj2, headers=headers)
        print(r)
        j = r.json()
        if j["response"] != "OK":
            print(j)
            break;
        
        jpath = path.replace(".mp4",".jpg")
        while not exists(jpath):
            sleep(5)

        # Replace 5 minute playback with 10 second playback to save space
        obj1["duration"] = 10
        r = post(url1, json=obj1, headers=headers)
        j = r.json()
        if j["response"] != "OK":
            print(j)
            break;

        start = start + delta


if __name__ == "__main__":
    process()