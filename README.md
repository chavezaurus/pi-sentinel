# Pi Sentinel

All-sky camera meteor detection and display software for the Raspberry Pi.

## Software Dependencies

The Pi Sentinel software required the following external programs and libraries:

```
sudo apt-get install build-essentials
sudo apt-get install gpac
sudo apt-get install v4l-utils
sudo pip3 install cherrypy
```

The `build-essentials` install provides the compiler and make system used to compile the C source code.

The `gpac` install provides the utility program (MP4Box) used to convert .h264 formatted files to .mp4.

The `v4l-utils` install provides the utility function (v4l2-ctl) used to configure the camera.

The `cherrypy` install provides the Python web server used for the graphical interface.

## Hardware 

The Pi Sentinel software was developed using the following hardware:

* Raspberry Pi 3 Model B
* USB Camera - SVPRO Mini USB Webcam 1920 1080P IMX322 180 Degree Wide Angle Digital Low Illumination 0.01 lux Industrial USB Camera 
HD 2mp for Automatic Vending Machines
* USB Cable - C2G 38988 USB Active Extension Cable - USB 2.0 A Male to A Female Cable, Center Booster Format, 
Black (25 Feet, 7.62 Meters)
* Power Supply - Raspberry Pi 3 Model B/B+ Plus Power Supply 5V 3A with Switch UL Certified Compatible w/ 2.5A 2A 1.5A 1A 
Fast Rapid Charge AC Adapter w/ 1.5m Extra Long On Off Power Switch Micro USB Cable
