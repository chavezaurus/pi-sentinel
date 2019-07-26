# Pi Sentinel

All-sky camera meteor detection and display software for the Raspberry Pi.

## Software Dependencies

The Pi Sentinel software required the following external programs and libraries:

```
sudo apt-get install git
sudo apt-get install gpac
sudo apt-get install python3-pip
sudo pip3 install cherrypy
```

The `git` install allows us to access the github repository.

The `gpac` install provides the utility program (MP4Box) used to convert .h264 formatted files to .mp4.

The `python3-pip` install gives us pip3, which is needed for the next install.

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

## Mask File

The mask file is used to specify areas of the image that should be ignored.  The mask file contains an image of the camera field of view in which the areas to be ignored are painted red.  The image should be scaled to 640 pixels wide and 360 pixels high and be saved as `mask.ppm` (PPM format) in the same directory as `sentinel.py`.  

To produce this file, I started with an event video (either naturally occurring or forced) and created a composite JPEG image file.  I then opened the JPEG file with Gimp, scaled it to 640 x 360 pixels, selected the Pencil tool, widened the tool to a convenient size, selected solid red as the foreground color, and then painted the areas of the image red that were not open sky.  I then did an Export As... and saved the result as `mask.ppm`.  Gimp can be run on the Raspberry Pi or on any other computer.

The mask file is read by the sentinel program every time it starts.
