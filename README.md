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

The Pi Sentinel software is currently being used with the following hardware:

* Raspberry Pi 4 Model B 2GB with 32GB SD card
* USB Camera - SVPRO Mini USB Webcam 1920 1080P IMX322 180 Degree Wide Angle Digital Low Illumination 0.01 lux Industrial USB Camera 
HD 2mp for Automatic Vending Machines
* USB Cable - C2G 38988 USB Active Extension Cable - USB 2.0 A Male to A Female Cable, Center Booster Format, 
Black (25 Feet, 7.62 Meters)
* WAVLINK USB 3.0 to SATA External Hard Drive Docking Station with 2USB 3.0 HUB and TF & SD Card for 2.5 inch/3.5 Inch HDD
* Power Supply - CanaKit USB-C Power Supply

The camera has a fisheye lens but the full sky image is somewhat cropped at the top and bottom. 

I wanted to have the camera far away from the Raspberry Pi, but since USB cables are not designed to work over long distances, I am using an Active Extension Cable that has built in circuitry to boost the signal.

I am using a Hard Drive Docking station that lets me attach a 1TB SATA hard drive for archival storage, and also provides a powered USB hub that connects to and provides power to the camera.

## User Interface

The Pi-Sentinel software provides a web interface which allows the user to control the software and view events with just a web browser.  I am currently using Firefox on a Mac, which is on the same WiFi network as the Raspberry Pi.

The interface consists of an image/video panel on the right and a control panel on the left.  The image panel is where event videos and images are shown.  The control panel lets you control the Pi Sentinel software and to examine events produced by Pi Sentinel.  The control panel is made up of three panes that you can view one at a time.  The first pane (Control) lets you control various aspects of the software and to view some status information.  The second pane (Events) lets you see all the events captured by Pi Sentinel and to select each event for examination.  The third pane (Playback) lets you produce an event using the video archive (if present) by specifying a time and duration.  

### Control Pane

![Control Panel 1](doc/PiSentinelControls1.png)

At the top of the pane is the **Actions** pop-down button, followed by three selection buttons: **Controls**, **Events**, and **Playback**.  This set of four buttons exists at the top of all three panes.  In fact, you use the selection buttons to select which pane is shown below.

Following is the **Request Update** button.  Pressing this sends a request to the Pi Sentinel software to update the table which follows with the current status and commanded state.  This table has the following items:

* **Running** - This indicates whether or not the Pi Sentinel software is processing camera data looking for changes that might be caused by meteors.
* **New, Saved, Trash** - This provides three numbers that indicate how many events are in the *new* directory, the *saved* directory, and the *trash* directory.  All new events are automatically placed in the *new* directory and it is up to the user to move them to one of the other directories.
* **Frame Rate** - If Pi Sentinel is processing camera data, this indicates the frame rate in frames pers second that the camera is generating.  The frame rate can change if the light level changes or if Pi Sentinel controls the rate itself.
* **Zenith Amplitude** - If the Pi Sentinel is processing camera data, this is an estimate of the brightness at the zenith, and is produced by sampling a set of pixels near the center of the image.  This estimate is sometimes used by Pi Sentinel to control exposure time.
* **Auto Start Time** - This is the local time at which Pi Sentinel will start processing camera data.  Typically, this is when you might expect to start seeing meteors.
* **Auto Stop Time** - This is the local time at which Pi Sentinel will stop processing camera data.
* **Noise Threshold** - Pixels that less than this number above background are ignored.
* **Trigger Threshold** - If the sum of pixel values that are more than the noise threshold above background exceeds this number, then a trigger is initiated and an event is produced.
* **Max Events Per Hour** - The nominal maximum number of events that can be produced per hour.  The way this works can be explained with an example.  Suppose that the Max Events Per Hour is set to 5.  When processing begins, Pi Sentinel has a starting credit of 5 events.  Every time an event occurs, the credit is reduced by 1.  If the credit reaches 0, no more events are allowed.  However, one credit is added every 12 minutes (1/5 hour) up to a maximum of 5 credits.
* **Camera Device** - This specifies which video device is being used by the Raspberry Pi to connect to the camera.  This will typically be `/dev/videoN` where N is some number.  Cameras often connect to several devices if they provide several different data formats.  For Pi Sentinel you need to connect to the device that provides H264 formatted data.
* **Archive Path** - This specifies the path to the directory that will contain the archive, which is all the video that the camera produces while Pi Sentinel is processing data.  If you do not wish to archive all the video, the path should be specified as: `none`.


## Mask File

The mask file is used to specify areas of the image that should be ignored.  The mask file contains an image of the camera field of view in which the areas to be ignored are painted red.  The image should be scaled to 640 pixels wide and 360 pixels high and be saved as `mask.ppm` (PPM format) in the same directory as `sentinel.py`.  

To produce this file, I started with an event video (either naturally occurring or forced) and created a composite JPEG image file.  I then opened the JPEG file with Gimp, scaled it to 640 x 360 pixels, selected the Pencil tool, widened the tool to a convenient size, selected solid red as the foreground color, and then painted the areas of the image red that were not open sky.  I then did an Export As... and saved the result as `mask.ppm`.  Gimp can be run on the Raspberry Pi or on any other computer.

The mask file is read by the sentinel program every time it starts.
