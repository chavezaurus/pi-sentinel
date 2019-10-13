#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <mcheck.h>

#include <getopt.h> /* getopt_long() */

#include <fcntl.h> /* low-level i/o */
#include <unistd.h>
#include <pthread.h>
#include <errno.h>
#include <math.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <libgen.h>

#include <linux/videodev2.h>
#include "bcm_host.h"
#include "IL/OMX_Broadcom.h"

#define IMAGE_WIDTH 1920	// JPEG image width
#define IMAGE_HEIGHT 1080	// JPEG image height

#define CLEAR(x) memset(&(x), 0, sizeof(x))
#define FRAMES_TO_KEEP 300

#ifndef V4L2_PIX_FMT_H264
#define V4L2_PIX_FMT_H264 v4l2_fourcc('H', '2', '6', '4') /* H264 with start codes */
#endif

#define OMX_INIT_STRUCTURE(a) \
    memset(&(a), 0, sizeof(a)); \
    (a).nSize = sizeof(a); \
    (a).nVersion.nVersion = OMX_VERSION; \
    (a).nVersion.s.nVersionMajor = OMX_VERSION_MAJOR; \
    (a).nVersion.s.nVersionMinor = OMX_VERSION_MINOR; \
    (a).nVersion.s.nRevision = OMX_VERSION_REVISION; \
    (a).nVersion.s.nStep = OMX_VERSION_STEP

struct buffer
{
    void *start;
    size_t length;
};

static OMX_HANDLETYPE decoderHandle;
static OMX_CALLBACKTYPE decoderCallbacks;
static OMX_HANDLETYPE encoderHandle;
static OMX_CALLBACKTYPE encoderCallbacks;
static OMX_BUFFERHEADERTYPE *decoderBuffer;
static OMX_BUFFERHEADERTYPE *pingBuffer;
static OMX_BUFFERHEADERTYPE *pongBuffer;
static OMX_BUFFERHEADERTYPE *outputBuffer;
static OMX_BUFFERHEADERTYPE *encoderBuffer;
static OMX_BUFFERHEADERTYPE *imageBuffer;
static OMX_BUFFERHEADERTYPE *jpegBuffers[3];

static char *composeBuffer;
static int  *averageBuffer;
static char dev_name[20];
static char archive_path[255];
static char *h264_name;
static int fd = -1;
struct buffer buffers[10];
static unsigned int n_buffers;
static int out_buf;
static int decodeBufferFull;
static int portSettingsChanged = 0;
static int frame_count = 300;
static int frame_number = 0;
static int force_count = -1;
static int noise_level = 40;
static unsigned char *testFrame = 0;
static unsigned char *referenceFrame = 0;
static unsigned char *maskFrame = 0;

static unsigned char *frameBuffers[FRAMES_TO_KEEP];
static int frameAlloc[FRAMES_TO_KEEP];
static int frameSizes[FRAMES_TO_KEEP];
static int frameTimestampLo[FRAMES_TO_KEEP];
static int frameTimestampHi[FRAMES_TO_KEEP];
static int nextFrameIndex;
static int archiveFrameIndex;
static int eventFrameIndex;
static int eventFrameStop;

static int sumBuffers[FRAMES_TO_KEEP];
static int xSumBuffers[FRAMES_TO_KEEP];
static int ySumBuffers[FRAMES_TO_KEEP];
static int sumCounts[FRAMES_TO_KEEP];
static int sumTimestamps[FRAMES_TO_KEEP];
static int nextSumIndex;
static int eventSumIndex;

static int triggered;
static int untriggered;
static int triggerCounter;
static int untriggerCounter;
static int sumThreshold = 175;
static int starMovieFrameCount = 0;

static int eventDuration;
static unsigned int eventTimeStamp_hi;
static unsigned int eventTimeStamp_lo;
static int rateLimitLastSecond;
static int rateLimitBank;
static int rateLimitEventsPerHour = 5;

static double frameRate = 30.0;
static double zenithAmplitude = 0.0;

static FILE *videoFile;
static FILE *textFile;
static FILE *logfile = 0;
static FILE *videoArchive = 0;
static FILE *textArchive = 0;

static pthread_mutex_t saveEventThreadMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t saveEventThreadCondition = PTHREAD_COND_INITIALIZER;
static pthread_t saveEventThread_id;

static pthread_mutex_t decodeBufferThreadMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t decodeBufferThreadCondition = PTHREAD_COND_INITIALIZER;
static pthread_t decodeBufferThread_id;

static pthread_mutex_t composeBufferThreadMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t composeBufferThreadCondition = PTHREAD_COND_INITIALIZER;
static pthread_t composeThread_id;
static pthread_t averageThread_id;
static pthread_t analyzeThread_id;
static pthread_t maskThread_id;
static pthread_t archiveThread_id;

static pthread_t runThread_id;

static int thread_exit;
static int running = 0;
static int stop_flag = 0;

#define ALIGN(x, y) (((x) + ((y)-1)) & ~((y)-1))

struct ThreadArguments
{
    unsigned int eventStartTimeHi;
    unsigned int eventStartTimeLo;
    unsigned int eventStopTimeHi;
    unsigned int eventStopTimeLo;
    int jobPending;
} threadingJob;

struct CalibrationStruct
{
    double V;
    double S;
    double D;
    double a0;
    double E;
    double eps;
    double COPx;
    double COPy;
    double alpha;
    double flat;

    double c;
    double d;
    double e;
    double f;
    double g;
    double h;

    double px;
    double py;
    double azim;
    double elev;
} calibrationParameters;

static void errno_exit(const char *s)
{
    fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
    exit(EXIT_FAILURE);
}

static int xioctl(int fh, int request, void *arg)
{
    int r;

    do
    {
        r = ioctl(fh, request, arg);
    } while (-1 == r && EINTR == errno);

    return r;
}

static void save_image(const void *p, int size)
{
    if (out_buf == 0)
    {
        /* write to file */
        FILE *fp = fopen("image.jpeg", "wb");
        fwrite(p, size, 1, fp);
        fflush(fp);
        fclose(fp);
    }
    else
    {
        perror("write");
    }
}

static double getEpochTimeShift()
{
    struct timeval epochtime;
    struct timespec vsTime;

    gettimeofday(&epochtime, NULL);
    clock_gettime(CLOCK_MONOTONIC, &vsTime);

    double uptime = vsTime.tv_sec + vsTime.tv_nsec / 1000000000.0;
    double epoch = epochtime.tv_sec + epochtime.tv_usec / 1000000.0;
    return epoch - uptime;
}

static int IsIDR(const unsigned char *p, int size)
{
    unsigned int word = 0xffffffff;
    int i;

    for (i = 0; i < size; ++i)
    {
        unsigned int c = p[i];
        if (word == 1)
        {
            c &= 0x0f;

            if (c == 1)
                return 0;

            if (c == 5)
                return 1;
        }

        word = (word << 8) | c;
    }

    return 0;
}

static int read_frame(void)
{
    struct v4l2_buffer buf;
    unsigned int i;

    CLEAR(buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf))
    {
        switch (errno)
        {
        case EAGAIN:
            return 0;

        case EIO:
            /* Could ignore EIO, see spec. */

            /* fall through */

        default:
            errno_exit("VIDIOC_DQBUF");
        }
    }

    assert(buf.index < n_buffers);

    save_image(buffers[buf.index].start, buf.bytesused);
    if ( logfile )
        fprintf(logfile, "Time(s): %d.%03d\n", buf.timestamp.tv_sec, buf.timestamp.tv_usec / 1000);

    if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
        errno_exit("VIDIOC_QBUF");

    return 1;
}

static void mainloop(void)
{
    unsigned int count;
    unsigned int loopIsInfinite = 0;

    if (frame_count == 0)
        loopIsInfinite = 1; //infinite loop
    count = frame_count;

    while ((count-- > 0) || loopIsInfinite)
    {
        for (;;)
        {
            fd_set fds;
            struct timeval tv;
            int r;

            FD_ZERO(&fds);
            FD_SET(fd, &fds);

            /* Timeout. */
            tv.tv_sec = 2;
            tv.tv_usec = 0;

            r = select(fd + 1, &fds, NULL, NULL, &tv);

            if (-1 == r)
            {
                if (EINTR == errno)
                    continue;
                errno_exit("select");
            }

            if (0 == r)
            {
                fprintf(stderr, "select timeout\n");
                exit(EXIT_FAILURE);
            }

            if (read_frame())
                break;
            /* EAGAIN - continue select loop. */
        }
    }
}

static void stop_capturing(void)
{
    enum v4l2_buf_type type;

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type))
        errno_exit("VIDIOC_STREAMOFF");
}

static void start_capturing(void)
{
    unsigned int i;
    enum v4l2_buf_type type;

    for (i = 0; i < n_buffers; ++i)
    {
        struct v4l2_buffer buf;

        CLEAR(buf);
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
            errno_exit("VIDIOC_QBUF");
    }

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
        errno_exit("VIDIOC_STREAMON");
}

static void uninit_device(void)
{
    unsigned int i;

    for (i = 0; i < n_buffers; ++i)
        if (-1 == munmap(buffers[i].start, buffers[i].length))
            errno_exit("munmap");

    // fprintf(stderr, "Free buffers\n");
    // free(buffers);
}

static void init_mmap(void)
{
    struct v4l2_requestbuffers req;

    CLEAR(req);

    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req))
    {
        if (EINVAL == errno)
        {
            fprintf(stderr, "%s does not support "
                            "memory mapping\n",
                    dev_name);
            exit(EXIT_FAILURE);
        }
        else
        {
            errno_exit("VIDIOC_REQBUFS");
        }
    }

    if (req.count < 2)
    {
        fprintf(stderr, "Insufficient buffer memory on %s\n",
                dev_name);
        exit(EXIT_FAILURE);
    }

    // buffers = calloc(req.count, sizeof(*buffers));

    // if (!buffers)
    //{
    //    fprintf(stderr, "Out of memory\n");
    //    exit(EXIT_FAILURE);
    //}

    for (n_buffers = 0; n_buffers < req.count; ++n_buffers)
    {
        struct v4l2_buffer buf;

        CLEAR(buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = n_buffers;

        if (-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
            errno_exit("VIDIOC_QUERYBUF");

        buffers[n_buffers].length = buf.length;
        buffers[n_buffers].start =
            mmap(NULL /* start anywhere */,
                 buf.length,
                 PROT_READ | PROT_WRITE /* required */,
                 MAP_SHARED /* recommended */,
                 fd, buf.m.offset);

        if (MAP_FAILED == buffers[n_buffers].start)
            errno_exit("mmap");
    }
}

static void init_device(int jpeg)
{
    struct v4l2_capability cap;
    struct v4l2_cropcap cropcap;
    struct v4l2_crop crop;
    struct v4l2_format fmt;
    struct v4l2_control control;
    struct v4l2_queryctrl queryctrl;

    unsigned int min;

    if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap))
    {
        if (EINVAL == errno)
        {
            fprintf(stderr, "%s is no V4L2 device\n",
                    dev_name);
            exit(EXIT_FAILURE);
        }
        else
        {
            errno_exit("VIDIOC_QUERYCAP");
        }
    }

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
    {
        fprintf(stderr, "%s is no video capture device\n",
                dev_name);
        exit(EXIT_FAILURE);
    }

    if (!(cap.capabilities & V4L2_CAP_STREAMING))
    {
        fprintf(stderr, "%s does not support streaming i/o\n",
                dev_name);
        exit(EXIT_FAILURE);
    }

    /* Select video input, video standard and tune here. */

    CLEAR(cropcap);

    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (0 == xioctl(fd, VIDIOC_CROPCAP, &cropcap))
    {
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        crop.c = cropcap.defrect; /* reset to default */

        if (-1 == xioctl(fd, VIDIOC_S_CROP, &crop))
        {
            switch (errno)
            {
            case EINVAL:
                /* Cropping not supported. */
                break;
            default:
                /* Errors ignored. */
                break;
            }
        }
    }
    else
    {
        /* Errors ignored. */
    }

    CLEAR(fmt);

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (-1 == xioctl(fd, VIDIOC_G_FMT, &fmt))
        errno_exit("VIDIOC_G_FMT");

    if ( fmt.fmt.pix.width != 1920 ||
         fmt.fmt.pix.height != 1080 ||
         fmt.fmt.pix.pixelformat != jpeg ? V4L2_PIX_FMT_MJPEG : V4L2_PIX_FMT_H264 )
    {
        fmt.fmt.pix.width = 1920;
        fmt.fmt.pix.height = 1080;
        fmt.fmt.pix.pixelformat = jpeg ? V4L2_PIX_FMT_MJPEG : V4L2_PIX_FMT_H264;
        fmt.fmt.pix.field = V4L2_FIELD_NONE;

        if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
            errno_exit("VIDIOC_S_FMT");
    }

    init_mmap();
}

static void close_device(void)
{
    if (-1 == close(fd))
        errno_exit("close");

    fd = -1;
    fprintf(stderr, "Close device\n");
}

static void open_device(void)
{
    struct stat st;

    if (-1 == stat(dev_name, &st))
    {
        fprintf(stderr, "Cannot identify '%s': %d, %s\n",
                dev_name, errno, strerror(errno));
        exit(EXIT_FAILURE);
    }

    if (!S_ISCHR(st.st_mode))
    {
        fprintf(stderr, "%s is no device\n", dev_name);
        exit(EXIT_FAILURE);
    }

    fd = open(dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);

    if (-1 == fd)
    {
        fprintf(stderr, "Cannot open '%s': %d, %s\n",
                dev_name, errno, strerror(errno));
        exit(EXIT_FAILURE);
    }
}

static OMX_ERRORTYPE
EventHandler(OMX_OUT OMX_HANDLETYPE hComponent,
             OMX_OUT OMX_PTR pAppData,
             OMX_OUT OMX_EVENTTYPE eEvent,
             OMX_OUT OMX_U32 Data1,
             OMX_OUT OMX_U32 Data2,
             OMX_OUT OMX_PTR pEventData)
{
    if ( eEvent == OMX_EventPortSettingsChanged )
    {
        fprintf(stderr, "Port Settings Changed\n");
        portSettingsChanged = 1;
    } 
    else 
    {
        fprintf(stderr, "EventHandler %d %x\n", eEvent, Data1);
    }
    return OMX_ErrorNone;
}

static void keepFrames(const unsigned char *p, int size, unsigned int timeStamp_hi, unsigned int timeStamp_lo)
{
    static int alloc_size = 8192;

    while ( size > alloc_size )
        alloc_size += 8192;

    if ( frameAlloc[nextFrameIndex] < size )
    {
        free(frameBuffers[nextFrameIndex]);
        frameBuffers[nextFrameIndex] = (unsigned char *)malloc(alloc_size);
        frameAlloc[nextFrameIndex] = alloc_size;
    }

    frameSizes[nextFrameIndex] = size;
    frameTimestampLo[nextFrameIndex] = timeStamp_lo;
    frameTimestampHi[nextFrameIndex] = timeStamp_hi;

    memcpy(frameBuffers[nextFrameIndex], p, size);

    nextFrameIndex = (nextFrameIndex + 1) % FRAMES_TO_KEEP;
}

static int archiveFrame( void )
{
    static unsigned int oldMinute = 0;
    static double epochTimeShift = 0;

    const unsigned char* p = frameBuffers[archiveFrameIndex];
    int size = frameSizes[archiveFrameIndex];
    unsigned int timeStamp_hi = frameTimestampHi[archiveFrameIndex];
    unsigned int timeStamp_lo = frameTimestampLo[archiveFrameIndex];

    char videoPath[255];
    char textPath[255];
    char folderPath[255];

    if ( videoArchive == 0 )
        epochTimeShift = getEpochTimeShift();

    double timelong = timeStamp_hi * 4294.967296 + 1.0e-6 * timeStamp_lo + epochTimeShift;
    int minute = timelong / 60.0;

    if ( minute != oldMinute )
    {
        if ( videoArchive != 0 )
            fclose( videoArchive );
        if ( textArchive != 0 )
            fclose( textArchive );

        videoArchive = 0;
        textArchive = 0;

        oldMinute = minute;
    }

    if ( videoArchive == 0 )
    {
        int hour = minute / 60;

        sprintf( folderPath, "%s/s%d", archive_path, hour );
        int e = mkdir(folderPath, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

        if ( e == -1 && errno != EEXIST )
            return -1;

        sprintf( videoPath, "%s/s%d.h264", folderPath, minute );
        sprintf( textPath,  "%s/s%d.txt",  folderPath, minute );

        videoArchive = fopen( videoPath, "wb" );
        if ( videoArchive == 0 )
            return -1;

        textArchive = fopen( textPath, "w" );
        if ( textArchive == 0 )
            return -1;

        epochTimeShift = getEpochTimeShift();
    }

    int count = fwrite( p, size, 1, videoArchive );
    if ( count != 1 )
        return -1;

    count = fprintf( textArchive, "%12.3f %6d\n", timelong, size );
    if ( count == 0 )
        return -1;

    return 0;
}

static void *archiveThread(void *argin)
{
    if ( !strcmp(archive_path, "none" ) )
        return 0;

    int archiving = 1;

    while (archiving)
    {
        while ( archiveFrameIndex != nextFrameIndex )
        {
            int test = archiveFrame();
            if ( test != 0 )
            {
                fprintf( stderr, "Archive aborted" );
                archiving = 0;
                break;
            }

            archiveFrameIndex = (archiveFrameIndex+1) % FRAMES_TO_KEEP;
        }

        usleep(30000);

        if ( thread_exit )
            archiving = 0;
    }

    if ( videoArchive )
        fclose( videoArchive );

    if ( textArchive )
        fclose( textArchive );

    videoArchive = 0;
    textArchive = 0;
}

static int decode_frame(void)
{
    static int count = 0;

    struct timespec spec;
    int ms;   // Milliseconds
    time_t s; // Seconds

    OMX_ERRORTYPE error;

    struct v4l2_buffer buf;
    unsigned int i;

    CLEAR(buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf))
    {
        switch (errno)
        {
        case EAGAIN:
            return 0;

        case EIO:
            /* Could ignore EIO, see spec. */

            /* fall through */

        default:
            errno_exit("VIDIOC_DQBUF");
        }
    }

    double timeStampSec = buf.timestamp.tv_sec + 1.0e-6 * buf.timestamp.tv_usec;
    unsigned int timeStamp_hi = timeStampSec / 4294.967296;
    unsigned int timeStamp_lo = fmod(timeStampSec, 4294.967296) * 1.0e6;

    keepFrames(buffers[buf.index].start, buf.bytesused, timeStamp_hi, timeStamp_lo);

    if (buf.bytesused > decoderBuffer->nAllocLen)
    {
        fprintf(stderr, "Input frame too big: %d\n", buf.bytesused);
        exit(EXIT_FAILURE);
    }

    assert(buf.index < n_buffers);

    pthread_mutex_lock(&composeBufferThreadMutex);
    while (decodeBufferFull != 0)
        pthread_cond_wait(&composeBufferThreadCondition, &composeBufferThreadMutex);

    decodeBufferFull = 1;
    pthread_mutex_unlock(&composeBufferThreadMutex);

    memcpy(decoderBuffer->pBuffer, buffers[buf.index].start, buf.bytesused);
    decoderBuffer->nFilledLen = buf.bytesused;
    decoderBuffer->nOffset = 0;
    decoderBuffer->nInputPortIndex = 130;
    decoderBuffer->nTimeStamp.nLowPart = timeStamp_lo;
    decoderBuffer->nTimeStamp.nHighPart = timeStamp_hi;

    // fprintf( logfile, "->--- %u\n", timeStamp_lo );
    error = OMX_EmptyThisBuffer(decoderHandle, decoderBuffer);

    if (error != OMX_ErrorNone)
    {
        fprintf(stderr, "Cannot EmptyThisBuffer: %d, %s\n",
                errno, strerror(errno));

        if (logfile != 0)
        {
            fflush(logfile);
            fclose(logfile);
        }

        exit(EXIT_FAILURE);
    }

    if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
        errno_exit("VIDIOC_QBUF");

    return 1;
}

static void saveEventJob(void)
{
    struct ThreadArguments arg = threadingJob;

    struct tm *ptm;
    char nameVideoFile[100];
    char nameTextFile[100];
    char nameSystemCmd[100];
    char nameMp4File[100];

    int frameIndex = -1;
    int sumIndex = -1;
    int stopIndex = -1;

    for (int i = 0; i < FRAMES_TO_KEEP; ++i)
    {
        if (sumTimestamps[i] == arg.eventStartTimeLo)
            sumIndex = i;
        if (frameTimestampLo[i] == arg.eventStartTimeLo)
            frameIndex = i;
        if (sumTimestamps[i] == arg.eventStopTimeLo)
            stopIndex = i;
    }

    if (frameIndex == -1 || sumIndex == -1 || stopIndex == -1)
        return;

    double timelong = arg.eventStartTimeHi * 4294.967296 + 1.0e-6 * arg.eventStartTimeLo + getEpochTimeShift();
    time_t unixTime = timelong;
    int milli = 1000 * (timelong-unixTime);

    ptm = gmtime(&unixTime);
    sprintf(nameVideoFile, "new/s%04d%02d%02d_%02d%02d%02d_%03d.h264",
            ptm->tm_year + 1900, ptm->tm_mon + 1, ptm->tm_mday,
            ptm->tm_hour, ptm->tm_min, ptm->tm_sec, milli);

    FILE *videoFile = fopen(nameVideoFile, "w");
    if (videoFile == 0)
        return;

    sprintf(nameTextFile, "new/s%04d%02d%02d_%02d%02d%02d_%03d.txt",
            ptm->tm_year + 1900, ptm->tm_mon + 1, ptm->tm_mday,
            ptm->tm_hour, ptm->tm_min, ptm->tm_sec, milli);

    FILE *textFile = fopen(nameTextFile, "w");
    if (textFile == 0)
        return;

    int backCount = 0;

    // Back up to find the frame that is far enough before the start frame and also a key frame
    while (backCount++ < 15 || !IsIDR(frameBuffers[frameIndex], frameSizes[frameIndex]))
    {
        frameIndex = (frameIndex + FRAMES_TO_KEEP - 1) % FRAMES_TO_KEEP;
        sumIndex = (sumIndex + FRAMES_TO_KEEP - 1) % FRAMES_TO_KEEP;

        if (backCount > 60)
            break;
    }

    int frameCount = 0;
    double xCentroid = 0.0;
    double yCentroid = 0.0;

    while (sumIndex != stopIndex)
    {
        double delta = 1.0e-6 * ((int)frameTimestampLo[frameIndex] - (int)arg.eventStartTimeLo);
        int counts = sumCounts[sumIndex];
        int sum = sumBuffers[sumIndex];
        xCentroid = (sum == 0) ? xCentroid : (double)xSumBuffers[sumIndex] / sum;
        yCentroid = (sum == 0) ? yCentroid : (double)ySumBuffers[sumIndex] / sum;
        fprintf(textFile, "%3d %7.3f %6d %6d %7.1f %7.1f\n",
                ++frameCount, delta, counts, sum, xCentroid, yCentroid);

        fwrite(frameBuffers[frameIndex], frameSizes[frameIndex], 1, videoFile);
        frameIndex = (frameIndex + 1) % FRAMES_TO_KEEP;
        sumIndex = (sumIndex + 1) % FRAMES_TO_KEEP;
    }

    fclose(videoFile);
    fclose(textFile);

    strcpy(nameMp4File, nameVideoFile);
    char *pstr = strstr(nameMp4File, "h264");
    if (pstr == 0)
        return;

    fprintf(stderr, "Event: %04d/%02d/%02d %02d:%02d:%02d.%03d\n",
           ptm->tm_year + 1900, ptm->tm_mon + 1, ptm->tm_mday,
           ptm->tm_hour, ptm->tm_min, ptm->tm_sec, milli);

    strcpy(pstr, "mp4");

    int iFrameRate = round(frameRate);

    sprintf(nameSystemCmd, "MP4Box -add %s -fps %d -quiet -new %s", nameVideoFile, iFrameRate, nameMp4File);
    system(nameSystemCmd);
}

static void *saveEventThread(void *argin)
{
    for (;;)
    {
        pthread_mutex_lock(&saveEventThreadMutex);
        while (!threadingJob.jobPending && !thread_exit)
            pthread_cond_wait(&saveEventThreadCondition, &saveEventThreadMutex);

        if (thread_exit)
        {
            pthread_mutex_unlock(&saveEventThreadMutex);
            return 0;
        }

        saveEventJob();

        threadingJob.jobPending = 0;
        pthread_mutex_unlock(&saveEventThreadMutex);
    }
}

static void initiateTrigger(unsigned int timeStamp_hi, unsigned int timeStamp_lo)
{
    eventTimeStamp_hi = timeStamp_hi;
    eventTimeStamp_lo = timeStamp_lo;

    triggered = 1;
    untriggered = 0;
    eventDuration = 0;

    // printf( "Triggered\n" );
}

static void continueTrigger(unsigned int timeStamp_lo)
{
}

static void terminateTrigger(unsigned int timeStamp_hi, unsigned int timeStamp_lo)
{
    triggered = 0;
    untriggered = 0;

    // Temporarily de-sensitize
    memset(referenceFrame, 0xff, 1920 * 1080 / 9);

    int test = rateLimitEventsPerHour * rateLimitBank / 3600;
    if (test == 0)
        return;

    pthread_mutex_lock(&saveEventThreadMutex);

    threadingJob.eventStartTimeHi = eventTimeStamp_hi;
    threadingJob.eventStartTimeLo = eventTimeStamp_lo;
    threadingJob.eventStopTimeHi = timeStamp_hi;
    threadingJob.eventStopTimeLo = timeStamp_lo;
    threadingJob.jobPending = 1;

    pthread_mutex_unlock(&saveEventThreadMutex);
    pthread_cond_signal(&saveEventThreadCondition);

    rateLimitBank -= 3600 / rateLimitEventsPerHour;
    if (rateLimitBank < 0)
        rateLimitBank = 0;
    // printf( "Untriggered\n" );
}

static void processTrigger(int sum, unsigned int timeStamp_hi, unsigned int timeStamp_lo)
{
    ++frame_number;

    if (triggered == 0)
    {
        if (frame_number == force_count)
        {
            initiateTrigger(timeStamp_hi, timeStamp_lo);
        }
        else if (sum >= sumThreshold)
        {
            ++triggerCounter;
            if (triggerCounter >= 2)
                initiateTrigger(timeStamp_hi, timeStamp_lo);
        }
        else
            triggerCounter = 0;
    }
    else if (triggered != 0 && untriggered == 0)
    {
        ++eventDuration;

        continueTrigger(timeStamp_lo);

        if (sum < sumThreshold)
        {
            ++untriggerCounter;
            if (untriggerCounter >= 2)
                untriggered = 1;
        }
        else
        {
            untriggerCounter = 0;
            if (eventDuration > 150)
                terminateTrigger(timeStamp_hi, timeStamp_lo);
        }
    }
    else
    {
        ++eventDuration;

        continueTrigger(timeStamp_lo);

        if (eventDuration > 60 && sum < sumThreshold)
        {
            ++untriggerCounter;
            if (untriggerCounter >= 15)
                terminateTrigger(timeStamp_hi, timeStamp_lo);
        }
        else
        {
            if (eventDuration > 150)
                terminateTrigger(timeStamp_hi, timeStamp_lo);
        }
    }
}

static void limitEventRate(unsigned microseconds)
{
    int seconds = microseconds / 1000000;

    if (seconds != rateLimitLastSecond)
    {
        ++rateLimitBank;
        if (rateLimitBank > 3600)
            rateLimitBank = 3600;

        rateLimitLastSecond = seconds;
    }
}

//
// Measure the filtered frame rate
// If the frame rate drops we may want to switch to manual exposure mode.
//
static void measureFrameRate(unsigned microseconds)
{
    static unsigned int lastMicrosecond = 0;

    if ( microseconds > lastMicrosecond )
    {
        unsigned delta = microseconds - lastMicrosecond;
        if ( delta > 20000 && delta < 100000 )
        {
            double rate = 1.0e6 / delta;

            frameRate = 0.99 * frameRate + 0.01 * rate;
        }
    }

    lastMicrosecond = microseconds;
}

//
// Measure the filtered average amplitude of an arbitrary collection of pixels near the zenith.
// This is used to determine if we are looking at a daylight sky where auto-exposure may be the preferred mode.
//
static void measureZenithAmplitude()
{
    if ( testFrame == 0 )
        return;

    int sum = 0;

    for ( int row = 170; row <= 190; row += 10)
    {
        for ( int col = 310; col <= 330; col += 10 )
            sum += testFrame[640*row + col];
    }

    double average = sum / 9.0;

    zenithAmplitude = 0.99 * zenithAmplitude + 0.01 * average;
}

//
// This processes each frame looking for changes in brightness
// To save on processing time, not every pixel is examined, just the center pixel of every 3x3 pixel grid.
//
static void ProcessDecodedBuffer(OMX_BUFFERHEADERTYPE *workingBuffer)
{
    static int countn = 0;
    // clock_t t = clock();
    // fprintf( logfile, "clock %d\n", ++countn );

    int sum = 0;
    int xsum = 0;
    int ysum = 0;
    int count = 0;
    int xcount = 0;
    int ycount = 0;

    unsigned char *pstart = (unsigned char *)workingBuffer->pBuffer;

    unsigned char *p = pstart + 1920 + 1;
    unsigned char *pvend = pstart + 1920 * 1080;
    unsigned char *psend = pstart + 1920 + 1920;

    unsigned char *pTest = testFrame;
    unsigned char *pRef = referenceFrame;
    unsigned char *pMask = maskFrame;

    while (p < pvend)
    {
        xcount = 0;
        while (p < psend)
        {
            *pTest++ = *p;
            int c = *p;

            int test = c - *pRef - *pMask++;
            if (test > 0)
            {
                sum += test;
                xsum += test * xcount;
                ysum += test * ycount;
                ++count;
            }

            p += 3;
            ++pRef;
            ++xcount;
        }

        p += 3840;
        psend += 5760;
        ++ycount;
    }

    pTest = testFrame;
    pRef = referenceFrame;

    pvend = testFrame + 1920 * 1080 / 9;

    while (pTest < pvend)
    {
        int n = *pRef;
        n *= 7;
        n += *pTest++;
        n >>= 3;

        *pRef++ = n;
    }

    sumBuffers[nextSumIndex] = sum;
    xSumBuffers[nextSumIndex] = xsum;
    ySumBuffers[nextSumIndex] = ysum;
    sumCounts[nextSumIndex] = count;
    sumTimestamps[nextSumIndex] = workingBuffer->nTimeStamp.nLowPart;
    nextSumIndex = (nextSumIndex + 1) % FRAMES_TO_KEEP;

    processTrigger(sum, workingBuffer->nTimeStamp.nHighPart, workingBuffer->nTimeStamp.nLowPart);
    limitEventRate(workingBuffer->nTimeStamp.nLowPart);
    measureFrameRate(workingBuffer->nTimeStamp.nLowPart);
    measureZenithAmplitude();

    if ( logfile )
        fprintf(logfile, "Sum Count Ref: %4d %3d %x\n", sum, count, referenceFrame[100]);
}

static void *decodedBufferThread(void *argin)
{
    OMX_ERRORTYPE error;
    int countn = 1;

    long int start_time;
    long int time_difference;
    struct timespec gettime_now;

    for (;;)
    {
        pthread_mutex_lock(&decodeBufferThreadMutex);
        while (outputBuffer == 0 && !thread_exit)
            pthread_cond_wait(&decodeBufferThreadCondition, &decodeBufferThreadMutex);

        clock_gettime(CLOCK_REALTIME, &gettime_now);
        start_time = gettime_now.tv_nsec; //Get nS value

        OMX_BUFFERHEADERTYPE *workingBuffer = outputBuffer;
        outputBuffer = 0;

        pthread_mutex_unlock(&decodeBufferThreadMutex);

        if (thread_exit)
            return 0;

        int ping = workingBuffer == pingBuffer;
        int pong = workingBuffer == pongBuffer;

        // fprintf( logfile, "FillThisBuffer %d\n", ++countn );

        error = OMX_FillThisBuffer(decoderHandle,
                                   ping ? pongBuffer : pingBuffer);

        if (error != OMX_ErrorNone)
        {
            if (errno == 0)
                return 0;

            fprintf(stderr, "FillThisBuffer failed: %d, %s, %d %d %d\n",
                    errno, strerror(errno), ping, pong, outputBuffer != 0);

            if (logfile != 0)
            {
                fflush(logfile);
                fclose(logfile);
            }

            exit(EXIT_FAILURE);
        }

        ProcessDecodedBuffer(workingBuffer);

        clock_gettime(CLOCK_REALTIME, &gettime_now);
        time_difference = gettime_now.tv_nsec - start_time;
        if (time_difference < 0)
            time_difference += 1000000000;

        double duration = 1.0e-9 * time_difference;

        if ( logfile )
            fprintf(logfile, "Duration: %7.4f\n", duration);
    }
}

static OMX_ERRORTYPE
FillBufferDone(OMX_HANDLETYPE omx_handle, OMX_PTR app_data, OMX_BUFFERHEADERTYPE *omx_buffer)
{
    static int count = 0;

    if (omx_buffer == 0)
        return OMX_ErrorNone;

    if (outputBuffer != 0)
    {
        fprintf(stderr, "Buffer Overflow\n");
        exit(EXIT_FAILURE);
    }

    pthread_mutex_lock(&decodeBufferThreadMutex);
    outputBuffer = omx_buffer;
    pthread_mutex_unlock(&decodeBufferThreadMutex);
    pthread_cond_signal(&decodeBufferThreadCondition);

    // if ( logfile )
    // 	fprintf( logfile, "FillBufferDone %d\n", ++count );

    return OMX_ErrorNone;
}

static OMX_ERRORTYPE
EmptyBufferDone(OMX_HANDLETYPE omx_handle, OMX_PTR app_data, OMX_BUFFERHEADERTYPE *omx_buffer)
{
    static int count = 0;

    pthread_mutex_lock(&composeBufferThreadMutex);
    decodeBufferFull = 0;
    pthread_mutex_unlock(&composeBufferThreadMutex);
    pthread_cond_signal(&composeBufferThreadCondition);

    // if ( logfile )
    // 	fprintf( logfile, "EmptyBufferDone %d\n", ++count );

    return OMX_ErrorNone;
}

static void decodeLoop(void)
{
    unsigned int count;
    unsigned int loopIsInfinite = 0;

    if (frame_count == 0)
        loopIsInfinite = 1; //infinite loop
    count = frame_count;
    decodeBufferFull = 0;

    while (!stop_flag && (count-- > 0 || loopIsInfinite))
    {
        for (;;)
        {
            fd_set fds;
            struct timeval tv;
            int r;

            FD_ZERO(&fds);
            FD_SET(fd, &fds);

            /* Timeout. */
            tv.tv_sec = 2;
            tv.tv_usec = 0;

            r = select(fd + 1, &fds, NULL, NULL, &tv);

            if (-1 == r)
            {
                if (EINTR == errno)
                    continue;
                errno_exit("select");
            }

            if (0 == r)
            {
                time_t result = time(NULL);
                fprintf(stderr, "select timeout at: %s\n", ctime(&result));
                continue;

                // exit(EXIT_FAILURE);
            }

            if (decode_frame())
                break;
            /* EAGAIN - continue select loop. */
        }
    }
}

static void omx_die(OMX_ERRORTYPE error, const char* message) 
{
	const char* e;
	switch(error) 
    {
		case OMX_ErrorNone:                     e = "no error";                                      break;
    	case OMX_ErrorBadParameter:             e = "bad parameter";                                 break;
		case OMX_ErrorIncorrectStateOperation:  e = "invalid state while trying to perform command"; break;
		case OMX_ErrorIncorrectStateTransition: e = "unallowed state transition";                    break;
		case OMX_ErrorInsufficientResources:    e = "insufficient resource";                         break;
		case OMX_ErrorBadPortIndex:             e = "bad port index, i.e. incorrect port";           break;
		case OMX_ErrorHardware:                 e = "hardware error";                                break;
		default:                                e = "(no description)";
    }

	fprintf(stderr, "0x%08x %s %s\n", error, message, e);
	exit(EXIT_FAILURE);
}

//
// Initialize the decoder used to decode .h264 frames into YUV420P frames
//
static void init_decoder(void)
{
    OMX_ERRORTYPE error;
    OMX_PARAM_PORTDEFINITIONTYPE paramPort;
    OMX_VIDEO_PARAM_PORTFORMATTYPE format;

    int inputBufferSize;

    error = OMX_Init();
    if (error != OMX_ErrorNone)
        omx_die( error, "Could not init");

    decoderCallbacks.EventHandler = EventHandler;
    decoderCallbacks.FillBufferDone = FillBufferDone;
    decoderCallbacks.EmptyBufferDone = EmptyBufferDone;

    error = OMX_GetHandle(&decoderHandle,
                          "OMX.broadcom.video_decode",
                          NULL,
                          &decoderCallbacks);

    if (error != OMX_ErrorNone)
        omx_die( error, "Could not get decoder handle");

    memset(&format, 0, sizeof(OMX_VIDEO_PARAM_PORTFORMATTYPE));
    format.nSize = sizeof(OMX_VIDEO_PARAM_PORTFORMATTYPE);
    format.nVersion.nVersion = OMX_VERSION;
    format.nPortIndex = 130;
    format.eCompressionFormat = OMX_VIDEO_CodingAVC;

    error = OMX_SetParameter(decoderHandle, OMX_IndexParamVideoPortFormat, &format);
    if (error != OMX_ErrorNone)
        omx_die( error, "Could not set parameter");

    paramPort.nSize = sizeof(OMX_PARAM_PORTDEFINITIONTYPE);
    paramPort.nVersion.nVersion = OMX_VERSION;
    paramPort.nPortIndex = 130;
    error = OMX_GetParameter(decoderHandle, OMX_IndexParamPortDefinition, &paramPort);
    if (error != OMX_ErrorNone)
        omx_die( error, "Could not get parameter");

    paramPort.nBufferCountActual = paramPort.nBufferCountMin;
    paramPort.nBufferSize = 4147200;
    error = OMX_SetParameter(decoderHandle, OMX_IndexParamPortDefinition, &paramPort);
    if (error != OMX_ErrorNone)
        omx_die( error, "Could not set parameter");

    inputBufferSize = paramPort.nBufferSize;

    paramPort.nPortIndex = 131;
    error = OMX_GetParameter(decoderHandle, OMX_IndexParamPortDefinition, &paramPort);
    if (error != OMX_ErrorNone)
        omx_die( error, "Could not get parameter");

    paramPort.nBufferCountActual = 2;
    paramPort.format.video.nFrameWidth = 1920;
    paramPort.format.video.nFrameHeight = 1080;
    paramPort.format.video.nStride = ALIGN(paramPort.format.video.nFrameWidth, 32);
    paramPort.format.video.nSliceHeight = ALIGN(paramPort.format.video.nFrameHeight, 16);
    paramPort.format.video.eColorFormat=OMX_COLOR_FormatYUV420PackedPlanar;

    paramPort.nBufferSize = paramPort.format.image.nStride *
                            paramPort.format.image.nSliceHeight * 3 / 2;

    error = OMX_SetParameter(decoderHandle, OMX_IndexParamPortDefinition, &paramPort);
    if (error != OMX_ErrorNone)
        omx_die( error, "Could not set parameter");

    // It appears that ports are already enabled so these are not needed
    // error = OMX_SendCommand ( decoderHandle, OMX_CommandPortEnable, 130, NULL);
    // error = OMX_SendCommand ( decoderHandle, OMX_CommandPortEnable, 131, NULL);
    error = OMX_SendCommand(decoderHandle, OMX_CommandStateSet, OMX_StateIdle, NULL);
    if (error != OMX_ErrorNone)
        omx_die( error, "Could not send command");

    error = OMX_AllocateBuffer(decoderHandle, &decoderBuffer, 130, NULL, inputBufferSize);
    if (error != OMX_ErrorNone)
        omx_die( error, "Could not allocate decode buffer");

    error = OMX_AllocateBuffer(decoderHandle, &pingBuffer, 131, NULL, paramPort.nBufferSize);
    if (error != OMX_ErrorNone)
        omx_die( error, "Could not allocate ping buffer");

    error = OMX_AllocateBuffer(decoderHandle, &pongBuffer, 131, NULL, paramPort.nBufferSize);
    if (error != OMX_ErrorNone)
        omx_die( error, "Could not allocate pong buffer");

    outputBuffer = 0;

    error = OMX_SendCommand(decoderHandle, OMX_CommandStateSet, OMX_StateExecuting, NULL);
    if (error != OMX_ErrorNone)
        omx_die( error, "Could not send commands");

    error = OMX_FillThisBuffer(decoderHandle, pingBuffer);
    if (error != OMX_ErrorNone)
        omx_die( error, "Could not Fill This Buffer");
}

static void init_jpeg_decoder(void)
{
    OMX_ERRORTYPE error;
    OMX_PARAM_PORTDEFINITIONTYPE paramPort;
    OMX_IMAGE_PARAM_PORTFORMATTYPE format;

    int inputBufferSize;

    error = OMX_Init();
    if (error != OMX_ErrorNone)
        omx_die( error, "Could not init");

    decoderCallbacks.EventHandler = EventHandler;
    decoderCallbacks.FillBufferDone = FillBufferDone;
    decoderCallbacks.EmptyBufferDone = EmptyBufferDone;

    error = OMX_GetHandle(&decoderHandle,
                          "OMX.broadcom.image_decode",
                          NULL,
                          &decoderCallbacks);

    if (error != OMX_ErrorNone)
        omx_die( error, "Could not get decoder handle");

    OMX_PORT_PARAM_TYPE port;
    memset(&port, 0, sizeof(OMX_PORT_PARAM_TYPE));
    port.nSize = sizeof(OMX_PORT_PARAM_TYPE);
    port.nVersion.nVersion = OMX_VERSION;
    error = OMX_GetParameter(decoderHandle, OMX_IndexParamImageInit, &port);
    if ( error != OMX_ErrorNone )
        omx_die( error, "Could not get port parameters");

    int inPort = port.nStartPortNumber;
    int outPort = port.nStartPortNumber + 1;

    error = OMX_SendCommand ( decoderHandle, OMX_CommandPortDisable, inPort, NULL);
    if ( error != OMX_ErrorNone )
        omx_die( error, "Could not disable port");

    error = OMX_SendCommand ( decoderHandle, OMX_CommandPortDisable, outPort, NULL);
    if ( error != OMX_ErrorNone )
        omx_die( error, "Could not disable port");

    error = OMX_SendCommand(decoderHandle, OMX_CommandStateSet, OMX_StateIdle, NULL);
    if (error != OMX_ErrorNone)
        omx_die( error, "Could not send commands");

    memset(&format, 0, sizeof(OMX_IMAGE_PARAM_PORTFORMATTYPE));
    format.nSize = sizeof(OMX_IMAGE_PARAM_PORTFORMATTYPE);
    format.nVersion.nVersion = OMX_VERSION;
    format.nPortIndex = inPort;
    format.eCompressionFormat = OMX_IMAGE_CodingJPEG;

    error = OMX_SetParameter(decoderHandle, OMX_IndexParamImagePortFormat, &format);
    if (error != OMX_ErrorNone)
        omx_die( error, "Could not set parameter");

    memset(&paramPort, 0, sizeof(OMX_PARAM_PORTDEFINITIONTYPE));
    paramPort.nSize = sizeof(OMX_PARAM_PORTDEFINITIONTYPE);
    paramPort.nVersion.nVersion = OMX_VERSION;
    paramPort.nPortIndex = inPort;
    error = OMX_GetParameter(decoderHandle, OMX_IndexParamPortDefinition, &paramPort);
    if (error != OMX_ErrorNone)
        omx_die( error, "Could not get parameter");

    int bufferCount = paramPort.nBufferCountActual;
    int bufferSize = paramPort.nBufferSize;

    error = OMX_SendCommand ( decoderHandle, OMX_CommandPortEnable, inPort, NULL);
    if ( error != OMX_ErrorNone )
        omx_die( error, "Could not enable port");

    for ( int i = 0; i < 3; ++i )
    {
        error = OMX_AllocateBuffer(decoderHandle, &jpegBuffers[i], inPort, NULL, bufferSize);
        if (error != OMX_ErrorNone)
            omx_die( error, "Could not allocate buffers");
    }

    paramPort.nPortIndex = outPort;
    error = OMX_GetParameter(decoderHandle, OMX_IndexParamPortDefinition, &paramPort);
    if (error != OMX_ErrorNone)
        omx_die( error, "Could not get parameter");

    paramPort.format.image.eCompressionFormat = OMX_IMAGE_CodingAutoDetect;

    error = OMX_SetParameter(decoderHandle, OMX_IndexParamPortDefinition, &paramPort);
    if (error != OMX_ErrorNone)
        omx_die( error, "Could not set parameter");

    outputBuffer = 0;

    error = OMX_SendCommand(decoderHandle, OMX_CommandStateSet, OMX_StateExecuting, NULL);
    if (error != OMX_ErrorNone)
        omx_die( error, "Could not send commands");
}

static void init_jpeg_output( OMX_HANDLETYPE decoderHandle, int outPort )
{
    OMX_ERRORTYPE error;
    OMX_PARAM_PORTDEFINITIONTYPE paramPort;

    error = OMX_SendCommand ( decoderHandle, OMX_CommandPortEnable, outPort, NULL);
    if ( error != OMX_ErrorNone )
        omx_die( error, "Could not enable port");

    memset(&paramPort, 0, sizeof(OMX_PARAM_PORTDEFINITIONTYPE));
    paramPort.nSize = sizeof(OMX_PARAM_PORTDEFINITIONTYPE);
    paramPort.nVersion.nVersion = OMX_VERSION;
    paramPort.nPortIndex = outPort;

    error = OMX_GetParameter(decoderHandle, OMX_IndexParamPortDefinition, &paramPort);
    if (error != OMX_ErrorNone)
        omx_die( error, "Could not get parameter");

    error = OMX_AllocateBuffer( decoderHandle, &pingBuffer, outPort, NULL, paramPort.nBufferSize);
    if (error != OMX_ErrorNone)
        omx_die( error, "Could not allocate ping buffer");

    error = OMX_FillThisBuffer(decoderHandle, pingBuffer);
    if (error != OMX_ErrorNone)
        omx_die( error, "Could not Fill This Buffer");
}

//
// Initialize the encoder used to convert from YUV420P format to JPEG
//
static void init_encoder(void)
{
    OMX_ERRORTYPE error;
    OMX_PARAM_PORTDEFINITIONTYPE encoder_portdef;
    OMX_IMAGE_PARAM_PORTFORMATTYPE format;

    int len = sizeof(OMX_IMAGE_PARAM_PORTFORMATTYPE);
    memset(&format, 0, len);

    int inputBufferSize;

    error = OMX_Init();
    if ( error != OMX_ErrorNone )
        omx_die( error, "Could not init");

    encoderCallbacks.EventHandler = EventHandler;
    encoderCallbacks.FillBufferDone = FillBufferDone;
    encoderCallbacks.EmptyBufferDone = EmptyBufferDone;

    error = OMX_GetHandle(&encoderHandle, 
                          "OMX.broadcom.image_encode", 
                          NULL, 
                          &encoderCallbacks);
                          
    if (error != OMX_ErrorNone)
        omx_die( error, "Cannot get encoder handle");

	OMX_INIT_STRUCTURE(encoder_portdef);
    encoder_portdef.nPortIndex=340; // Input port
    error = OMX_GetParameter(encoderHandle, OMX_IndexParamPortDefinition, &encoder_portdef);
    if ( error != OMX_ErrorNone)
        omx_die( error, "Cannot get encoder parameter");

    encoder_portdef.format.image.nFrameWidth=IMAGE_WIDTH;
	encoder_portdef.format.image.nFrameHeight=IMAGE_HEIGHT;
    int width32 = ALIGN(IMAGE_WIDTH, 32);
    int height16 = ALIGN(IMAGE_HEIGHT, 16);
	encoder_portdef.format.image.nSliceHeight=height16;
	encoder_portdef.format.image.nStride=width32;
	encoder_portdef.format.image.bFlagErrorConcealment=OMX_FALSE;
	encoder_portdef.format.image.eColorFormat=OMX_COLOR_FormatYUV420PackedPlanar;
	encoder_portdef.format.image.eCompressionFormat=OMX_IMAGE_CodingUnused;
	encoder_portdef.nBufferSize=1920*1088*3/2;

    error = OMX_SetParameter( encoderHandle, OMX_IndexParamPortDefinition, &encoder_portdef);
	if( error != OMX_ErrorNone) 
        omx_die( error, "Cannot get encoder parameter");

	OMX_INIT_STRUCTURE(encoder_portdef);
	encoder_portdef.nPortIndex=340; // Input port

    error = OMX_GetParameter(encoderHandle, OMX_IndexParamPortDefinition, &encoder_portdef);
	if( error != OMX_ErrorNone) 
        omx_die( error, "Cannot get encoder parameter");

	OMX_INIT_STRUCTURE(encoder_portdef);
    encoder_portdef.nPortIndex=341; // Output port

    error = OMX_GetParameter( encoderHandle, OMX_IndexParamPortDefinition, &encoder_portdef);
	if( error != OMX_ErrorNone) 
        omx_die( error, "Cannot get encoder parameter");

    encoder_portdef.nBufferSize = 200000;
    error = OMX_SetParameter(encoderHandle, OMX_IndexParamPortDefinition, &encoder_portdef);
    if (error != OMX_ErrorNone)
        omx_die( error, "Failed to set port definition for encoder output port 341");

    // Set the JPEG quality factor
    OMX_IMAGE_PARAM_QFACTORTYPE qFactor;
    OMX_INIT_STRUCTURE(qFactor);
    qFactor.nPortIndex = encoder_portdef.nPortIndex;
    qFactor.nQFactor = 15;
    error = OMX_SetParameter(encoderHandle, OMX_IndexParamQFactor, &qFactor);
    if (error != OMX_ErrorNone)
        omx_die( error, "Could not set Q Factor");

	format.nSize = len;
    format.nVersion.nVersion = OMX_VERSION;
    format.nPortIndex = 341;
    format.eCompressionFormat = OMX_IMAGE_CodingJPEG;
    format.eColorFormat = OMX_COLOR_FormatUnused;

    // updating attributes from port 341 (output)
    error = OMX_SetParameter( encoderHandle, OMX_IndexParamImagePortFormat, &format);
	if( error != OMX_ErrorNone) 
        omx_die( error, "Cannot set encoder parameter");

    // It appears that ports are already enabled so these are not needed
    // error = OMX_SendCommand ( encoderHandle, OMX_CommandPortEnable, 340, NULL);
    // error = OMX_SendCommand ( encoderHandle, OMX_CommandPortEnable, 341, NULL);
    error = OMX_SendCommand( encoderHandle, OMX_CommandStateSet, OMX_StateIdle, NULL);
    if (error != OMX_ErrorNone)
        omx_die( error, "Cannot send command");

	OMX_INIT_STRUCTURE(encoder_portdef);
	encoder_portdef.nPortIndex=340;
    error = OMX_GetParameter(encoderHandle, OMX_IndexParamPortDefinition, &encoder_portdef);
    if (error != OMX_ErrorNone)
        omx_die( error, "Failed to get port definition for encoder input port 340");

    error = OMX_AllocateBuffer(encoderHandle, &encoderBuffer, 340, NULL, encoder_portdef.nBufferSize);
    if (error != OMX_ErrorNone)
        omx_die( error, "Cannot allocate Buffer");

	OMX_INIT_STRUCTURE(encoder_portdef);
	encoder_portdef.nPortIndex=341;
    error = OMX_GetParameter(encoderHandle, OMX_IndexParamPortDefinition, &encoder_portdef);
    if (error != OMX_ErrorNone)
        omx_die( error, "Failed to get port definition for encoder output port 341");

    error = OMX_AllocateBuffer(encoderHandle, &imageBuffer, 341, NULL, encoder_portdef.nBufferSize);
    if (error != OMX_ErrorNone)
        omx_die( error, "Cannot allocate Buffer");

    outputBuffer = 0;

    error = OMX_SendCommand(encoderHandle, OMX_CommandStateSet, OMX_StateExecuting, NULL);
    if (error != OMX_ErrorNone)
        omx_die( error, "Cannot send commands");
}

static void uninit_encoder(void)
{
    OMX_ERRORTYPE error = OMX_ErrorNone;

    error = OMX_SendCommand(encoderHandle, OMX_CommandStateSet, OMX_StateIdle, NULL);
    if (error != OMX_ErrorNone)
        omx_die( error, "Cannot set idle state");

    error = OMX_SendCommand(encoderHandle, OMX_CommandPortDisable, 340, NULL);
    if (error != OMX_ErrorNone)
        omx_die( error, "Cannot disable port 340");

    error = OMX_SendCommand(encoderHandle, OMX_CommandPortDisable, 341, NULL);
    if (error != OMX_ErrorNone)
        omx_die( error, "Cannot disable port 341");

    error = OMX_FreeBuffer(encoderHandle, 340, encoderBuffer);
    if (error != OMX_ErrorNone)
        omx_die( error, "Cannot free encoder buffer");

    error = OMX_FreeBuffer(encoderHandle, 341, imageBuffer);
    if (error != OMX_ErrorNone)
        omx_die( error, "Cannot free image buffer");

    error = OMX_FreeHandle(encoderHandle);
    if (error != OMX_ErrorNone)
        omx_die( error, "Cannot free handle");

    error = OMX_Deinit();
    if (error != OMX_ErrorNone)
        omx_die( error, "Cannot Deinit");
}

static void init_sentinel(void)
{
    int frame_size = 1920 * 1080 / 9;

    if ( testFrame == 0 )
    {
        testFrame = (unsigned char *)malloc(frame_size);
        memset( testFrame, 0, frame_size );
    }

    if ( referenceFrame == 0 )
    {
        referenceFrame = (unsigned char *)malloc(frame_size);
        memset( referenceFrame, 255, frame_size );
    }

    if ( maskFrame == 0 )
        maskFrame = (unsigned char *)malloc(frame_size);

    memset( maskFrame, noise_level, frame_size );

    for (int i = 0; i < FRAMES_TO_KEEP; ++i)
    {
        frameBuffers[i] = 0;
        frameAlloc[i] = 0;
        frameSizes[i] = 0;
        sumBuffers[i] = 0;
        sumCounts[i] = 0;
        xSumBuffers[i] = 0;
        ySumBuffers[i] = 0;
    }

    nextFrameIndex = 0;
    archiveFrameIndex = 0;
    eventFrameIndex = 0;
    eventFrameStop = 0;
    nextSumIndex = 0;
    eventSumIndex = 0;
    triggered = 0;
    untriggered = 0;
    triggerCounter = 0;
    untriggerCounter = 0;
    eventDuration = 0;
    videoFile = 0;
    textFile = 0;

    rateLimitLastSecond = 0;
    rateLimitBank = 3600;

    threadingJob.eventStartTimeHi = 0;
    threadingJob.eventStartTimeLo = 0;
    threadingJob.eventStopTimeHi = 0;
    threadingJob.eventStopTimeLo = 0;
    threadingJob.jobPending = 0;

    thread_exit = 0;

    int rc = 0;

    rc = pthread_create(&archiveThread_id, NULL, &archiveThread, NULL);
    if (rc)
    {
        fprintf(stderr, "Error creating archiveThread_id\n");
    }

    rc = pthread_create(&saveEventThread_id, NULL, &saveEventThread, NULL);
    if (rc)
    {
        fprintf(stderr, "Error creating saveEventThread_id\n");
    }

    outputBuffer = 0;
    rc = pthread_create(&decodeBufferThread_id, NULL, &decodedBufferThread, NULL);
    if (rc)
    {
        fprintf(stderr, "Error creating decodeBufferThread_id\n");
    }
}

static void uninit_sentinel(void)
{
    thread_exit = 1;

    pthread_cond_signal(&saveEventThreadCondition);
    pthread_cond_signal(&decodeBufferThreadCondition);

    pthread_join(saveEventThread_id, NULL);
    pthread_join(decodeBufferThread_id, NULL);
    pthread_join(archiveThread_id, NULL);

    // fprintf(stderr, "Free testFrame\n");
    free(testFrame);
    // fprintf(stderr, "Free referenceFrame\n");
    free(referenceFrame);
    // fprintf(stderr, "Free maskFrame\n");
    free(maskFrame);

    for ( int i = 0; i < FRAMES_TO_KEEP; ++i )
    {
        free(frameBuffers[i]);
        frameAlloc[i] = 0;
    }

    testFrame = 0;
    referenceFrame = 0;
    maskFrame = 0;

    if ( videoArchive != 0 )
    {
        fclose(videoArchive);
        videoArchive = 0;
    }

    if ( textArchive != 0 )
    {
        fclose(textArchive);
        textArchive = 0;
    }
}

static void uninit_decoder(void)
{
    // printf( "uninit_decoder\n" );
    OMX_ERRORTYPE error = OMX_ErrorNone;

    error = OMX_SendCommand(decoderHandle, OMX_CommandFlush, 130, NULL );
    if (error != OMX_ErrorNone)
        omx_die( error, "Could not flush port 130");

    error = OMX_SendCommand(decoderHandle, OMX_CommandFlush, 131, NULL );
    if (error != OMX_ErrorNone)
        omx_die( error, "Could not flush port 131");

    error = OMX_SendCommand(decoderHandle, OMX_CommandStateSet, OMX_StateIdle, NULL);
    if (error != OMX_ErrorNone)
        omx_die( error, "Could not set state to idle");

    error = OMX_SendCommand(decoderHandle, OMX_CommandPortDisable, 130, NULL);
    if (error != OMX_ErrorNone)
        omx_die( error, "Could not disable port 130");

    error = OMX_SendCommand(decoderHandle, OMX_CommandPortDisable, 131, NULL);
    if (error != OMX_ErrorNone)
        omx_die( error, "Could not disable port 131");

    error = OMX_FreeBuffer(decoderHandle, 130, decoderBuffer);
    if (error != OMX_ErrorNone)
        omx_die( error, "Could not free decoder buffer");
    decoderBuffer = 0;

    error = OMX_FreeBuffer(decoderHandle, 131, pongBuffer);
    if (error != OMX_ErrorNone)
        omx_die( error, "Could not free pong buffer");
    pongBuffer = 0;

    error = OMX_FreeBuffer(decoderHandle, 131, pingBuffer);
    if (error != OMX_ErrorNone)
        omx_die( error, "Could not free ping buffer");
    pingBuffer = 0;

    error = OMX_FreeHandle(decoderHandle);
    if (error != OMX_ErrorNone)
        omx_die( error, "Could not free handle");

    error = OMX_Deinit();
    if (error != OMX_ErrorNone)
        omx_die( error, "Could not Deinit");
}

static void uninit_jpeg_decoder(void)
{
    OMX_ERRORTYPE error = OMX_ErrorNone;

    error = OMX_SendCommand(decoderHandle, OMX_CommandStateSet, OMX_StateIdle, NULL);
    if (error != OMX_ErrorNone)
        omx_die( error, "Could not set state to idle");

    error = OMX_SendCommand(decoderHandle, OMX_CommandPortDisable, 320, NULL);
    if (error != OMX_ErrorNone)
        omx_die( error, "Could not disable port 320");

    error = OMX_SendCommand(decoderHandle, OMX_CommandPortDisable, 321, NULL);
    if (error != OMX_ErrorNone)
        omx_die( error, "Could not disable port 321");

    for ( int i = 0; i < 3; ++i )
    {
        error = OMX_FreeBuffer(decoderHandle, 320, jpegBuffers[i]);
        if (error != OMX_ErrorNone)
            omx_die( error, "Could not free JPEG buffers");
    }

    if ( pingBuffer != 0 )
    {
        error = OMX_FreeBuffer(decoderHandle, 321, pingBuffer);
        if (error != OMX_ErrorNone)
            omx_die( error, "Could not free ping buffer");
        pingBuffer = 0;
    }

    error = OMX_FreeHandle(decoderHandle);
    if (error != OMX_ErrorNone)
        omx_die( error, "Could not free handle");

    error = OMX_Deinit();
    if (error != OMX_ErrorNone)
        omx_die( error, "Could not Deinit");
}

static void readMask(void)
{
    char buffer[100];

    int frame_size = 1920 * 1080 / 9;

    if ( maskFrame == 0 )
        maskFrame = (unsigned char *)malloc(frame_size);

    memset( maskFrame, noise_level, frame_size );

    FILE *file = fopen("mask.ppm", "rb");
    if (file == 0)
    {
        fprintf(stderr, "No mask file found. Noise level set to: %d\n", noise_level);
        return;
    }

    int width;
    int height;
    int maxval;

    int count = fscanf(file, "%s ", buffer);
    if (count != 1 || buffer[0] != 'P' || buffer[1] != '6')
    {
        fprintf(stderr, "Cannot read mask file, not a PPM file.\n");
        exit(EXIT_FAILURE);
    }

    int c = fgetc(file);
    if (c == '#')
        fgets(buffer, 99, file);
    else
        ungetc(c, file);

    count = fscanf(file, " %d %d %d ", &width, &height, &maxval);
    if (count != 3)
    {
        fprintf(stderr, "Cannot read mask file, header format error.\n");
        exit(EXIT_FAILURE);
    }

    if (width != 640 || height != 360)
    {
        fprintf(stderr, "Wrong size mask file: %d %d\n", width, height);
        exit(EXIT_FAILURE);
    }

    if (maxval != 255)
    {
        fprintf(stderr, "Wrong max pixel value: %d\n", maxval);
        exit(EXIT_FAILURE);
    }

    for (int i = 0; i < 640 * 360; ++i)
    {
        int r = fgetc(file);
        int g = fgetc(file);
        int b = fgetc(file);

        if (r > 250 && g < 10 && b < 10)
            maskFrame[i] = 255;
        else
            maskFrame[i] = noise_level;
    }
}

static void ProcessJpegMask(void)
{
    int frame_size = 1920 * 1080 / 9;

    if ( maskFrame == 0 )
        maskFrame = (unsigned char *)malloc(frame_size);

    memset( maskFrame, noise_level, frame_size );

    if (outputBuffer == 0)
    {
        fprintf( stderr, "JPEG Mask image conversion failed");
        return;
    }

    OMX_BUFFERHEADERTYPE *workingBuffer = outputBuffer;
    outputBuffer = 0;
    
    unsigned char *pMask = maskFrame;

    for (int iy = 1; iy < 1080; iy+=3)
    {
        for (int ix = 1; ix < 1920; ix+=3)
        {
            int indexY = 1920 * iy + ix;
            int indexU = 1920 * 1088 + 960 * (iy / 2) + (ix / 2);
            int indexV = indexU + 1920 * 1088 / 4;

            int y = workingBuffer->pBuffer[indexY];
            int u = workingBuffer->pBuffer[indexU];
            int v = workingBuffer->pBuffer[indexV];

            int r = y + (1.370705 * (v-128));
            int g = y - (0.698001 * (v-128)) - (0.337633 * (u-128));
            int b = y + (1.732446 * (u-128));

            *pMask = noise_level;
            if ( r > 240 && g < 30 && b < 30 )
                *pMask = 255;

            ++pMask;
        }
    }
}

static void ProcessComposeBuffer(void)
{
    if (outputBuffer == 0)
        return;

    OMX_ERRORTYPE error;

    OMX_BUFFERHEADERTYPE *workingBuffer = outputBuffer;
    outputBuffer = 0;

    error = OMX_FillThisBuffer(decoderHandle,
                               (workingBuffer == pingBuffer) ? pongBuffer : pingBuffer);

    if (error != OMX_ErrorNone)
        omx_die( error, "Could not Fill This Buffer");

    for (int iy = 0; iy < 1080; ++iy)
    {
        for (int ix = 0; ix < 1920; ++ix)
        {
            int indexY = 1920 * iy + ix;
            int indexU = 1920 * 1088 + 960 * (iy / 2) + (ix / 2);
            int indexV = indexU + 1920 * 1088 / 4;

            if ((const char)workingBuffer->pBuffer[indexY] > composeBuffer[indexY])
            {
                composeBuffer[indexY] = workingBuffer->pBuffer[indexY];
                composeBuffer[indexU] = workingBuffer->pBuffer[indexU];
                composeBuffer[indexV] = workingBuffer->pBuffer[indexV];
            }
        }
    }
}

static void OverlayMask(void)
{
    if (maskFrame == 0)
        readMask();

    for (int iy = 0; iy < 1080; ++iy)
    {
        for (int ix = 0; ix < 1920; ++ix)
        {
            int indexY = 1920 * iy + ix;
            int indexU = 1920 * 1088 + 960 * (iy / 2) + (ix / 2);
            int indexV = indexU + 1920 * 1088 / 4;

            int indexMask = 640*(iy/3) + ix/3;

            if ( maskFrame[indexMask] == 255 )
            {
                composeBuffer[indexU] = 116;
                composeBuffer[indexV] = 140;
            }
        }
    }
}

static int ProcessAverageBuffer( int doSubtractInstead )
{
    if (outputBuffer == 0)
        return 0;

    OMX_ERRORTYPE error;

    OMX_BUFFERHEADERTYPE *workingBuffer = outputBuffer;
    outputBuffer = 0;

    error = OMX_FillThisBuffer(decoderHandle,
                               (workingBuffer == pingBuffer) ? pongBuffer : pingBuffer);

    if (error != OMX_ErrorNone)
        omx_die( error, "Could not Fill This Buffer");

    if ( doSubtractInstead )
    {
        for (int iy = 0; iy < 1080; ++iy)
        {
            for (int ix = 0; ix < 1920; ++ix)
            {
                int indexY = 1920 * iy + ix;

                averageBuffer[indexY] -= workingBuffer->pBuffer[indexY];
            }
        }
    }
    else
    {
        for (int iy = 0; iy < 1080; ++iy)
        {
            for (int ix = 0; ix < 1920; ++ix)
            {
                int indexY = 1920 * iy + ix;

                averageBuffer[indexY] += workingBuffer->pBuffer[indexY];
            }
        }
    }

    return 1;
}

void ReadCalibrationParameters( void )
{
    char str[1024];
    char* pEnd;

    FILE *file = fopen("calibration.json", "r");
    if ( file == 0 )
        return;

    int count = fread(str,1,1023,file);

    fclose( file );

    calibrationParameters.V     =     0.002278;
    calibrationParameters.S     =     0.639837;
    calibrationParameters.D     =    -0.000982;
    calibrationParameters.a0    =  -103.264;
    calibrationParameters.E     =   253.174;
    calibrationParameters.eps   =     1.053;
    calibrationParameters.COPx  =   986.444;
    calibrationParameters.COPy  =   539.240;
    calibrationParameters.alpha =     1.570796;
    calibrationParameters.flat  =     0.000000;

    char* token = strtok(str," :\"\n\t\r");

    if ( strcmp(token,"{") )
        return;

    for (;;)
    {
        token = strtok(NULL," :\"\n\t\r");

        if ( !strcmp(token,"}"))
            break;

        if ( !strcmp(token,"V") )
        {
            token = strtok(NULL," :\"\n\t\r");
            calibrationParameters.V = strtod(token,&pEnd);
        }
        else if ( !strcmp(token,"S") )
        {
            token = strtok(NULL," :\"\n\t\r");
            calibrationParameters.S = strtod(token,&pEnd);
        }
        else if ( !strcmp(token,"D") )
        {
            token = strtok(NULL," :\"\n\t\r");
            calibrationParameters.D = strtod(token,&pEnd);
        }
        else if ( !strcmp(token,"a0") )
        {
            token = strtok(NULL," :\"\n\t\r");
            calibrationParameters.a0 = strtod(token,&pEnd);
        }
        else if ( !strcmp(token,"E") )
        {
            token = strtok(NULL," :\"\n\t\r");
            calibrationParameters.E = strtod(token,&pEnd);
        }
        else if ( !strcmp(token,"eps") )
        {
            token = strtok(NULL," :\"\n\t\r");
            calibrationParameters.eps = strtod(token,&pEnd);
        }
        else if ( !strcmp(token,"COPx") )
        {
            token = strtok(NULL," :\"\n\t\r");
            calibrationParameters.COPx = strtod(token,&pEnd);
        }
        else if ( !strcmp(token,"COPy") )
        {
            token = strtok(NULL," :\"\n\t\r");
            calibrationParameters.COPy = strtod(token,&pEnd);
        }
        else if ( !strcmp(token,"alpha") )
        {
            token = strtok(NULL," :\"\n\t\r");
            calibrationParameters.alpha = strtod(token,&pEnd);
        }
        else if ( !strcmp(token,"flat") )
        {
            token = strtok(NULL," :\"\n\t\r");
            calibrationParameters.flat = strtod(token,&pEnd);
        }
        else
            break;
    }

    double alpha = calibrationParameters.alpha;
    double flat  = calibrationParameters.flat;
    double COPx  = calibrationParameters.COPx;
    double COPy  = calibrationParameters.COPy;

    double dilation = sqrt(1.0-flat);
    double K = COPx*sin(alpha) + COPy*cos(alpha);
    double L = COPy*sin(alpha) - COPx*cos(alpha);

    calibrationParameters.c = cos(alpha)*cos(alpha)*dilation + sin(alpha)*sin(alpha)/dilation;
    calibrationParameters.d = sin(alpha)*cos(alpha)*dilation - sin(alpha)*cos(alpha)/dilation;
    calibrationParameters.e = -(K*cos(alpha)*dilation*dilation - COPy*dilation + L*sin(alpha))/dilation;
    calibrationParameters.f = sin(alpha)*cos(alpha)*dilation - sin(alpha)*cos(alpha)/ dilation;
    calibrationParameters.g = sin(alpha)*sin(alpha)*dilation + cos(alpha)*cos(alpha)/dilation;
    calibrationParameters.h = -(K*sin(alpha)*dilation*dilation - COPx*dilation - L*cos(alpha))/dilation;
}

void CalibrationFunction( void )
{
    double V     = calibrationParameters.V;
    double S     = calibrationParameters.S;
    double D     = calibrationParameters.D;
    double a0    = calibrationParameters.a0  * M_PI / 180.0;
    double E     = calibrationParameters.E   * M_PI / 180.0;
    double eps   = calibrationParameters.eps * M_PI / 180.0;
    double COPx  = calibrationParameters.COPx;
    double COPy  = calibrationParameters.COPy;

    double c = calibrationParameters.c;
    double d = calibrationParameters.d;
    double e = calibrationParameters.e;
    double f = calibrationParameters.f;
    double g = calibrationParameters.g;
    double h = calibrationParameters.h;

    double px = calibrationParameters.px;
    double py = calibrationParameters.py;
    
    // Apply the affine transformation to the input coordinates
    // PURPOSE: Correct elliptical image distortion

    double pxt = g*px + f*py + h;
    double pyt = d*px + c*py + e;

    double x = pxt - COPx;
    double y = pyt - COPy;

    double r = sqrt( x*x + y*y );
    double u = V*r + S*(exp(D*r) - 1);
    double b = a0 - E + atan2(x, y);

    double angle = b;
    double z = u;

    if ( eps != 0.0 )
    {
        z = acos(cos(u)*cos(eps)-sin(u)*sin(eps)*cos(b));
        double sinAngle = sin(b)*sin(u)/sin(z);
        double cosAngle = (cos(u)-cos(eps)*cos(z))/(sin(eps)*sin(z));
        angle = atan2(sinAngle,cosAngle);
    }

    double elev = M_PI/2 - z;
    double azim = angle + E - M_PI; // Measured from cardinal NORTH.

    azim *= 180.0 / M_PI;
    elev *= 180.0 / M_PI;

    while ( azim > 180.0 )
        azim -= 360.0;

    while ( azim <= -180.0 )
        azim += 360.0;

    calibrationParameters.azim = azim;
    calibrationParameters.elev = elev;
}

static int ProcessAnalyzeBuffer( FILE* textFile, FILE* csvFile, double eventTime )
{
    struct tm * ptm;

    if (outputBuffer == 0)
        return 0;

    OMX_ERRORTYPE error;

    OMX_BUFFERHEADERTYPE *workingBuffer = outputBuffer;
    outputBuffer = 0;

    error = OMX_FillThisBuffer(decoderHandle,
                               (workingBuffer == pingBuffer) ? pongBuffer : pingBuffer);

    if (error != OMX_ErrorNone)
        omx_die( error, "Could not Fill This Buffer");

    int sum = 0;
    double sumx = 0.0;
    double sumy = 0.0;
    int tcount = 0;

    for (int iy = 0; iy < 1080; ++iy)
    {
        for (int ix = 0; ix < 1920; ++ix)
        {
            int index = 1920*iy + ix;

            int test = workingBuffer->pBuffer[index] - averageBuffer[index];
            if ( test > 0 )
            {
                sum += test;
                sumx += ix * test;
                sumy += iy * test;
                ++tcount;
            }
        }
    }

    double px = sum == 0 ? 0 : sumx/sum;
    double py = sum == 0 ? 0 : sumy/sum;

    calibrationParameters.px = px;
    calibrationParameters.py = py;

    CalibrationFunction();

    double azim = sum == 0 ? 0 : calibrationParameters.azim;
    double elev = sum == 0 ? 0 : calibrationParameters.elev;

    int    txtCount;
    double timeOffset; 
    fscanf( textFile, "%d %lf %*d %*d %*f %*f", &txtCount, &timeOffset );

    double ftime = eventTime+timeOffset;
    time_t dtime = ftime;

    int millisec = (ftime-dtime)*1000;

    ptm = localtime( &dtime );

    fprintf( csvFile, "%04d%02d%02d_%02d%02d%02d_%03d,%6d,%9d,%10.1f,%10.1f,%10.2f,%10.2f\n", 
                      ptm->tm_year+1900, ptm->tm_mon, ptm->tm_mday, ptm->tm_hour, ptm->tm_min, ptm->tm_sec, millisec,
                      tcount, sum, px, py, azim, elev );

    return 1;
}

static void composeLoop(unsigned char* path)
{
    OMX_ERRORTYPE error = OMX_ErrorNone;
    char buffer[4096];
    FILE* file;

    file = fopen(path, "rb");
    if (file == 0)
        return;

    outputBuffer = 0;

    int count = fread(buffer, 1, 4096, file);

    decoderBuffer->nOffset = 0;
    decoderBuffer->nInputPortIndex = 130;

    while (count > 0)
    {
        ProcessComposeBuffer();

        memcpy(decoderBuffer->pBuffer, buffer, count);
        decoderBuffer->nFilledLen = count;
        decodeBufferFull = 1;

        error = OMX_EmptyThisBuffer(decoderHandle, decoderBuffer);
        if (error != OMX_ErrorNone)
        {
            fprintf(stderr, "Cannot EmptyThisBuffer: %d, %s\n",
                    errno, strerror(errno));

            if (logfile != 0)
            {
                fflush(logfile);
                fclose(logfile);
            }

            exit(EXIT_FAILURE);
        }

        pthread_mutex_lock(&composeBufferThreadMutex);
        while (decodeBufferFull != 0)
            pthread_cond_wait(&composeBufferThreadCondition, &composeBufferThreadMutex);

        pthread_mutex_unlock(&composeBufferThreadMutex);

        count = fread(buffer, 1, 4096, file);
    }

    fclose(file);

    usleep(10000);
    ProcessComposeBuffer();
}

static void readJpegLoop(unsigned char* path)
{
    OMX_ERRORTYPE error = OMX_ErrorNone;
    FILE* file;

    file = fopen(path, "rb");
    if (file == 0)
        return;

    int bufferIndex = 0;
    outputBuffer = 0;

    portSettingsChanged = 0;

    for (;;)
    {
        OMX_BUFFERHEADERTYPE* pb = jpegBuffers[bufferIndex];
        bufferIndex = (bufferIndex+1)%3;

        pb->nOffset = 0;
        pb->nInputPortIndex = 320;

        int count = fread(pb->pBuffer, 1, pb->nAllocLen, file);
        if ( count == 0 )
            break;

        if ( feof( file ) )
            pb->nFlags = OMX_BUFFERFLAG_EOS;

        pb->nFilledLen = count;
        decodeBufferFull = 1;

        error = OMX_EmptyThisBuffer(decoderHandle, pb);
        if (error != OMX_ErrorNone)
            omx_die( error, "Cannot Empty This Buffer");

        pthread_mutex_lock(&composeBufferThreadMutex);
        while (decodeBufferFull != 0)
            pthread_cond_wait(&composeBufferThreadCondition, &composeBufferThreadMutex);

        pthread_mutex_unlock(&composeBufferThreadMutex);

        if ( pingBuffer == 0 )
        {
            usleep(100000);
            if ( portSettingsChanged )
                init_jpeg_output(decoderHandle, 321 );
        }
    }

    fclose(file);

    usleep(100000);
    ProcessJpegMask();
}

static void analyzeLoop1( const char* path )
{
    OMX_ERRORTYPE error = OMX_ErrorNone;
    char buffer[4096];

    FILE *file = fopen(path, "rb");
    if (file == 0)
        return;

    outputBuffer = 0;

    int count = fread(buffer, 1, 4096, file);

    decoderBuffer->nOffset = 0;
    decoderBuffer->nInputPortIndex = 130;

    int frameCount = 0;

    while (count > 0)
    {
        frameCount += ProcessAverageBuffer( 0 );
        if ( frameCount >= 30 )
            break;

        memcpy(decoderBuffer->pBuffer, buffer, count);
        decoderBuffer->nFilledLen = count;
        decodeBufferFull = 1;

        error = OMX_EmptyThisBuffer(decoderHandle, decoderBuffer);
        if (error != OMX_ErrorNone)
        {
            fprintf(stderr, "Cannot EmptyThisBuffer: %d, %s\n",
                    errno, strerror(errno));

            if (logfile != 0)
            {
                fflush(logfile);
                fclose(logfile);
            }

            exit(EXIT_FAILURE);
        }

        pthread_mutex_lock(&composeBufferThreadMutex);
        while (decodeBufferFull != 0)
            pthread_cond_wait(&composeBufferThreadCondition, &composeBufferThreadMutex);

        pthread_mutex_unlock(&composeBufferThreadMutex);

        count = fread(buffer, 1, 4096, file);
    }

    if (frameCount == 0)
        return;

    readMask();

    for ( int iy = 0; iy < 1080; ++iy )
    {
        for ( int ix = 0; ix < 1920; ++ix )
        {
            int index = 1920*iy + ix;
            int indexMask = 640*(iy/3)+(ix/3);

            averageBuffer[index] /= frameCount;
            averageBuffer[index] += maskFrame[indexMask];
        }
    }

    fclose(file);
}

static void analyzeLoop2( const char* path )
{
    OMX_ERRORTYPE error = OMX_ErrorNone;
    char buffer[4096];
    char tpath[255]; 

    struct tm tmv;

    strcpy(tpath,path);

    int year;
    int month;
    int day;
    int hour;
    int minute;
    int second;
    int millisec = 0;
    if ( strlen(basename(tpath)) == 20 )
        sscanf(basename(tpath),"s%4d%2d%2d_%2d%2d%2d",&year,&month,&day,&hour,&minute,&second);
    else
        sscanf(basename(tpath),"s%4d%2d%2d_%2d%2d%2d_%3d",&year,&month,&day,&hour,&minute,&second,&millisec);

    fprintf( stderr, "%04d%02d%02d_%02d%02d%02d_%03d\n", year, month, day, hour, minute, second, millisec );

    tmv.tm_year = year-1900;
    tmv.tm_mon  = month;
    tmv.tm_mday = day;
    tmv.tm_hour = hour;
    tmv.tm_min  = minute;
    tmv.tm_sec  = second;

    time_t eventTime = mktime(&tmv);
    double ftime = eventTime + millisec/1000.0;

    FILE *file = fopen(tpath, "rb");
    if (file == 0)
        return;

    char *pDot = strrchr(tpath, '.');
    if (pDot)
        strcpy(pDot, ".txt");
    else
        strcat(tpath, ".txt");

    FILE *textFile = fopen(tpath,"r");
    if (textFile == 0)
        return;

    strcpy(tpath,path);
    pDot = strrchr(tpath, '.');
    if (pDot)
        strcpy(pDot, ".csv");
    else
        strcat(tpath, ".csv");

    FILE* csvFile = fopen(tpath,"w");
    if ( csvFile == 0)
        return;

    int count = fread(buffer, 1, 4096, file);

    int frameCount = 0;

    decoderBuffer->nOffset = 0;
    decoderBuffer->nInputPortIndex = 130;

    ReadCalibrationParameters();

    while (count > 0)
    {
        frameCount += ProcessAnalyzeBuffer( textFile, csvFile, ftime );

        memcpy(decoderBuffer->pBuffer, buffer, count);
        decoderBuffer->nFilledLen = count;
        decodeBufferFull = 1;

        error = OMX_EmptyThisBuffer(decoderHandle, decoderBuffer);
        if (error != OMX_ErrorNone)
        {
            fprintf(stderr, "Cannot EmptyThisBuffer: %d, %s\n",
                    errno, strerror(errno));

            if (logfile != 0)
            {
                fflush(logfile);
                fclose(logfile);
            }

            exit(EXIT_FAILURE);
        }

        pthread_mutex_lock(&composeBufferThreadMutex);
        while (decodeBufferFull != 0)
            pthread_cond_wait(&composeBufferThreadCondition, &composeBufferThreadMutex);

        pthread_mutex_unlock(&composeBufferThreadMutex);

        count = fread(buffer, 1, 4096, file);
    }

    fclose(file);
    fclose(textFile);
    fclose(csvFile);
}

static void averagerLoop( const char* path )
{
    OMX_ERRORTYPE error = OMX_ErrorNone;
    char buffer[4096];

    FILE *file = fopen(path, "rb");
    if (file == 0)
        return;

    outputBuffer = 0;
    averageBuffer = (int *)malloc(sizeof(int) * 1920 * 1088 * 3 / 2);

    memset(averageBuffer, 0, sizeof(int) * 1920 * 1088 * 3 / 2);
    memset(composeBuffer+1920*1088, 128, 1920 * 1088 / 2);

    int count = fread(buffer, 1, 4096, file);

    decoderBuffer->nOffset = 0;
    decoderBuffer->nInputPortIndex = 130;

    int frameCount = 0;

    int halfFrameCount = (starMovieFrameCount-10) / 2;
    int fullFrameCount = 2 * halfFrameCount;

    fprintf(stderr,"Full: %d Half: %d\n", fullFrameCount, halfFrameCount );

    while (count > 0)
    {
        if ( fullFrameCount != 0 && frameCount >= fullFrameCount )
            break;

        int doSubtractInstead = halfFrameCount != 0 && frameCount >= halfFrameCount;

        frameCount += ProcessAverageBuffer( doSubtractInstead );

        memcpy(decoderBuffer->pBuffer, buffer, count);
        decoderBuffer->nFilledLen = count;
        decodeBufferFull = 1;

        error = OMX_EmptyThisBuffer(decoderHandle, decoderBuffer);
        if (error != OMX_ErrorNone)
        {
            fprintf(stderr, "Cannot EmptyThisBuffer: %d, %s\n",
                    errno, strerror(errno));

            if (logfile != 0)
            {
                fflush(logfile);
                fclose(logfile);
            }

            exit(EXIT_FAILURE);
        }

        pthread_mutex_lock(&composeBufferThreadMutex);
        while (decodeBufferFull != 0)
            pthread_cond_wait(&composeBufferThreadCondition, &composeBufferThreadMutex);

        pthread_mutex_unlock(&composeBufferThreadMutex);

        count = fread(buffer, 1, 4096, file);
    }

    fclose(file);

    fprintf(stderr,"frameCount: %d\n", frameCount);

    if ( starMovieFrameCount != 0 )
    {
        for (int i = 0; frameCount != 0 && i < 1920 * 1080; ++i)
        {
            double scaled = 200.0 * averageBuffer[i]/frameCount;
            if ( scaled < 0.0 )
                scaled = 0.0;

            composeBuffer[i] = (scaled > 250.0) ? 250 : scaled;
        }
    }
    else
    {
        for (int i = 0; frameCount != 0 && i < 1920 * 1080; ++i)
        {
            int scaled = averageBuffer[i];
            scaled *= 20;
            scaled /= frameCount;
            composeBuffer[i] = (scaled > 250) ? 250 : scaled;
        }
    }

    free(averageBuffer);
}

static void EncodeLoop(const char* path)
{
    OMX_ERRORTYPE error = OMX_ErrorNone;
    outputBuffer = 0;

    memcpy( encoderBuffer->pBuffer, composeBuffer, encoderBuffer->nAllocLen );
    encoderBuffer->nFilledLen = encoderBuffer->nAllocLen;

    error = OMX_EmptyThisBuffer( encoderHandle, encoderBuffer );
    if ( error != OMX_ErrorNone )
        omx_die( error, "Cannot empty buffer" );

    error = OMX_FillThisBuffer( encoderHandle, imageBuffer );
    if ( error != OMX_ErrorNone )
        omx_die( error, "Cannot fill buffer" );

    sleep(1);
    if ( outputBuffer == 0 )
    {
        fprintf(stderr, "Encoder failer\n");
        return;
    }

    FILE* file = fopen(path, "wb");
    if ( file == 0 )
    {
        fprintf(stderr,"Could not open %s\n", path );
        return;
    }

    fprintf(stderr,"Image written to: %s\n", path );
    fwrite(outputBuffer->pBuffer, outputBuffer->nFilledLen, 1, file);
    fclose(file);

    outputBuffer = 0;
}

static void *runThread(void *args)
{
    running = 1;

    open_device();
    init_device(0);
    init_decoder();
    init_sentinel();
    readMask();
    start_capturing();
    decodeLoop();
    stop_capturing();
    uninit_sentinel();
    uninit_decoder();
    uninit_device();
    close_device();

    running = 0;

    return 0;
}

static void *maskThread(void *args)
{
    running = 1;

    init_jpeg_decoder();
    readJpegLoop("mask.jpg");
    uninit_jpeg_decoder();

    running = 0;
}

static void* composeThread(void *args)
{
    char path[255];

    running = 1;

    strncpy( path, (char *)args, 254 );

    composeBuffer = (char *)malloc(1920 * 1088 * 3 / 2);
    memset(composeBuffer, 0, 1920 * 1088 * 3 / 2);

    init_decoder();
    composeLoop( path );
    uninit_decoder();

    char *pDot = strrchr(path, '.');
    if (pDot)
        strcpy(pDot, ".jpg");
    else
        strcat(path, ".jpg");

    sleep(1);

    init_encoder();
    EncodeLoop( path );

    if (pDot)
        strcpy(pDot, "m.jpg");
    else
        strcat(path, "m.jpg");

    OverlayMask();
    EncodeLoop( path );
    uninit_encoder();

    free(composeBuffer);

    running = 0;
}

static void* averageThread(void *args)
{
    char path[255];

    running = 1;

    strncpy( path, (char *)args, 254 );

    composeBuffer = (char *)malloc(1920 * 1088 * 3 / 2);
    memset(composeBuffer, 0, 1920 * 1088 * 3 / 2);

    init_decoder();
    averagerLoop( path );
    uninit_decoder();

    char *pDot = strrchr(path, '.');
    if (pDot)
        strcpy(pDot, ".jpg");
    else
        strcat(path, ".jpg");

    sleep(1);

    init_encoder();
    EncodeLoop( path );
    uninit_encoder();

    free(composeBuffer);

    running = 0;
}

static void* analyzeThread(void *args)
{
    char path[255];

    running = 1;

    strncpy( path, (char *)args, 254 );

    averageBuffer = (int *)malloc(sizeof(int) * 1920 * 1088 * 3 / 2);
    memset(averageBuffer, 0, sizeof(int) * 1920 * 1088 * 3 / 2);

    init_decoder();
    analyzeLoop1( path );
    uninit_decoder();

    init_decoder();
    analyzeLoop2( path );
    uninit_decoder();

    free(averageBuffer);

    running = 0;
}

static void runInteractiveLoop( int mum )
{
    char cmd[100];

    fprintf(stderr, "Running Interactive\n");

    for (;;)
    {
        fflush(stdout);

        if ( !mum ) fprintf( stderr, "Cmd: ");

        fgets( cmd, 99, stdin);

        char* token = strtok(cmd," \n\t\r");
        if ( token == NULL )
            continue;

        if (!strcmp(token,"mum"))
        {
            mum = 1;
            printf( "=OK\n" );
            continue;
        }

        if (!strcmp(token,"quit"))
        {
            if ( running )
            {
                stop_flag = 1;
                printf( "Stopped\n");
            }

            printf( "Quit\n");
            return;
        }

        if (!strcmp(token,"get_running"))
        {
            if ( running )
            {
                printf( "=Yes\n" );
            } else {
                printf( "=No\n" );
            }
        }
        else if (!strcmp(token,"start"))
        {
            if ( running )
            {
                printf( "=No - Already Started\n" );
                continue;
            }

            frame_count = 0; // Run forever
            stop_flag = 0;

            int rc = pthread_create(&runThread_id, NULL, &runThread, NULL);
            rc = rc || pthread_detach(runThread_id);
            if (rc)
            {
                fprintf(stderr, "Error creating runThread_id\n");
                printf( "=No\n");
            } else {
                printf( "=OK\n" );
            }
        }
        else if (!strcmp(token,"stop"))
        {
            if ( !running )
            {
                printf( "=No - Already Stopped\n" );
                continue;
            }

            stop_flag = 1;
            printf( "=OK\n");
        }
        else if (!strcmp(token,"compose"))
        {
            if ( running )
            {
                printf( "=No\n" );
                continue;
            }

            token = strtok(NULL," \n\t\r");

            int rc = pthread_create(&composeThread_id, NULL, &composeThread, token );
            rc = rc || pthread_detach(composeThread_id);
            if ( rc != 0 )
            {
                fprintf(stderr, "Error creating composeThread_id\n");
                printf( "=No\n");
            } else {
                printf( "=OK\n" );
            }
        }
        else if (!strcmp(token,"average"))
        {
            if ( running )
            {
                printf( "=No\n" );
                continue;
            }

            token = strtok(NULL," \n\t\r");

            int rc = pthread_create(&averageThread_id, NULL, &averageThread, token );
            rc = rc || pthread_detach(averageThread_id);
            if ( rc != 0 )
            {
                fprintf(stderr, "Error creating averageThread_id\n");
                printf( "=No\n");
            } else {
                printf( "=OK\n" );
            }
        }
        else if (!strcmp(token,"analyze"))
        {
            if ( running )
            {
                printf( "=No\n" );
                continue;
            }

            token = strtok(NULL," \n\t\r");

            int rc = pthread_create(&analyzeThread_id, NULL, &analyzeThread, token );
            rc = rc || pthread_detach(analyzeThread_id);
            if ( rc != 0 )
            {
                fprintf(stderr, "Error creating analyzeThread_id\n");
                printf( "=No\n");
            } else {
                printf( "=OK\n" );
            }
        }
        else if (!strcmp(token,"mask"))
        {
            if ( running )
            {
                printf( "=No\n" );
                continue;
            }

            int rc = pthread_create(&maskThread_id, NULL, &maskThread, NULL );
            rc = rc || pthread_detach(maskThread_id);
            if ( rc != 0 )
            {
                fprintf(stderr, "Error creating maskThread_id\n");
                printf( "=No\n");
            } else {
                printf( "=OK\n" );
            }
        }
        else if (!strcmp(token,"set_star_movie_frame_count"))
        {
            token = strtok(NULL," \n\t\r");
            starMovieFrameCount = strtol(token, NULL, 0);
            printf( "=OK\n");
        }
        else if (!strcmp(token,"set_noise"))
        {
            token = strtok(NULL," \n\t\r");
            int test = strtol(token, NULL, 0);
            if ( test != 0 )
            {
                noise_level = test;
                printf( "=OK\n" );
            }
        }
        else if (!strcmp(token,"get_noise"))
        {
            printf( "=%d\n", noise_level);
        }
        else if (!strcmp(token,"set_sum_threshold"))
        {
            token = strtok(NULL," \n\t\r");
            int test = strtol(token, NULL, 0);
            if ( test != 0 )
            {
                sumThreshold = test;
                printf( "=OK\n" );
            }
        }
        else if (!strcmp(token,"get_sum_threshold"))
        {
            printf( "=%d\n", sumThreshold);
        }
        else if (!strcmp(token,"set_dev_name"))
        {
            token = strtok(NULL," \n\t\r");
            strcpy(dev_name, token);
            printf( "=OK\n");
        }
        else if (!strcmp(token,"get_dev_name"))
        {
            printf( "=%s\n", dev_name );
        }
        else if (!strcmp(token,"set_archive_path"))
        {
            token = strtok(NULL," \n\t\r");
            strcpy(archive_path, token);
            printf( "=OK\n");
        }
        else if (!strcmp(token,"get_archive_path"))
        {
            printf( "=%s\n", archive_path );
        }
        else if (!strcmp(token,"set_max_events_per_hour"))
        {
            token = strtok(NULL," \n\t\r");
            int test = strtol(token, NULL, 0);
            if ( test != 0 )
            {
                rateLimitEventsPerHour = test;
                printf( "=OK\n" );
            }
        }
        else if (!strcmp(token,"get_max_events_per_hour"))
        {
            printf("=%d\n", rateLimitEventsPerHour);
        }
        else if (!strcmp(token,"get_frame_rate"))
        {
            printf("=%5.2f\n", frameRate );
        }
        else if (!strcmp(token,"get_zenith_amplitude"))
        {
            printf("=%5.2f\n", zenithAmplitude );
        }
        else if (!strcmp(token,"force_trigger"))
        {
            force_count = frame_number + 30;
            printf( "=OK\n" );
        }
    }
}

static void usage(FILE *fp, int argc, char **argv)
{
    fprintf(fp,
            "Usage: %s [options]\n\n"
            "Version 1.3\n"
            "Options:\n"
            "-d | --device name   Video device name [%s]\n"
            "-j | --jpeg          Output single jpeg image\n"
            "-m | --mpeg          Get MPEG video\n"
            "-h | --help          Print this message\n"
            "-c | --count         Number of frames to process [%i] - use 0 for infinite\n"
            "-f | --force         Force a trigger after [%d] frames\n"
            "-n | --noise         Noise level [%d]\n"
            "-i | --interactive   Run interactive\n"
            "-s | --silent        Run silent interactive\n"
            "\n"
            "Example usage: ./sentinel.bin -j -d /dev/video0\n",
            argv[0], dev_name, h264_name, h264_name, frame_count, force_count, noise_level);
}

static const char short_options[] = "d:jmc:f:n:is";

static const struct option
    long_options[] = {
        {"device", required_argument, NULL, 'd'},
        {"jpeg", no_argument, NULL, 'j'},
        {"mpeg", no_argument, NULL, 'm'},
        {"help", no_argument, NULL, 'h'},
        {"count", required_argument, NULL, 'c'},
        {"force", required_argument, NULL, 'f'},
        {"noise", required_argument, NULL, 'n'},
        {"interactive", no_argument, NULL, 'i'},
        {"silent", no_argument, NULL, 's'},
        {0, 0, 0, 0}};

int main(int argc, char **argv)
{
    // mtrace();

    int getJpeg = 0;
    int getMpeg = 0;
    int composeH264 = 0;
    int averageH264 = 0;
    int runInteractive = 0;
    int silent = 0;

    strcpy(dev_name,"/dev/video2");
    strcpy( archive_path, "none");
    h264_name = "video.h264";

    for (;;)
    {
        int idx;
        int c;

        c = getopt_long(argc, argv,
                        short_options, long_options, &idx);

        if (-1 == c)
            break;

        switch (c)
        {
        case 0: /* getopt_long() flag */
            break;

        case 'd':
            strcpy(dev_name,optarg);
            break;

        case 'h':
            usage(stdout, argc, argv);
            exit(EXIT_SUCCESS);

        case 'j':
            strcpy(dev_name, "/dev/video0");
            getJpeg = 1;
            break;

        case 'm':
            strcpy(dev_name, "/dev/video0");
            getMpeg = 1;
            break;

        case 'c':
            errno = 0;
            frame_count = strtol(optarg, NULL, 0);
            if (errno)
                errno_exit(optarg);
            break;

        case 'f':
            force_count = strtol(optarg, NULL, 0);
            if (errno)
                errno_exit(optarg);
            break;

        case 'n':
            noise_level = strtol(optarg, NULL, 0);
            if (errno)
                errno_exit(optarg);
            break;

        case 'i':
            runInteractive = 1;
            break;

        case 's':
            runInteractive = 1;
            silent = 1;
            break;

        default:
            usage(stderr, argc, argv);
            exit(EXIT_FAILURE);
        }
    }

    if ( runInteractive )
    {
        bcm_host_init();

        runInteractiveLoop(silent);

        sleep(5);
        return 0;
    }

    if (getJpeg)
    {
        logfile = fopen("logfile.txt", "w");
        frame_count = 300;
        open_device();
        init_device(1);
        start_capturing();
        mainloop();
        stop_capturing();
        uninit_device();
        close_device();
        if (logfile != 0)
            fclose(logfile);

        return 0;
    }

    if (getMpeg)
    {
        logfile = fopen("logfile.txt", "w");
        open_device();
        init_device(1);
        start_capturing();
        mainloop();
        stop_capturing();
        uninit_device();
        close_device();
        if (logfile != 0)
            fclose(logfile);

        return 0;
    }

    logfile = fopen("logfile.txt", "w");
    bcm_host_init();

    int rc = pthread_create(&runThread_id, NULL, &runThread, NULL);
    if (rc)
    {
        fprintf(stderr, "Error creating runThread_id\n");
    }

    pthread_join(runThread_id,NULL);

    if (logfile != 0)
        fclose(logfile);

    // muntrace();

    return 0;
}
