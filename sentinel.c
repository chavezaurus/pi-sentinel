/*
 *  V4L2 video capture example
 *
 *  This program can be used and distributed without restrictions.
 *
 *      This program is provided with the V4L2 API
 * see http://linuxtv.org/docs.php for more information
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

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

#include <linux/videodev2.h>
#include "bcm_host.h"
#include "IL/OMX_Broadcom.h"

#define CLEAR(x) memset(&(x), 0, sizeof(x))
#define FRAMES_TO_KEEP 300

#ifndef V4L2_PIX_FMT_H264
#define V4L2_PIX_FMT_H264 v4l2_fourcc('H', '2', '6', '4') /* H264 with start codes */
#endif

struct buffer
{
    void *start;
    size_t length;
};

static OMX_HANDLETYPE decoderHandle;
static OMX_CALLBACKTYPE decoderCallbacks;
static OMX_BUFFERHEADERTYPE *decoderBuffer;
static OMX_BUFFERHEADERTYPE *pingBuffer;
static OMX_BUFFERHEADERTYPE *pongBuffer;
static OMX_BUFFERHEADERTYPE *outputBuffer;

static char *composeBuffer;
static short *averageBuffer;
static char *dev_name;
static char *h264_name;
static int fd = -1;
struct buffer *buffers;
static unsigned int n_buffers;
static int out_buf;
static int decodeBufferFull;
static int frame_count = 300;
static int frame_number = 0;
static int force_count = -1;
static int noise_level = 40;
static unsigned char *testFrame;
static unsigned char *referenceFrame;
static unsigned char *maskFrame;

static unsigned char *frameBuffers[FRAMES_TO_KEEP];
static int frameSizes[FRAMES_TO_KEEP];
static int frameTimestamps[FRAMES_TO_KEEP];
static int nextFrameIndex;
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
static int sumThreshold;
static int eventDuration;
static unsigned int eventTimeStamp_hi;
static unsigned int eventTimeStamp_lo;
static int rateLimitLastSecond;
static int rateLimitBank;
static int rateLimitEventsPerHour;

static FILE *videoFile;
static FILE *textFile;
static FILE *logfile = 0;

static pthread_mutex_t saveEventThreadMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t saveEventThreadCondition = PTHREAD_COND_INITIALIZER;
static pthread_t saveEventThread_id;

static pthread_mutex_t decodeBufferThreadMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t decodeBufferThreadCondition = PTHREAD_COND_INITIALIZER;
static pthread_t decodeBufferThread_id;

static pthread_t runThread_id;

static int thread_exit;
static int running = 0;

#define ALIGN(x, y) (((x) + ((y)-1)) & ~((y)-1))

struct ThreadArguments
{
    unsigned int eventStartTimeHi;
    unsigned int eventStartTimeLo;
    unsigned int eventStopTimeHi;
    unsigned int eventStopTimeLo;
    int jobPending;
} threadingJob;

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

    free(buffers);
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

    buffers = calloc(req.count, sizeof(*buffers));

    if (!buffers)
    {
        fprintf(stderr, "Out of memory\n");
        exit(EXIT_FAILURE);
    }

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

    fmt.fmt.pix.width = 1920;
    fmt.fmt.pix.height = 1080;
    fmt.fmt.pix.pixelformat = jpeg ? V4L2_PIX_FMT_MJPEG : V4L2_PIX_FMT_H264;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;

    if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
        errno_exit("VIDIOC_S_FMT");

    /* Buggy driver paranoia. */
    min = fmt.fmt.pix.width * 2;
    if (fmt.fmt.pix.bytesperline < min)
        fmt.fmt.pix.bytesperline = min;
    min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
    if (fmt.fmt.pix.sizeimage < min)
        fmt.fmt.pix.sizeimage = min;

    init_mmap();
}

static void close_device(void)
{
    if (-1 == close(fd))
        errno_exit("close");

    fd = -1;
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
    printf("EventHandler %d %x\n", eEvent, Data1);
    fflush(stdout);
    return OMX_ErrorNone;
}

static void keepFrames(const unsigned char *p, int size, unsigned int timeStamp_lo)
{
    frameBuffers[nextFrameIndex] = (unsigned char *)realloc(frameBuffers[nextFrameIndex], size);
    frameSizes[nextFrameIndex] = size;
    frameTimestamps[nextFrameIndex] = timeStamp_lo;

    memcpy(frameBuffers[nextFrameIndex], p, size);

    nextFrameIndex = (nextFrameIndex + 1) % FRAMES_TO_KEEP;
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

    keepFrames(buffers[buf.index].start, buf.bytesused, timeStamp_lo);
    if (buf.bytesused > decoderBuffer->nAllocLen)
    {
        fprintf(stderr, "Input frame too big: %d\n", buf.bytesused);
        exit(EXIT_FAILURE);
    }

    assert(buf.index < n_buffers);

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
        if (frameTimestamps[i] == arg.eventStartTimeLo)
            frameIndex = i;
        if (sumTimestamps[i] == arg.eventStopTimeLo)
            stopIndex = i;
    }

    if (frameIndex == -1 || sumIndex == -1 || stopIndex == -1)
        return;

    double timelong = arg.eventStartTimeHi * 4294.967296 + 1.0e-6 * arg.eventStartTimeLo + getEpochTimeShift();
    time_t unixTime = timelong;

    ptm = gmtime(&unixTime);
    sprintf(nameVideoFile, "events/s%04d%02d%02d_%02d%02d%02d.h264",
            ptm->tm_year + 1900, ptm->tm_mon + 1, ptm->tm_mday,
            ptm->tm_hour, ptm->tm_min, ptm->tm_sec);

    FILE *videoFile = fopen(nameVideoFile, "w");
    if (videoFile == 0)
        return;

    sprintf(nameTextFile, "events/s%04d%02d%02d_%02d%02d%02d.txt",
            ptm->tm_year + 1900, ptm->tm_mon + 1, ptm->tm_mday,
            ptm->tm_hour, ptm->tm_min, ptm->tm_sec);

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
    while (sumIndex != stopIndex)
    {
        double delta = 1.0e-6 * ((int)frameTimestamps[frameIndex] - (int)arg.eventStartTimeLo);
        int counts = sumCounts[sumIndex];
        int sum = sumBuffers[sumIndex];
        double xCentroid = (sum == 0) ? 0.0 : (double)xSumBuffers[sumIndex] / sum;
        double yCentroid = (sum == 0) ? 0.0 : (double)ySumBuffers[sumIndex] / sum;
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

    printf("Event: %04d/%02d/%02d %02d:%02d:%02d\n",
           ptm->tm_year + 1900, ptm->tm_mon + 1, ptm->tm_mday,
           ptm->tm_hour, ptm->tm_min, ptm->tm_sec);
    fflush(stdout);

    strcpy(pstr, "mp4");

    sprintf(nameSystemCmd, "MP4Box -add %s -fps 30 -quiet %s", nameVideoFile, nameMp4File);
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
        if (sum >= sumThreshold)
        {
            ++triggerCounter;
            if (triggerCounter >= 2)
                initiateTrigger(timeStamp_hi, timeStamp_lo);
        }
        else if (frame_number == force_count)
            initiateTrigger(timeStamp_hi, timeStamp_lo);
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

    decodeBufferFull = 0;

    // if ( logfile )
    // 	fprintf( logfile, "EmptyBufferDone %d\n", ++count );

    return OMX_ErrorNone;
}

static void decodeLoop(void)
{
    unsigned int count;
    unsigned int loopIsInfinite = 0;
    running = 1;

    if (frame_count == 0)
        loopIsInfinite = 1; //infinite loop
    count = frame_count;

    while (running && (count-- > 0 || loopIsInfinite))
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

            if (decode_frame())
                break;
            /* EAGAIN - continue select loop. */
        }
    }
}

static void init_decoder(void)
{
    OMX_ERRORTYPE error;
    OMX_PARAM_PORTDEFINITIONTYPE paramPort;
    OMX_VIDEO_PARAM_PORTFORMATTYPE format;

    int inputBufferSize;

    error = OMX_Init();
    if (error != OMX_ErrorNone)
    {
        fprintf(stderr, "Cannot init: %d, %s\n",
                errno, strerror(errno));
        exit(EXIT_FAILURE);
    }

    decoderCallbacks.EventHandler = EventHandler;
    decoderCallbacks.FillBufferDone = FillBufferDone;
    decoderCallbacks.EmptyBufferDone = EmptyBufferDone;

    error = OMX_GetHandle(&decoderHandle,
                          "OMX.broadcom.video_decode",
                          NULL,
                          &decoderCallbacks);

    if (error != OMX_ErrorNone)
    {
        fprintf(stderr, "Cannot get decoder handle: %d, %s\n",
                errno, strerror(errno));
        exit(EXIT_FAILURE);
    }

    memset(&format, 0, sizeof(OMX_VIDEO_PARAM_PORTFORMATTYPE));
    format.nSize = sizeof(OMX_VIDEO_PARAM_PORTFORMATTYPE);
    format.nVersion.nVersion = OMX_VERSION;
    format.nPortIndex = 130;
    format.eCompressionFormat = OMX_VIDEO_CodingAVC;

    error = OMX_SetParameter(decoderHandle, OMX_IndexParamVideoPortFormat, &format);
    if (error != OMX_ErrorNone)
    {
        fprintf(stderr, "Cannot set parameter: %d, %s\n",
                errno, strerror(errno));
        exit(EXIT_FAILURE);
    }

    paramPort.nSize = sizeof(OMX_PARAM_PORTDEFINITIONTYPE);
    paramPort.nVersion.nVersion = OMX_VERSION;
    paramPort.nPortIndex = 130;
    error = OMX_GetParameter(decoderHandle, OMX_IndexParamPortDefinition, &paramPort);
    if (error != OMX_ErrorNone)
    {
        fprintf(stderr, "Cannot get parameter: %d, %s\n",
                errno, strerror(errno));
        exit(EXIT_FAILURE);
    }

    paramPort.nBufferCountActual = paramPort.nBufferCountMin;
    paramPort.nBufferSize = 4147200;
    error = OMX_SetParameter(decoderHandle, OMX_IndexParamPortDefinition, &paramPort);
    if (error != OMX_ErrorNone)
    {
        fprintf(stderr, "Cannot set parameter: %d, %s\n",
                errno, strerror(errno));
        exit(EXIT_FAILURE);
    }

    inputBufferSize = paramPort.nBufferSize;

    paramPort.nPortIndex = 131;
    error = OMX_GetParameter(decoderHandle, OMX_IndexParamPortDefinition, &paramPort);
    if (error != OMX_ErrorNone)
    {
        fprintf(stderr, "Cannot get parameter: %d, %s\n",
                errno, strerror(errno));
        exit(EXIT_FAILURE);
    }

    paramPort.nBufferCountActual = 2;
    paramPort.format.video.nFrameWidth = 1920;
    paramPort.format.video.nFrameHeight = 1080;
    paramPort.format.video.nStride = ALIGN(paramPort.format.video.nFrameWidth, 32);
    paramPort.format.image.nSliceHeight = ALIGN(paramPort.format.video.nFrameHeight, 16);
    paramPort.nBufferSize = paramPort.format.image.nStride *
                            paramPort.format.image.nSliceHeight * 3 / 2;

    error = OMX_SetParameter(decoderHandle, OMX_IndexParamPortDefinition, &paramPort);
    if (error != OMX_ErrorNone)
    {
        fprintf(stderr, "Cannot set parameter: %d, %s\n",
                errno, strerror(errno));
        exit(EXIT_FAILURE);
    }

    // It appears that ports are already enabled so these are not needed
    // error = error || OMX_SendCommand ( decoderHandle, OMX_CommandPortEnable, 130, NULL);
    // error = error || OMX_SendCommand ( decoderHandle, OMX_CommandPortEnable, 131, NULL);
    error = error || OMX_SendCommand(decoderHandle, OMX_CommandStateSet, OMX_StateIdle, NULL);
    if (error != OMX_ErrorNone)
    {
        fprintf(stderr, "Cannot send command: %d, %s\n",
                errno, strerror(errno));
        exit(EXIT_FAILURE);
    }

    error = error || OMX_AllocateBuffer(decoderHandle, &decoderBuffer, 130, NULL, inputBufferSize);
    error = error || OMX_AllocateBuffer(decoderHandle, &pingBuffer, 131, NULL, paramPort.nBufferSize);
    error = error || OMX_AllocateBuffer(decoderHandle, &pongBuffer, 131, NULL, paramPort.nBufferSize);
    if (error != OMX_ErrorNone)
    {
        fprintf(stderr, "Cannot allocate buffers: %d, %s\n",
                errno, strerror(errno));
        exit(EXIT_FAILURE);
    }

    outputBuffer = 0;

    error = error || OMX_SendCommand(decoderHandle, OMX_CommandStateSet, OMX_StateExecuting, NULL);
    if (error != OMX_ErrorNone)
    {
        fprintf(stderr, "Cannot send commands: %d, %s\n",
                errno, strerror(errno));
        exit(EXIT_FAILURE);
    }

    error = OMX_FillThisBuffer(decoderHandle, pingBuffer);
    if (error != OMX_ErrorNone)
    {
        fprintf(stderr, "Cannot fill this buffer: %d, %s\n",
                errno, strerror(errno));
        exit(EXIT_FAILURE);
    }
}

static void init_sentinel(void)
{
    int frame_size = 1920 * 1080 / 9;

    testFrame = (unsigned char *)malloc(frame_size);
    referenceFrame = (unsigned char *)malloc(frame_size);
    maskFrame = (unsigned char *)malloc(frame_size);

    for (int i = 0; i < frame_size; ++i)
    {
        testFrame[i] = 0;
        referenceFrame[i] = 255;
        maskFrame[i] = 40;
    }

    for (int i = 0; i < FRAMES_TO_KEEP; ++i)
    {
        frameBuffers[i] = 0;
        frameSizes[i] = 0;
        sumBuffers[i] = 0;
        sumCounts[i] = 0;
        xSumBuffers[i] = 0;
        ySumBuffers[i] = 0;
    }

    nextFrameIndex = 0;
    eventFrameIndex = 0;
    eventFrameStop = 0;
    nextSumIndex = 0;
    eventSumIndex = 0;
    triggered = 0;
    untriggered = 0;
    triggerCounter = 0;
    untriggerCounter = 0;
    sumThreshold = 175;
    eventDuration = 0;
    videoFile = 0;
    textFile = 0;

    rateLimitLastSecond = 0;
    rateLimitEventsPerHour = 5;
    rateLimitBank = 3600;

    threadingJob.eventStartTimeHi = 0;
    threadingJob.eventStartTimeLo = 0;
    threadingJob.eventStopTimeHi = 0;
    threadingJob.eventStopTimeLo = 0;
    threadingJob.jobPending = 0;

    thread_exit = 0;

    int rc = pthread_create(&saveEventThread_id, NULL, &saveEventThread, NULL);
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

    free(testFrame);
    free(referenceFrame);
    free(maskFrame);
}

static void uninit_decoder(void)
{
    // printf( "uninit_decoder\n" );
    OMX_ERRORTYPE error = OMX_ErrorNone;

    error = error || OMX_SendCommand(decoderHandle, OMX_CommandStateSet, OMX_StateIdle, NULL);
    error = error || OMX_SendCommand(decoderHandle, OMX_CommandPortDisable, 130, NULL);
    error = error || OMX_SendCommand(decoderHandle, OMX_CommandPortDisable, 131, NULL);
    if (error != OMX_ErrorNone)
    {
        fprintf(stderr, "Cannot send commands: %d, %s\n",
                errno, strerror(errno));
        exit(EXIT_FAILURE);
    }

    error = error || OMX_FreeBuffer(decoderHandle, 130, decoderBuffer);
    error = error || OMX_FreeBuffer(decoderHandle, 131, pingBuffer);
    error = error || OMX_FreeBuffer(decoderHandle, 131, pongBuffer);
    if (error != OMX_ErrorNone)
    {
        fprintf(stderr, "Cannot free buffers: %d, %s\n",
                errno, strerror(errno));
        exit(EXIT_FAILURE);
    }

    error = OMX_FreeHandle(decoderHandle);

    if (error != OMX_ErrorNone)
    {
        fprintf(stderr, "Cannot free handle: %d, %s\n",
                errno, strerror(errno));
        exit(EXIT_FAILURE);
    }

    error = OMX_Deinit();
    if (error != OMX_ErrorNone)
    {
        fprintf(stderr, "Cannot uninit decoder: %d, %s\n",
                errno, strerror(errno));
        exit(EXIT_FAILURE);
    }
}

static void readMask(void)
{
    char buffer[100];

    FILE *file = fopen("mask.ppm", "rb");
    if (file == 0)
    {
        printf("No mask file found. Noise level set to: %d\n", noise_level);
        memset(maskFrame, noise_level, 640 * 360);
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
    {
        fprintf(stderr, "FillThisBuffer failed: %d, %s\n",
                errno, strerror(errno));
        exit(EXIT_FAILURE);
    }

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

static void composeLoop(void)
{
    OMX_ERRORTYPE error = OMX_ErrorNone;
    char buffer[4096];

    char yuvName[100];
    char jpgName[100];

    char cmd[1000];

    FILE *file = fopen(h264_name, "rb");
    if (file == 0)
        return;

    outputBuffer = 0;
    composeBuffer = (char *)malloc(1920 * 1088 * 3 / 2);
    memset(composeBuffer, 0, 1920 * 1088 * 3 / 2);

    int count = fread(buffer, 1, 4096, file);

    decoderBuffer->nOffset = 0;
    decoderBuffer->nInputPortIndex = 130;

    while (count > 0)
    {
        decodeBufferFull = 1;

        decoderBuffer->nFilledLen = count;
        memcpy(decoderBuffer->pBuffer, buffer, count);
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

        ProcessComposeBuffer();
        while (decodeBufferFull)
        {
            usleep(5000);
            ProcessComposeBuffer();
        }

        count = fread(buffer, 1, 4096, file);
    }

    fclose(file);

    strncpy(yuvName, h264_name, 99);
    strncpy(jpgName, h264_name, 99);

    char *pDot = strrchr(yuvName, '.');
    if (pDot)
        strcpy(pDot, ".yuv");
    else
        strcat(yuvName, ".yuv");

    pDot = strrchr(jpgName, '.');
    if (pDot)
        strcpy(pDot, ".jpg");
    else
        strcat(jpgName, ".jpg");

    sprintf(cmd, "ffmpeg -s 1920x1080 -pix_fmt yuv420p -i %s %s", yuvName, jpgName);

    FILE *fraw = fopen(yuvName, "wb");
    if (fraw == 0)
        return;

    fwrite(composeBuffer, 1, 1920 * 1080, fraw);
    fwrite(composeBuffer + 1920 * 1088, 1, 1920 * 1080 / 4, fraw);
    fwrite(composeBuffer + 1920 * 1088 + 1920 * 1088 / 4, 1, 1920 * 1080 / 4, fraw);
    fclose(fraw);
    free(composeBuffer);

    system(cmd);
}

static int ProcessAverageBuffer(void)
{
    if (outputBuffer == 0)
        return 0;

    OMX_ERRORTYPE error;

    OMX_BUFFERHEADERTYPE *workingBuffer = outputBuffer;
    outputBuffer = 0;

    error = OMX_FillThisBuffer(decoderHandle,
                               (workingBuffer == pingBuffer) ? pongBuffer : pingBuffer);

    if (error != OMX_ErrorNone)
    {
        fprintf(stderr, "FillThisBuffer failed: %d, %s\n",
                errno, strerror(errno));
        exit(EXIT_FAILURE);
    }

    for (int iy = 0; iy < 1080; ++iy)
    {
        for (int ix = 0; ix < 1920; ++ix)
        {
            int indexY = 1920 * iy + ix;

            averageBuffer[indexY] += workingBuffer->pBuffer[indexY];
        }
    }

    return 1;
}

static void averagerLoop(void)
{
    OMX_ERRORTYPE error = OMX_ErrorNone;
    const char *buffer[4096];

    FILE *file = fopen(h264_name, "rb");
    if (file == 0)
        return;

    outputBuffer = 0;
    averageBuffer = (short *)malloc(sizeof(short) * 1920 * 1088 * 3 / 2);
    composeBuffer = (char *)malloc(1920 * 1088 * 3 / 2);

    memset(averageBuffer, 0, sizeof(short) * 1920 * 1088 * 3 / 2);
    memset(composeBuffer, 128, 1920 * 1088 * 3 / 2);

    int count = fread(buffer, 1, 4096, file);

    decoderBuffer->nOffset = 0;
    decoderBuffer->nInputPortIndex = 130;

    int frameCount = 0;

    while (count > 0)
    {
        decodeBufferFull = 1;

        decoderBuffer->nFilledLen = count;
        memcpy(decoderBuffer->pBuffer, buffer, count);
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

        frameCount += ProcessAverageBuffer();
        while (decodeBufferFull)
        {
            usleep(5000);
            frameCount += ProcessAverageBuffer();
        }

        count = fread(buffer, 1, 4096, file);
    }

    fclose(file);

    FILE *fraw = fopen("averaged.yuv", "wb");
    if (fraw == 0)
        return;

    printf("Frames: %d\n", frameCount);

    for (int i = 0; frameCount != 0 && i < 1920 * 1080; ++i)
    {
        int scaled = 10 * averageBuffer[i] / frameCount;
        composeBuffer[i] = (scaled > 255) ? 255 : scaled;
    }

    fwrite(composeBuffer, 1, 1920 * 1080, fraw);
    fwrite(composeBuffer + 1920 * 1088, 1, 1920 * 1080 / 4, fraw);
    fwrite(composeBuffer + 1920 * 1088 + 1920 * 1088 / 4, 1, 1920 * 1080 / 4, fraw);
    fclose(fraw);
    free(averageBuffer);
    free(composeBuffer);
}

static void *runThread(void *args)
{
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

    return 0;
}

static void runInteractiveLoop()
{
    char cmd[100];

    printf( "Running Interactive\n");
    int mum = 0;

    for (;;)
    {
        fflush(stdout);

        if (!mum) printf( "Cmd: ");

        fgets( cmd, 99, stdin);

        char* token = strtok(cmd," \n\t\r");
        if ( token == NULL )
            continue;

        if (!strcmp(token,"quit"))
        {
            if ( running )
            {
                running = 0;
                pthread_join(runThread_id,NULL);
                if (!mum) printf( "Stopped\n");
            }

            if (!mum) printf( "Quit\n");
            return;
        }

        if (!strcmp(token,"get_running"))
        {
            if ( running )
            {
                if (!mum) printf( "=Yes\n");
            } else {
                if (!mum) printf( "=No\n");
            }
        }
        else if (!strcmp(token,"start"))
        {
            if ( running )
            {
                if (!mum) printf( "=No - Already Started\n");
                continue;
            }

            frame_count = 0; // Run forever

            int rc = pthread_create(&runThread_id, NULL, &runThread, NULL);
            if (rc)
            {
                fprintf(stderr, "Error creating runThread_id\n");
            } else {
                if (!mum) printf( "=OK\n" );
            }
        }
        else if (!strcmp(token,"stop"))
        {
            if ( !running )
            {
                if (!mum) printf( "=No - Already Stopped\n" );
                continue;
            }

            running = 0;
            pthread_join(runThread_id,NULL);
            if (!mum) printf( "=OK\n");
        }
        else if (!strcmp(token,"set_noise"))
        {
            token = strtok(NULL," \n\t\r");
            int test = strtol(token, NULL, 0);
            if ( test != 0 )
            {
                noise_level = test;
                if (!mum) printf( "=OK\n" );
            }
        }
        else if (!strcmp(token,"get_noise"))
        {
            if (!mum) printf( "=%d\n", noise_level);
        }
        else if (!strcmp(token,"mum"))
        {
            mum = !mum;
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
            "-p | --compose file  H264 file name [%s]\n"
            "-a | --average file  H264 file name [%s]\n"
            "-j | --jpeg          Output single jpeg image\n"
            "-m | --mpeg          Get MPEG video\n"
            "-h | --help          Print this message\n"
            "-c | --count         Number of frames to process [%i] - use 0 for infinite\n"
            "-f | --force         Force a trigger after [%d] frames\n"
            "-n | --noise         Noise level [%d]\n"
            "-i | --interactive   Run interactive\n"
            "\n"
            "Example usage: sentinel -j -d /dev/video0\n",
            argv[0], dev_name, h264_name, h264_name, frame_count, force_count, noise_level);
}

static const char short_options[] = "d:p:a:jmc:f:n:i";

static const struct option
    long_options[] = {
        {"device", required_argument, NULL, 'd'},
        {"compose", required_argument, NULL, 'p'},
        {"average", required_argument, NULL, 'a'},
        {"jpeg", no_argument, NULL, 'j'},
        {"mpeg", no_argument, NULL, 'm'},
        {"help", no_argument, NULL, 'h'},
        {"count", required_argument, NULL, 'c'},
        {"force", required_argument, NULL, 'f'},
        {"noise", required_argument, NULL, 'n'},
        {"interactive", no_argument, NULL, 'i'},
        {0, 0, 0, 0}};

int main(int argc, char **argv)
{
    int getJpeg = 0;
    int getMpeg = 0;
    int composeH264 = 0;
    int averageH264 = 0;
    int runInteractive = 0;

    dev_name = "/dev/video2";
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
            dev_name = optarg;
            break;

        case 'p':
            composeH264 = 1;
            h264_name = optarg;
            break;

        case 'a':
            averageH264 = 1;
            h264_name = optarg;
            break;

        case 'h':
            usage(stdout, argc, argv);
            exit(EXIT_SUCCESS);

        case 'j':
            dev_name = "/dev/video0";
            getJpeg = 1;
            break;

        case 'm':
            dev_name = "/dev/video0";
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

        default:
            usage(stderr, argc, argv);
            exit(EXIT_FAILURE);
        }
    }

    if ( runInteractive )
    {
        bcm_host_init();

        runInteractiveLoop();

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

    if (composeH264)
    {
        bcm_host_init();

        init_decoder();
        composeLoop();
        uninit_decoder();

        return 0;
    }

    if (averageH264)
    {
        bcm_host_init();

        init_decoder();
        averagerLoop();
        uninit_decoder();

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

    return 0;
}
