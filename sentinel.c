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

#include <getopt.h>             /* getopt_long() */

#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
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

#ifndef V4L2_PIX_FMT_H264
#define V4L2_PIX_FMT_H264     v4l2_fourcc('H', '2', '6', '4') /* H264 with start codes */
#endif

struct buffer {
	void   *start;
	size_t length;
};

static OMX_HANDLETYPE decoderHandle;
static OMX_CALLBACKTYPE decoderCallbacks;
static OMX_BUFFERHEADERTYPE *decoderBuffer;
static OMX_BUFFERHEADERTYPE *pingBuffer;
static OMX_BUFFERHEADERTYPE *pongBuffer;
static OMX_BUFFERHEADERTYPE *outputBuffer;


static char            *dev_name;
static int fd = -1;
struct buffer          *buffers;
static unsigned int n_buffers;
static int out_buf;
//static int frame_count = 1;
static int frame_count = 300;
static int frame_number = 0;
static long toEpochOffset_us;
static unsigned char* testFrame;
static unsigned char* referenceFrame;
static unsigned char* maskFrame;
static FILE* logfile;

#define ALIGN(x, y) (((x) + ((y) - 1)) & ~((y) - 1))

static void errno_exit(const char *s)
{
	fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
	exit(EXIT_FAILURE);
}

static int xioctl(int fh, int request, void *arg)
{
	int r;

	do {
		r = ioctl(fh, request, arg);
	} while (-1 == r && EINTR == errno);

	return r;
}

static void process_image(const void *p, int size)
{
  char buffer[100];
  
  int status;
    frame_number++;

    if (out_buf==0)
    {
		sprintf( buffer, "video%03d.jpeg", frame_number );
        /* write to file */
        FILE *fp=fopen("video.raw","ab");
//      FILE *fp=fopen("video.jpeg","ab");
//      FILE *fp=fopen( buffer, "wb");
        fwrite(p, size, 1, fp);
        fflush(fp);
        fclose(fp);
    }
    else
    {
        /* write to stdout */
      status = write(1, p, size);
      if(status == -1)
	perror("write");
    }
}

static long long getEpochTimeShift()
{
	struct timeval epochtime;
	struct timespec  vsTime;

	gettimeofday(&epochtime, NULL);
	clock_gettime(CLOCK_MONOTONIC, &vsTime);

	long long uptime_us = (long long) vsTime.tv_sec * 1000000 + round( vsTime.tv_nsec/ 1000.0);
	long long epoch_us =  (long long) epochtime.tv_sec * 1000000  + epochtime.tv_usec;
	return epoch_us - uptime_us;
}

static int IsIDR( const void* p, int size )
{
	unsigned int word = 0xffffffff;
	int i;
	
	for ( i = 0; i < size; ++i )
	  {
		  unsigned int c = ((unsigned char *)p)[i];
		  if ( word == 1 )
		    {
				c &= 0x0f;
				
				if ( c == 1 )
				  return 0;
				  
				if ( c == 5 )
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

	if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf)) {
		switch (errno) {
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
	int isIDR = IsIDR( buffers[buf.index].start, buf.bytesused );
	
	/*
    printf( "%6d %6d %6d.%06d %6d %6d\n", 
            buf.sequence, buf.index, buf.timestamp.tv_sec, buf.timestamp.tv_usec,
            buf.bytesused, isIDR );
            */
            
	process_image(buffers[buf.index].start, buf.bytesused);

	if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
		errno_exit("VIDIOC_QBUF");

	return 1;
}



static void mainloop(void)
{
	unsigned int count;
	unsigned int loopIsInfinite = 0;

    if (frame_count == 0) loopIsInfinite = 1; //infinite loop
	count = frame_count;

    while ((count-- > 0) || loopIsInfinite) {
		for (;; ) {
			fd_set fds;
			struct timeval tv;
			int r;

			FD_ZERO(&fds);
			FD_SET(fd, &fds);

			/* Timeout. */
			tv.tv_sec = 2;
			tv.tv_usec = 0;

			r = select(fd + 1, &fds, NULL, NULL, &tv);

			if (-1 == r) {
				if (EINTR == errno)
					continue;
				errno_exit("select");
			}

			if (0 == r) {
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

	for (i = 0; i < n_buffers; ++i) {
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

	if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
		if (EINVAL == errno) {
			fprintf(stderr, "%s does not support "
			        "memory mapping\n", dev_name);
			exit(EXIT_FAILURE);
		} else {
			errno_exit("VIDIOC_REQBUFS");
		}
	}

	if (req.count < 2) {
		fprintf(stderr, "Insufficient buffer memory on %s\n",
		        dev_name);
		exit(EXIT_FAILURE);
	}

	buffers = calloc(req.count, sizeof(*buffers));

	if (!buffers) {
		fprintf(stderr, "Out of memory\n");
		exit(EXIT_FAILURE);
	}

	for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
		struct v4l2_buffer buf;

		CLEAR(buf);

		buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory      = V4L2_MEMORY_MMAP;
		buf.index       = n_buffers;

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

static void init_device(void)
{
	struct v4l2_capability cap;
	struct v4l2_cropcap cropcap;
	struct v4l2_crop crop;
	struct v4l2_format fmt;
	unsigned int min;

	if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap)) {
		if (EINVAL == errno) {
			fprintf(stderr, "%s is no V4L2 device\n",
			        dev_name);
			exit(EXIT_FAILURE);
		} else {
			errno_exit("VIDIOC_QUERYCAP");
		}
	}

	if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
		fprintf(stderr, "%s is no video capture device\n",
		        dev_name);
		exit(EXIT_FAILURE);
	}

	if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
		fprintf(stderr, "%s does not support streaming i/o\n",
			    dev_name);
		exit(EXIT_FAILURE);
	}

	/* Select video input, video standard and tune here. */


	CLEAR(cropcap);

	cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	if (0 == xioctl(fd, VIDIOC_CROPCAP, &cropcap)) {
		crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		crop.c = cropcap.defrect; /* reset to default */

		if (-1 == xioctl(fd, VIDIOC_S_CROP, &crop)) {
			switch (errno) {
			case EINVAL:
				/* Cropping not supported. */
				break;
			default:
				/* Errors ignored. */
				break;
			}
		}
	} else {
		/* Errors ignored. */
	}


	CLEAR(fmt);

	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	
    fmt.fmt.pix.width       = 1920;     
    fmt.fmt.pix.height      = 1080;  
 	fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_H264;
// 	fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    fmt.fmt.pix.field       = V4L2_FIELD_NONE;

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

	if (-1 == stat(dev_name, &st)) {
		fprintf(stderr, "Cannot identify '%s': %d, %s\n",
		        dev_name, errno, strerror(errno));
		exit(EXIT_FAILURE);
	}

	if (!S_ISCHR(st.st_mode)) {
		fprintf(stderr, "%s is no device\n", dev_name);
		exit(EXIT_FAILURE);
	}

	fd = open(dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);

	if (-1 == fd) {
		fprintf(stderr, "Cannot open '%s': %d, %s\n",
		        dev_name, errno, strerror(errno));
		exit(EXIT_FAILURE);
	}
}

static OMX_ERRORTYPE 
EventHandler ( OMX_OUT OMX_HANDLETYPE hComponent,
               OMX_OUT OMX_PTR pAppData,
               OMX_OUT OMX_EVENTTYPE eEvent,
               OMX_OUT OMX_U32 Data1,
               OMX_OUT OMX_U32 Data2,
               OMX_OUT OMX_PTR pEventData)
{
	printf( "EventHandler %d %x\n", eEvent, Data1 );
	return OMX_ErrorNone;
}

static int decode_frame(void)
{
	static int count = 0;
	
	struct timespec spec;
    struct tm * ptm;
    int            ms; // Milliseconds
    time_t          s;  // Seconds

	OMX_ERRORTYPE error;

	struct v4l2_buffer buf;
	unsigned int i;

	CLEAR(buf);

	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;

	if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf)) {
		switch (errno) {
		case EAGAIN:
			return 0;

		case EIO:
		/* Could ignore EIO, see spec. */

		/* fall through */

		default:
			errno_exit("VIDIOC_DQBUF");
		}
	}

	int isIDR = IsIDR( buffers[buf.index].start, buf.bytesused );
	
	long long temp_us = 1000000 * (long long) buf.timestamp.tv_sec + (long long) buf.timestamp.tv_usec;
    long long epochTimeStamp_us = temp_us + toEpochOffset_us ;

	if ( isIDR )
	{
		clock_gettime(CLOCK_REALTIME, &spec);
		s  = spec.tv_sec;
		ms = round(spec.tv_nsec / 1.0e6); // Convert nanoseconds to milliseconds
		if (ms > 999) 
		{
			s++;
			ms = 0;
		}
		
		ptm = gmtime ( &s );
		fprintf( logfile, "Time: %d.%06d %2d:%02d:%02d\n", 
		        buf.timestamp.tv_sec, buf.timestamp.tv_usec, 
		        ptm->tm_hour, ptm->tm_min, ptm->tm_sec );
	}
	
	assert(buf.index < n_buffers);

	decoderBuffer->pBuffer = buffers[buf.index].start;
	decoderBuffer->nFilledLen = buf.bytesused;
	decoderBuffer->nAllocLen = buf.length;
	decoderBuffer->nOffset = 0;
	decoderBuffer->nInputPortIndex = 130;
	decoderBuffer->nTimeStamp.nLowPart = epochTimeStamp_us & 0xffffffff;
	decoderBuffer->nTimeStamp.nHighPart = (int) (epochTimeStamp_us >> 32);
	
	fprintf( logfile, "->--- %u\n", decoderBuffer->nTimeStamp.nLowPart );
	error = OMX_EmptyThisBuffer( decoderHandle, decoderBuffer );

	if (error != OMX_ErrorNone) {
		fprintf(stderr, "Cannot EmptyThisBuffer: %d, %s\n",
		        errno, strerror(errno));
		exit(EXIT_FAILURE);
	}
	
	if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
		errno_exit("VIDIOC_QBUF");

	return 1;
}


static void ProcessDecodedBuffer(void)
{
	if ( outputBuffer == 0 )
		return;
		
	OMX_ERRORTYPE error;

	OMX_BUFFERHEADERTYPE* workingBuffer = outputBuffer;
	outputBuffer = 0;
	
	// printf( "--->- %d %d\n", workingBuffer->nTimeStamp.nLowPart, workingBuffer->nFilledLen );
 	
    error = OMX_FillThisBuffer( decoderHandle, 
                                (workingBuffer == pingBuffer) ?  pongBuffer : pingBuffer );
            
	if (error != OMX_ErrorNone) {
		fprintf(stderr, "FillThisBuffer failed: %d, %s\n",
		        errno, strerror(errno));
		exit(EXIT_FAILURE);
	}
	
	int sum = 0;
	int count = 0;
	
	unsigned char* pstart = (unsigned char*)workingBuffer->pBuffer;
	
	unsigned char* p     = pstart + 1920 + 1;
	unsigned char* pvend = pstart + 1920 * 1080;
	unsigned char* psend = pstart + 1920 + 1920;
	
	unsigned char* pTest = testFrame;
	unsigned char* pRef  = referenceFrame;
	unsigned char* pMask = maskFrame;
	
	clock_t t = clock();
	
	while ( p < pvend )
	{
		while ( p < psend )
		{
			*pTest++ = *p;
			int c = *p;
			
			int test = c - *pRef - *pMask++;
			if ( test > 0 )
			{
				++count;
				sum += c;
			}
				
			p += 3;
			++pRef;
		}
		
		p += 3840;
		psend += 5760;
	}
	
	pTest = testFrame;
	pRef  = referenceFrame;
	
	pvend = testFrame + 1920*1080/9;
	
	while ( pTest < pvend )
	{
		int n = *pRef;
		n *= 15;
		n += *pTest++;
		n >>= 4;
		
		*pRef++ = n;
	}
	
	t = clock()-t;
	double duration = (double)t/CLOCKS_PER_SEC;
					
	fprintf( logfile, "Sum Count Ref: %d %d %x %7.4f\n", sum, count, referenceFrame[100], duration );
}

static OMX_ERRORTYPE
FillBufferDone (OMX_HANDLETYPE omx_handle, OMX_PTR app_data, OMX_BUFFERHEADERTYPE *omx_buffer)
{
	if ( omx_buffer == 0 )
	    return OMX_ErrorNone;
	    
	if ( outputBuffer != 0 )
	{
		fprintf( stderr, "Buffer Overflow\n" );
		exit(EXIT_FAILURE);
	}
	
	outputBuffer = omx_buffer;
	
	return OMX_ErrorNone;
}

static OMX_ERRORTYPE
EmptyBufferDone (OMX_HANDLETYPE omx_handle, OMX_PTR app_data, OMX_BUFFERHEADERTYPE *omx_buffer)
{
	return OMX_ErrorNone;
}

static void decodeLoop(void)
{
	unsigned int count;
	count = frame_count;

    while ( count-- > 0 ) {
		for (;;) {
			fd_set fds;
			struct timeval tv;
			int r;

			FD_ZERO(&fds);
			FD_SET(fd, &fds);

			/* Timeout. */
			tv.tv_sec = 2;
			tv.tv_usec = 0;

			ProcessDecodedBuffer();
			
			r = select(fd + 1, &fds, NULL, NULL, &tv);

			ProcessDecodedBuffer();
			
			if (-1 == r) {
				if (EINTR == errno)
					continue;
				errno_exit("select");
			}

			if (0 == r) {
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
	
	OMX_Init();
	
	decoderCallbacks.EventHandler = EventHandler;
	decoderCallbacks.FillBufferDone = FillBufferDone;
	decoderCallbacks.EmptyBufferDone = EmptyBufferDone;
	
	error = OMX_GetHandle(&decoderHandle, 
	                      "OMX.broadcom.video_decode", 
	                      NULL,
	                      &decoderCallbacks );
	                      
	if (error != OMX_ErrorNone) {
		fprintf(stderr, "Cannot get decoder handle: %d, %s\n",
		        errno, strerror(errno));
		exit(EXIT_FAILURE);
	}

    memset(&format, 0, sizeof(OMX_VIDEO_PARAM_PORTFORMATTYPE));
    format.nSize = sizeof(OMX_VIDEO_PARAM_PORTFORMATTYPE);
    format.nVersion.nVersion = OMX_VERSION;
    format.nPortIndex = 130;
    format.eCompressionFormat = OMX_VIDEO_CodingAVC;
    
    error = OMX_SetParameter( decoderHandle, OMX_IndexParamVideoPortFormat, &format);

	paramPort.nSize = sizeof( OMX_PARAM_PORTDEFINITIONTYPE );
	paramPort.nVersion.nVersion = OMX_VERSION;
	paramPort.nPortIndex = 130;
    error = OMX_GetParameter( decoderHandle, OMX_IndexParamPortDefinition, &paramPort );
    
    paramPort.nBufferCountActual = paramPort.nBufferCountMin;
    error = OMX_SetParameter( decoderHandle, OMX_IndexParamPortDefinition, &paramPort );
    
    inputBufferSize = paramPort.nBufferSize;
    
	if (error != OMX_ErrorNone) {
		fprintf(stderr, "Cannot get parameter: %d, %s\n",
		        errno, strerror(errno));
		exit(EXIT_FAILURE);
	}
	
	paramPort.nPortIndex = 131;
    error = OMX_GetParameter( decoderHandle, OMX_IndexParamPortDefinition, &paramPort );
	
	if (error != OMX_ErrorNone) {
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
			                
	printf( "Buffer size: %d %d %d\n", paramPort.format.video.nStride,
	                                   paramPort.format.image.nSliceHeight,
	                                   paramPort.nBufferSize );
			                
    error = OMX_SetParameter( decoderHandle, OMX_IndexParamPortDefinition, &paramPort );
	
    error = OMX_SendCommand( decoderHandle, OMX_CommandStateSet, OMX_StateIdle, NULL);

    error = OMX_AllocateBuffer( decoderHandle, &decoderBuffer, 130, NULL, inputBufferSize );
    error = OMX_AllocateBuffer( decoderHandle, &pingBuffer, 131, NULL, paramPort.nBufferSize );
    error = OMX_AllocateBuffer( decoderHandle, &pongBuffer, 131, NULL, paramPort.nBufferSize );
    outputBuffer = 0;
    
    error = OMX_SendCommand ( decoderHandle, OMX_CommandPortDisable, 130, NULL);
    error = OMX_SendCommand ( decoderHandle, OMX_CommandPortEnable, 130, NULL);
    
    error = OMX_SendCommand ( decoderHandle, OMX_CommandPortDisable, 131, NULL);
    error = OMX_SendCommand ( decoderHandle, OMX_CommandPortEnable, 131, NULL);
    
    error = OMX_SendCommand ( decoderHandle, OMX_CommandStateSet, OMX_StateExecuting, NULL);
    
    error = OMX_FillThisBuffer( decoderHandle, pingBuffer );
    
    toEpochOffset_us = getEpochTimeShift();
}

static void init_sentinel(void)
{
	int frame_size = 1920 * 1080 / 9;
	
	testFrame = (unsigned char *) malloc( frame_size );
	referenceFrame = (unsigned char *) malloc( frame_size );
	maskFrame = (unsigned char *) malloc( frame_size );
	
	for ( int i = 0; i < frame_size; ++i )
	{
		testFrame[i] = 0;
		referenceFrame[i] = 255;
		maskFrame[i] = 40;
	}
}

static void uninit_decoder(void)
{
	printf( "uninit_decoder\n" );
	OMX_ERRORTYPE error = OMX_ErrorNone;
	
    error = error ? error : OMX_SendCommand( decoderHandle, OMX_CommandStateSet, OMX_StateIdle, NULL);
    
    error = error ? error : OMX_SendCommand ( decoderHandle, OMX_CommandPortDisable, 130, NULL);
    error = error ? error : OMX_SendCommand ( decoderHandle, OMX_CommandPortDisable, 131, NULL);
    
	error = error ? error : OMX_FreeBuffer(decoderHandle, 131, pingBuffer );
	error = error ? error : OMX_FreeBuffer(decoderHandle, 131, pongBuffer );
	error = error ? error : OMX_FreeHandle(decoderHandle);
	
	if (error != OMX_ErrorNone) {
		fprintf(stderr, "Cannot uninit decoder: %d, %s\n",
		        errno, strerror(errno));
		exit(EXIT_FAILURE);
	}
}

static void usage(FILE *fp, int argc, char **argv)
{
	fprintf(fp,
	        "Usage: %s [options]\n\n"
	        "Version 1.3\n"
	        "Options:\n"
	        "-d | --device name   Video device name [%s]\n"
	        "-h | --help          Print this message\n"
	        "-m | --mmap          Use memory mapped buffers [default]\n"
	        "-r | --read          Use read() calls\n"
	        "-u | --userp         Use application allocated buffers\n"
	        "-o | --output        Outputs stream to stdout\n"
                 "-f | --format        Force format to 640x480 YUYV\n"
		 "-F | --formatH264    Force format to 1920x1080 H264\n"
                 "-c | --count         Number of frames to grab [%i] - use 0 for infinite\n"
                 "\n"
		 "Example usage: capture -F -o -c 300 > output.raw\n"
		 "Captures 300 frames of H264 at 1920x1080 - use raw2mpg4 script to convert to mpg4\n",
	        argv[0], dev_name, frame_count);
}

static const char short_options[] = "d:hmruofFc:";

static const struct option
        long_options[] = {
	{ "device", required_argument, NULL, 'd' },
	{ "help",   no_argument,       NULL, 'h' },
	{ "mmap",   no_argument,       NULL, 'm' },
	{ "read",   no_argument,       NULL, 'r' },
	{ "userp",  no_argument,       NULL, 'u' },
	{ "output", no_argument,       NULL, 'o' },
	{ "format", no_argument,       NULL, 'f' },
	{ "formatH264", no_argument,   NULL, 'F' },
	{ "count",  required_argument, NULL, 'c' },
	{ 0, 0, 0, 0 }
};

int main(int argc, char **argv)
{
	logfile = fopen( "logfile.txt", "w" );
	dev_name = "/dev/video1";
//	dev_name = "/dev/video0";
    bcm_host_init();

	open_device();
	init_device();
	init_decoder();
	init_sentinel();
	start_capturing();
	FillBufferDone( decoderHandle, NULL, NULL );
	decodeLoop();
	// mainloop();
	stop_capturing();
	uninit_decoder();
	uninit_device();
	close_device();
	fclose( logfile );
	fprintf(stderr, "\n");
	return 0;
}
