#include <chrono>
#include <iostream>
#include <fstream>
#include <sstream>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <poll.h>
#include <string.h>
#include <unistd.h>
#include <libv4l2.h>
#include <jpeglib.h>
#include <math.h>
#include <list>
#include <filesystem>

#include "pi-sentinel.hpp"

using namespace std::chrono_literals;

//
// Stripped down JSON parser, parses only vanilla dictionary of doubles
//
static void Parse( const char* json, map<string,double>& doubleMap )
{
    char key[100];
    char value[100];
    string stringValue;
    
    const char* ptr = json;
    
    while ( *ptr != 0 && isspace(*ptr) )
        ++ptr;
    
    if ( *ptr++ != '{' )
        return;
    
    for (;;)
    {
        while ( *ptr != 0 && isspace(*ptr) )
            ++ptr;
        
        if ( *ptr++ != '"' )
            return;
        
        char* pkey = key;
        while ( *ptr != 0 && *ptr != '"' )
            *pkey++ = *ptr++;

        *pkey = 0;
        
        if ( *ptr++ != '"' )
            return;
        
        while (*ptr != 0 && isspace(*ptr) )
            ++ptr;
        
        if ( *ptr++ != ':' )
            return;
        
        while (*ptr != 0 && isspace(*ptr) )
            ++ptr;
        
        int count = 0;
        char* pvalue = value;
        while ( *ptr=='.' || *ptr=='-' || *ptr=='+' || (*ptr >= '0' && *ptr <= '9' ) )
        {
            *pvalue++ = *ptr++;
            if ( ++count >= 90 )
            {
                *pvalue = 0;
                stringValue += value;
                pvalue = value;
            }
        }
        *pvalue = 0;
        stringValue += value;
        
		doubleMap[key] = std::stod(stringValue);
        stringValue = "";
        
        while (*ptr != 0 && isspace(*ptr) )
            ++ptr;
        
        if (*ptr++ != ',' )
            return;
    }
}

vector<SentinelCamera*> SentinelCamera::cameraVector;

SentinelCamera::SentinelCamera()
{
	running = false;
    id = cameraVector.size();
    cameraVector.push_back( this );
	max_frame_count = 0;
	checkBufferHead = 0;
	checkBufferTail = 0;
	noise_level = 50;
	moonx = 0;
	moony = 0;
	force_count = 0;
	sumThreshold = 175;
	max_events_per_hour = 10;
	force_event = false;

    cm = new CameraManager();
    cm->start();

    if ( cm->cameras().empty() )
    {
        cm->stop();
        throw std::runtime_error("No cameras were identified on the system");
    }

	std::string cameraId = cm->cameras()[0]->id();
	camera = cm->get(cameraId);
	camera->acquire();

    // Setup the VideoRecording stream to produce the encoded archive files
    // and the ViewFinder stream to produce the frames used for event triggering
    config = camera->generateConfiguration( { StreamRole::VideoRecording, StreamRole::Viewfinder } );

	StreamConfiguration &streamConfig1 = config->at(0);
	StreamConfiguration &streamConfig2 = config->at(1);

	streamConfig2.size.width = 640;
	streamConfig2.size.height = 360;
	streamConfig2.pixelFormat = libcamera::formats::YUV420;

	camera->configure(config.get());

	allocator = new FrameBufferAllocator(camera);

	for (StreamConfiguration &cfg : *config) 
    {
		int ret = allocator->allocate(cfg.stream());
		if ( ret < 0 ) 
            throw std::runtime_error("Can't allocate buffers");
	}

	Stream *stream1 = streamConfig1.stream();
	Stream *stream2 = streamConfig2.stream();

	const std::vector<std::unique_ptr<FrameBuffer>> &buffers1 = allocator->buffers(stream1);
	const std::vector<std::unique_ptr<FrameBuffer>> &buffers2 = allocator->buffers(stream2);

	for (unsigned int i = 0; i < buffers1.size(); ++i) 
    {
		std::unique_ptr<Request> request = camera->createRequest( id );
		if ( !request )
            throw std::runtime_error("Can't create request");

		const std::unique_ptr<FrameBuffer> &buffer1 = buffers1[i];
		int ret = request->addBuffer(stream1, buffer1.get());
		if ( ret < 0 )
            throw std::runtime_error("Can't set buffer for request");

		const std::unique_ptr<FrameBuffer> &buffer2 = buffers2[i];
		ret = request->addBuffer(stream2, buffer2.get());
		if ( ret < 0 )
            throw std::runtime_error("Can't set buffer for request");

        //
        // Set frame rate to 30/sec
        //
		ControlList &controls = request->controls();
		int64_t frame_time = 1000000 / 30; // in us
		controls.set(controls::FrameDurationLimits, { frame_time, frame_time });

		requests.push_back(std::move(request));
	}

    camera->requestCompleted.connect(requestComplete);
	syncTime();

	for ( int i = 0; i < NUM_CHECK_BUFFERS; ++i )
	{
		checkBuffers[i].mem = new unsigned char[CHECK_FRAME_SIZE];
		checkBuffers[i].timestamp_us = 0;
	}
}

SentinelCamera::~SentinelCamera()
{
	camera->stop();
    for ( StreamConfiguration &cfg : *config ) 
    {
        allocator->free(cfg.stream());
    }

	delete allocator;
	camera->release();
	camera.reset();
	cm->stop();

	for ( int i = 0; i < NUM_CHECK_BUFFERS; ++i )
		delete[] checkBuffers[i].mem;
}

void SentinelCamera::requestComplete(Request* request)
{
	if (request->status() == Request::RequestCancelled)
		return;

    SentinelCamera* sc = cameraVector[request->cookie()];

	sc->requestMutex.lock();
	sc->completedRequests.push_back(request);
	sc->requestMutex.unlock();

	sc->requestCondition.notify_one();

    // std::cout << "requestComplete" << std::endl;
}

void SentinelCamera::syncTime()
{
	struct timespec tspec1, tspec2, tspec3, tspec4;

	clock_gettime( CLOCK_MONOTONIC, &tspec1 );
	clock_gettime( CLOCK_REALTIME,  &tspec2 );
	clock_gettime( CLOCK_MONOTONIC, &tspec3 );

	if ( tspec3.tv_sec == tspec1.tv_sec && (tspec3.tv_nsec-tspec1.tv_nsec) < 100000 )
	{
		tspec4.tv_sec = tspec2.tv_sec - tspec1.tv_sec;
		tspec4.tv_nsec = tspec2.tv_nsec - tspec1.tv_nsec;
		if ( tspec4.tv_nsec < 0 )
		{
			tspec4.tv_nsec += 1000000000;
			tspec4.tv_sec -= 1;
		}

		time_offset_mutex.lock();
		time_offset = tspec4;
		time_offset_mutex.unlock();

		// std::cout << time_offset.tv_sec << " " << time_offset.tv_nsec << std::endl;
	}
}

void SentinelCamera::requestThread()
{

    abortRequestThread = false;
	int frame_count = 0;
	Request* request;

    for (;;)
    {
		++frame_count;
		if ( max_frame_count != 0 && frame_count > max_frame_count )
			abortRequestThread = true;

		if ( (frame_count % 60) == 0 )
			syncTime();

		{
			std::unique_lock<std::mutex> locker(requestMutex);
			requestCondition.wait_for( locker, 200ms, [this]() {
				return !this->completedRequests.empty() || abortRequestThread;
			});

        	if ( abortRequestThread )
				return;

			if ( completedRequests.empty() )
				continue;

			request = completedRequests.front();
			completedRequests.pop_front();
		}

	    const Request::BufferMap &buffers = request->buffers();

	    for (auto bufferPair : buffers) 
        {
		    const libcamera::Stream *stream = bufferPair.first;
		    std::string config = stream->configuration().toString();
		    FrameBuffer *buffer = bufferPair.second;
		    const FrameMetadata &metadata = buffer->metadata();
		    size_t frameSize = metadata.planes()[0].bytesused;

		    int fd = buffer->planes()[0].fd.fd();
		    int64_t timestamp_us = metadata.timestamp / 1000;

		    if ( config == "1920x1080-YUV420" )
		    	encodeBuffer( fd, frameSize*3/2, timestamp_us );
			else if ( config == "640x360-YUV420" )
				fillCheckBuffer( fd, timestamp_us );
	    }

	    // Re-queue the Request to the camera.
	    request->reuse(Request::ReuseBuffers);
	    camera->queueRequest(request);

		// std::cout << "requestThread" << std::endl;
    }
}

void SentinelCamera::fillCheckBuffer( int fd, int64_t timestamp_us )
{
	if ( mappedBuffers.find( fd ) == mappedBuffers.end() )
		mappedBuffers[ fd ] = mmap( NULL, CHECK_FRAME_SIZE, PROT_READ, MAP_SHARED, fd, 0 );
	void* mem = mappedBuffers[ fd ];

	CheckBufferDescription& desc = checkBuffers[checkBufferHead];
	memcpy( desc.mem, mem, CHECK_FRAME_SIZE );
	desc.timestamp_us = timestamp_us;

	check_mutex.lock();
	checkBufferHead = (checkBufferHead+1) % NUM_CHECK_BUFFERS;
	check_mutex.unlock();

	checkCondition.notify_one();
}

void SentinelCamera::start()
{
	if ( running )
		return;

	while ( !input_buffers_available.empty() )
		input_buffers_available.pop();
	createEncoder();

	completedRequests.clear();

    request_thread = thread( &SentinelCamera::requestThread, this );
    poll_thread = thread( &SentinelCamera::pollThread, this );
    output_thread = thread( &SentinelCamera::outputThread, this );
	check_thread = thread( &SentinelCamera::checkThread, this );
	event_thread = thread( &SentinelCamera::eventThread, this );

	camera->start();
	for (std::unique_ptr<Request> &request : requests)
	{
		request->reuse(Request::ReuseBuffers);
		camera->queueRequest(request.get());
	}

	running = true;
}

void SentinelCamera::initiateShutdown()
{
	requestMutex.lock();
    abortRequestThread = true;
	requestMutex.unlock();
}

void SentinelCamera::completeShutdown()
{
    request_thread.join();

	input_buffers_available_mutex.lock();
    abortPoll = true;
	input_buffers_available_mutex.unlock();

	output_mutex.lock();
    abortOutput = true;
    output_mutex.unlock();

	check_mutex.lock();
    abortCheckThread = true;
    check_mutex.unlock();

	event_mutex.lock();
	abortEventThread = true;
	event_mutex.unlock();

    poll_thread.join();
    output_thread.join();
	check_thread.join();
	event_thread.join();

    camera->stop();

	close(encoder_fd);
}

void SentinelCamera::stop()
{
	if ( !running )
		return;

	initiateShutdown();
	completeShutdown();
	running = false;
}

int SentinelCamera::xioctl(int fd, unsigned long ctl, void *arg)
{
	int ret, num_tries = 10;
	do
	{
		ret = ioctl(fd, ctl, arg);
	} while (ret == -1 && errno == EINTR && num_tries-- > 0);
	return ret;
}

void SentinelCamera::createDecoder()
{
	const char device_name[] = "/dev/video10";
	const int frameWidth = 1920;
	const int frameHeight = 1080;

	decoder_fd = open(device_name, O_RDWR | O_NONBLOCK);
	if ( decoder_fd < 0 )
		throw std::runtime_error("failed to open V4L2 decoder");

    v4l2_format coded_format = {};
    coded_format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    coded_format.fmt.pix_mp.width = frameWidth;
    coded_format.fmt.pix_mp.height = frameHeight;
    coded_format.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_H264;
    coded_format.fmt.pix_mp.num_planes = 1;
    if (v4l2_ioctl(decoder_fd, VIDIOC_S_FMT, &coded_format))
		throw std::runtime_error("error setting coded format");

    v4l2_requestbuffers coded_reqbuf = {};
    coded_reqbuf.count = 8;
    coded_reqbuf.type = coded_format.type;
    coded_reqbuf.memory = V4L2_MEMORY_MMAP;
    if (v4l2_ioctl(decoder_fd, VIDIOC_REQBUFS, &coded_reqbuf))
		throw std::runtime_error("error creating coded buffers");

    if (v4l2_ioctl(decoder_fd, VIDIOC_STREAMON, &coded_format.type))
		throw std::runtime_error("error starting coded stream");

    v4l2_format decoded_format = {};
    decoded_format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (v4l2_ioctl(decoder_fd, VIDIOC_G_FMT, &decoded_format))
		throw std::runtime_error("error getting decoded format");

    v4l2_requestbuffers decoded_reqbuf = {};
    decoded_reqbuf.count = 8;
    decoded_reqbuf.type = decoded_format.type;
    decoded_reqbuf.memory = V4L2_MEMORY_MMAP;
    if (v4l2_ioctl(decoder_fd, VIDIOC_REQBUFS, &decoded_reqbuf))
		throw std::runtime_error("error creating decoded buffers");

    if (v4l2_ioctl(decoder_fd, VIDIOC_STREAMON, &decoded_format.type))
		throw std::runtime_error("error starting decoded stream");
}

std::vector<std::unique_ptr<MappedBuffer>> SentinelCamera::map_decoder_buffers(v4l2_buf_type const type) 
{
    std::vector<std::unique_ptr<MappedBuffer>> buffers;

    for (int bi = 0;; ++bi) 
	{
        auto mapped = std::make_unique<MappedBuffer>();
        mapped->buffer.type = type;
        mapped->buffer.index = bi;
        mapped->buffer.length = 1;
        mapped->buffer.m.planes = &mapped->plane;

        if (v4l2_ioctl(decoder_fd, VIDIOC_QUERYBUF, &mapped->buffer)) 
		{
            if (bi > 0 && errno == EINVAL) 
				break;

			throw std::runtime_error("error querying buffers");	
        }

        mapped->size = mapped->plane.length;
        mapped->mmap = mmap(
            nullptr, mapped->size, PROT_READ | PROT_WRITE, MAP_SHARED, decoder_fd,
            mapped->plane.m.mem_offset
        );

        if (mapped->mmap == MAP_FAILED) 
			throw std::runtime_error("error memory mapping");

        buffers.push_back(std::move(mapped));
    }

    return buffers;
}

void SentinelCamera::readMask()
{
	struct jpeg_decompress_struct info;
	struct jpeg_error_mgr err;

	unsigned long int imgWidth, imgHeight;

	unsigned long int dwBufferBytes;
	unsigned char* lpData;

	unsigned char* lpRowBuffer[1];

	int frame_size = 1920 * 1080 / 9;
    memset( maskFrame, noise_level, frame_size );

	FILE* fHandle;

	fHandle = fopen( "mask.jpg", "rb");
	if ( fHandle == NULL )
	{
		std::cerr << "No mask file found. Noise level set to: " << noise_level << std::endl;
		return;
	}

	info.err = jpeg_std_error(&err);
	jpeg_create_decompress(&info);

	jpeg_stdio_src(&info, fHandle);
	jpeg_read_header(&info, TRUE);

	jpeg_start_decompress(&info);
	imgWidth = info.output_width;
	imgHeight = info.output_height;

	dwBufferBytes = imgWidth * imgHeight * 3; /* We only read RGB, not A */
	lpData = new unsigned char[dwBufferBytes];

	/* Read scanline by scanline */
	while ( info.output_scanline < info.output_height ) 
	{
		lpRowBuffer[0] = (unsigned char *)(&lpData[3*info.output_width*info.output_scanline]);
		jpeg_read_scanlines(&info, lpRowBuffer, 1);
	}

	jpeg_finish_decompress(&info);
	jpeg_destroy_decompress(&info);
	fclose(fHandle);

	if ( imgWidth == 640 && imgWidth == 360 )
	{
		for ( int i = 0; i < 640*360; ++i )
		{
			int r = lpData[3*i];
			int g = lpData[3*i+1];
			int b = lpData[3*i+2];

        	if (r > 250 && g < 10 && b < 10)
            	maskFrame[i] = 255;
		}
	}

	delete[] lpData;
}

void SentinelCamera::runDecoder( string videoFilePath, ProcessType process ) 
{
	std::ifstream videoFile;
	videoFile.open( videoFilePath.c_str(), std::ios::binary );
	if ( !videoFile.is_open() )
		return;

	size_t index = videoFilePath.find(".h264");
	if ( index == string::npos )
		return;

	string textFilePath = videoFilePath;
	textFilePath.replace( index, 5, ".txt" );

	// std::cout << textFilePath << std::endl;

	std::ifstream textFile;
	textFile.open( textFilePath.c_str() );
	if ( !textFile.is_open() )
		return;

    auto const coded_buf_type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    auto const decoded_buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    auto const coded_buffers = map_decoder_buffers(coded_buf_type);
    auto const decoded_buffers = map_decoder_buffers(decoded_buf_type);

    std::vector<MappedBuffer*> coded_free, decoded_free;
    for (auto const &b : coded_buffers) 
		coded_free.push_back(b.get());
    for (auto const &b : decoded_buffers) 
		decoded_free.push_back(b.get());

    v4l2_plane received_plane = {};
    v4l2_buffer received = {};
    received.memory = V4L2_MEMORY_MMAP;
    received.length = 1;
    received.m.planes = &received_plane;

    bool drained = false;

	std::list<string> timeList;

    v4l2_decoder_cmd command = {};
    command.cmd = V4L2_DEC_CMD_START;
    if (v4l2_ioctl(decoder_fd, VIDIOC_DECODER_CMD, &command)) 
		throw std::runtime_error("error sending start");

    while (!drained) 
	{
        // Reclaim coded buffers once consumed by the decoder.
        received.type = coded_buf_type;
        while (!v4l2_ioctl(decoder_fd, VIDIOC_DQBUF, &received)) 
		{
            if (received.index > coded_buffers.size()) 
				throw std::runtime_error( "bad reclaimed indes");
            coded_free.push_back(coded_buffers[received.index].get());
        }
        if (errno != EAGAIN)
			throw std::runtime_error( "error reclaiming coded buffer");

        // Push coded data into the decoder.
        while (!coded_free.empty() && !textFile.eof() ) 
		{
            auto* coded = coded_free.back();
            coded_free.pop_back();
            coded->plane.bytesused = 0;

			int frameCount = 0;
			string timeValue;
			int frameSize = 0;

			textFile >> frameCount >> timeValue >> frameSize;
			
			if ( !textFile.fail() )
			{
				timeList.push_back( timeValue );

				// std::cout << frameCount << " " << timeValue << " " << frameSize << std::endl;

				videoFile.read( (char *)coded->mmap, frameSize );
				coded->plane.bytesused = frameSize;

            	if (v4l2_ioctl(decoder_fd, VIDIOC_QBUF, &coded->buffer))
					throw std::runtime_error("error sending coded buffer");
			}
			else
			{
                v4l2_decoder_cmd command = {};
                command.cmd = V4L2_DEC_CMD_STOP;
                if (v4l2_ioctl(decoder_fd, VIDIOC_DECODER_CMD, &command)) 
					throw std::runtime_error("error sending STOP");
            }
        }

        // Send empty decoded buffers to be filled by the decoder.
        while (!decoded_free.empty()) 
		{
            auto* decoded = decoded_free.back();
            decoded_free.pop_back();

            if (v4l2_ioctl(decoder_fd, VIDIOC_QBUF, &decoded->buffer))
				throw std::runtime_error( "error cycling buffer " );
        }

        // Receive decoded data and return the buffers.
        received.type = decoded_buf_type;
        while (!drained && !v4l2_ioctl(decoder_fd, VIDIOC_DQBUF, &received)) 
		{
            if (received.index > decoded_buffers.size())
				throw std::runtime_error( "bad decoded index");

            drained = (received.flags & V4L2_BUF_FLAG_LAST);
            auto* decoded = decoded_buffers[received.index].get();
            //
			if ( !timeList.empty() )
			{
				string time = timeList.front();
				timeList.pop_front();
				bool more = process( decoded->mmap, time );
				if ( !more )
					drained = true;
			}

            decoded_free.push_back(decoded);
        }
        if (errno != EAGAIN)
			throw std::runtime_error( "error receiving decoded buffers" );

        // Poll after a 10ms delay -- TODO use poll() instead
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void SentinelCamera::stopDecoder() 
{
    int coded_type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    if (v4l2_ioctl(decoder_fd, VIDIOC_STREAMOFF, &coded_type)) 
		throw std::runtime_error("error stopping coded stream");

    v4l2_requestbuffers coded_reqbuf = {};
    coded_reqbuf.type = coded_type;
    coded_reqbuf.memory = V4L2_MEMORY_MMAP;
    if (v4l2_ioctl(decoder_fd, VIDIOC_REQBUFS, &coded_reqbuf)) 
		throw std::runtime_error("error releasing coded buffers");

    int decoded_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (v4l2_ioctl(decoder_fd, VIDIOC_STREAMOFF, &decoded_type)) 
		throw std::runtime_error("error stopping decoded stream");

    v4l2_requestbuffers decoded_reqbuf = {};
    decoded_reqbuf.type = decoded_type;
    decoded_reqbuf.memory = V4L2_MEMORY_MMAP;
    if (v4l2_ioctl(decoder_fd, VIDIOC_REQBUFS, &decoded_reqbuf)) 
		throw std::runtime_error("error releasing decoded buffers");
}

void SentinelCamera::encodeJPEG( void* mem, const string& fileName )
{
	struct jpeg_compress_struct cinfo;
	struct jpeg_error_mgr jerr;
	cinfo.err = jpeg_std_error(&jerr);
	jpeg_create_compress(&cinfo);

	unsigned int width = 1920;
	unsigned int height = 1080;
	unsigned int stride = 1920;
	unsigned int default_quality = 93;

	// Copied from YUV420_to_JPEG_fast in jpeg.cpp.
	cinfo.image_width = width;
	cinfo.image_height = height;
	cinfo.input_components = 3;
	cinfo.in_color_space = JCS_YCbCr;
	cinfo.restart_interval = 0;

	jpeg_set_defaults(&cinfo);
	cinfo.raw_data_in = TRUE;
	jpeg_set_quality(&cinfo, default_quality, TRUE);
	uint8_t* encoded_buffer = nullptr;
	size_t buffer_len = 0;
	unsigned long jpeg_mem_len;
	jpeg_mem_dest(&cinfo, &encoded_buffer, &jpeg_mem_len);
	jpeg_start_compress(&cinfo, TRUE);

	int stride2 = stride / 2;
	uint8_t *Y = (uint8_t *)mem;
	uint8_t *U = (uint8_t *)Y + stride * (height + 8);
	uint8_t *V = (uint8_t *)U + stride2 * (height / 2 + 4);
	uint8_t *Y_max = U - stride;
	uint8_t *U_max = V - stride2;
	uint8_t *V_max = U_max + stride2 * (height / 2);

	JSAMPROW y_rows[16];
	JSAMPROW u_rows[8];
	JSAMPROW v_rows[8];

	for (uint8_t *Y_row = Y, *U_row = U, *V_row = V; cinfo.next_scanline < height;)
	{
		for (int i = 0; i < 16; i++, Y_row += stride)
			y_rows[i] = std::min(Y_row, Y_max);

		for (int i = 0; i < 8; i++, U_row += stride2, V_row += stride2)
		{
			u_rows[i] = std::min(U_row, U_max);
			v_rows[i] = std::min(V_row, V_max);
		}

		JSAMPARRAY rows[] = { y_rows, u_rows, v_rows };
		jpeg_write_raw_data(&cinfo, rows, 16);
	}

	jpeg_finish_compress(&cinfo);
	buffer_len = jpeg_mem_len;

	std::ofstream out( fileName.c_str(), std::ios::binary );
	out.write( (char *)encoded_buffer, buffer_len );
	out.close();

	jpeg_destroy_compress(&cinfo);
}

void SentinelCamera::createEncoder()
{
	// First open the encoder device. Maybe we should double-check its "caps".

	const char device_name[] = "/dev/video11";
	const int frameWidth = 1920;
	const int frameHeight = 1080;

	encoder_fd = open(device_name, O_RDWR, 0);
	if (encoder_fd < 0)
		throw std::runtime_error("failed to open V4L2 H264 encoder");

	// std::cerr << "Opened H264Encoder on " << device_name << " as fd " << encoder_fd << std::endl;

	// Apply any options->

	v4l2_control ctrl = {};

	ctrl.id = V4L2_CID_MPEG_VIDEO_BITRATE;
	ctrl.value = 10000000;
	if (xioctl(encoder_fd, VIDIOC_S_CTRL, &ctrl) < 0)
		throw std::runtime_error("failed to set bitrate");
	
	ctrl.id = V4L2_CID_MPEG_VIDEO_H264_PROFILE;
    // Or V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE or V4L2_MPEG_VIDEO_H264_PROFILE_MAIN
	ctrl.value = V4L2_MPEG_VIDEO_H264_PROFILE_HIGH; 
	if (xioctl(encoder_fd, VIDIOC_S_CTRL, &ctrl) < 0)
		throw std::runtime_error("failed to set profile");

	ctrl.id = V4L2_CID_MPEG_VIDEO_H264_LEVEL;
    // Or V4L2_MPEG_VIDEO_H264_LEVEL_4_1 or V4L2_MPEG_VIDEO_H264_LEVEL_4_2
	ctrl.value = V4L2_MPEG_VIDEO_H264_LEVEL_4_0; 
	if (xioctl(encoder_fd, VIDIOC_S_CTRL, &ctrl) < 0)
		throw std::runtime_error("failed to set level");

	ctrl.id = V4L2_CID_MPEG_VIDEO_H264_I_PERIOD;
	ctrl.value = 30;
	if (xioctl(encoder_fd, VIDIOC_S_CTRL, &ctrl) < 0)
		throw std::runtime_error("failed to set intra period");
	
	ctrl.id = V4L2_CID_MPEG_VIDEO_REPEAT_SEQ_HEADER;
	ctrl.value = 1;
	if (xioctl(encoder_fd, VIDIOC_S_CTRL, &ctrl) < 0)
		throw std::runtime_error("failed to set inline headers");

	// Set the output and capture formats. We know exactly what they will be.

	v4l2_format fmt = {};
	fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	fmt.fmt.pix_mp.width = frameWidth;
	fmt.fmt.pix_mp.height = frameHeight;
	fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_YUV420;
	fmt.fmt.pix_mp.field = V4L2_FIELD_ANY;
	// libcamera currently has no means to request the right colour space, hence:
	fmt.fmt.pix_mp.colorspace = V4L2_COLORSPACE_JPEG;
	fmt.fmt.pix_mp.num_planes = 1;
	if (xioctl(encoder_fd, VIDIOC_S_FMT, &fmt) < 0)
		throw std::runtime_error("failed to set output format");

	fmt = {};
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	fmt.fmt.pix_mp.width = frameWidth;
	fmt.fmt.pix_mp.height = frameHeight;
	fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_H264;
	fmt.fmt.pix_mp.field = V4L2_FIELD_ANY;
	fmt.fmt.pix_mp.colorspace = V4L2_COLORSPACE_DEFAULT;
	fmt.fmt.pix_mp.num_planes = 1;
	fmt.fmt.pix_mp.plane_fmt[0].bytesperline = 0;
	fmt.fmt.pix_mp.plane_fmt[0].sizeimage = 512 << 10;
	if (xioctl(encoder_fd, VIDIOC_S_FMT, &fmt) < 0)
		throw std::runtime_error("failed to set capture format");

	// Request that the necessary buffers are allocated. The output queue
	// (input to the encoder) shares buffers from our caller, these must be
	// DMABUFs. Buffers for the encoded bitstream must be allocated and
	// m-mapped.

	v4l2_requestbuffers reqbufs = {};
	reqbufs.count = NUM_OUTPUT_BUFFERS;
	reqbufs.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	reqbufs.memory = V4L2_MEMORY_DMABUF;
	if (xioctl(encoder_fd, VIDIOC_REQBUFS, &reqbufs) < 0)
		throw std::runtime_error("request for output buffers failed");

	// std::cerr << "Got " << reqbufs.count << " output buffers" << std::endl;

	// We have to maintain a list of the buffers we can use when our caller gives
	// us another frame to encode.
	for (unsigned int i = 0; i < reqbufs.count; i++)
		input_buffers_available.push(i);

	reqbufs = {};
	reqbufs.count = NUM_CAPTURE_BUFFERS;
	reqbufs.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	reqbufs.memory = V4L2_MEMORY_MMAP;
	if (xioctl(encoder_fd, VIDIOC_REQBUFS, &reqbufs) < 0)
		throw std::runtime_error("request for capture buffers failed");

	// std::cerr << "Got " << reqbufs.count << " capture buffers" << std::endl;

	for (unsigned int i = 0; i < reqbufs.count; i++)
	{
		v4l2_plane planes[VIDEO_MAX_PLANES];
		v4l2_buffer buffer = {};
		buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
		buffer.memory = V4L2_MEMORY_MMAP;
		buffer.index = i;
		buffer.length = 1;
		buffer.m.planes = planes;
		if (xioctl(encoder_fd, VIDIOC_QUERYBUF, &buffer) < 0)
			throw std::runtime_error("failed to capture query buffer " + std::to_string(i));
		buffers[i].mem = mmap(0, buffer.m.planes[0].length, PROT_READ | PROT_WRITE, MAP_SHARED, encoder_fd,
							   buffer.m.planes[0].m.mem_offset);
		if (buffers[i].mem == MAP_FAILED)
			throw std::runtime_error("failed to mmap capture buffer " + std::to_string(i));
		buffers[i].size = buffer.m.planes[0].length;
		// Whilst we're going through all the capture buffers, we may as well queue
		// them ready for the encoder to write into.
		if (xioctl(encoder_fd, VIDIOC_QBUF, &buffer) < 0)
			throw std::runtime_error("failed to queue capture buffer " + std::to_string(i));
	}

	// Enable streaming and we're done.

	v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	if (xioctl(encoder_fd, VIDIOC_STREAMON, &type) < 0)
		throw std::runtime_error("failed to start output streaming");
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	if (xioctl(encoder_fd, VIDIOC_STREAMON, &type) < 0)
		throw std::runtime_error("failed to start capture streaming");

	// std::cerr << "Codec streaming started" << std::endl;
}

void SentinelCamera::encodeBuffer(int fd, size_t size, int64_t timestamp_us)
{
	int index;
	{
		// We need to find an available output buffer (input to the codec) to
		// "wrap" the DMABUF.
		std::lock_guard<std::mutex> lock(input_buffers_available_mutex);
		if (input_buffers_available.empty())
			throw std::runtime_error("no buffers available to queue codec input");
		index = input_buffers_available.front();
		input_buffers_available.pop();
	}
	v4l2_buffer buf = {};
	v4l2_plane planes[VIDEO_MAX_PLANES] = {};
	buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	buf.index = index;
	buf.field = V4L2_FIELD_NONE;
	buf.memory = V4L2_MEMORY_DMABUF;
	buf.length = 1;
	buf.timestamp.tv_sec = timestamp_us / 1000000;
	buf.timestamp.tv_usec = timestamp_us % 1000000;
	buf.m.planes = planes;
	buf.m.planes[0].m.fd = fd;
	buf.m.planes[0].bytesused = size;
	buf.m.planes[0].length = size;
	if (xioctl(encoder_fd, VIDIOC_QBUF, &buf) < 0)
		throw std::runtime_error("failed to queue input to codec");
}

void SentinelCamera::pollThread()
{
    abortPoll = false;

	while (true)
	{
		pollfd p = { encoder_fd, POLLIN, 0 };
		int ret = poll(&p, 1, 200);
		{
			std::lock_guard<std::mutex> lock(input_buffers_available_mutex);
			if (abortPoll && input_buffers_available.size() == NUM_OUTPUT_BUFFERS)
				break;
		}
		if (ret == -1)
		{
			if (errno == EINTR)
				continue;
			throw std::runtime_error("unexpected errno " + std::to_string(errno) + " from poll");
		}
		if (p.revents & POLLIN)
		{
			v4l2_buffer buf = {};
			v4l2_plane planes[VIDEO_MAX_PLANES] = {};
			buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
			buf.memory = V4L2_MEMORY_DMABUF;
			buf.length = 1;
			buf.m.planes = planes;
			int ret = xioctl(encoder_fd, VIDIOC_DQBUF, &buf);
			if (ret == 0)
			{
				// Return this to the caller, first noting that this buffer, identified
				// by its index, is available for queueing up another frame.
				{
					std::lock_guard<std::mutex> lock(input_buffers_available_mutex);
					input_buffers_available.push(buf.index);
				}
				// input_done_callback_(nullptr);
			}

			buf = {};
			memset(planes, 0, sizeof(planes));
			buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
			buf.memory = V4L2_MEMORY_MMAP;
			buf.length = 1;
			buf.m.planes = planes;
			ret = xioctl(encoder_fd, VIDIOC_DQBUF, &buf);
			if (ret == 0)
			{
				// We push this encoded buffer to another thread so that our
				// application can take its time with the data without blocking the
				// encode process.
				int64_t timestamp_us = (buf.timestamp.tv_sec * (int64_t)1000000) + buf.timestamp.tv_usec;
				OutputItem item = { buffers[buf.index].mem,
									buf.m.planes[0].bytesused,
									buf.m.planes[0].length,
									buf.index,
									!!(buf.flags & V4L2_BUF_FLAG_KEYFRAME),
									timestamp_us };
				std::lock_guard<std::mutex> lock(output_mutex);
				output_queue.push(item);
				output_cond_var.notify_one();
			}
		}
	}
}

string SentinelCamera::secondsString( int64_t timestamp_us )
{
	char buff[20];

	time_offset_mutex.lock();
	timespec ts = time_offset;
	time_offset_mutex.unlock();

	long seconds = ts.tv_sec + timestamp_us / 1000000;
	int microsecs = ts.tv_nsec / 1000 + timestamp_us % 1000000;

	if ( microsecs >= 1000000 )
	{
		microsecs -= 1000000;
		seconds += 1;
	}

	sprintf( buff, "%ld.%06d", seconds, microsecs );
	return buff;
}

string SentinelCamera::dateTimeString( int64_t timestamp_us )
{
	char buff[50];

	time_offset_mutex.lock();
	timespec ts = time_offset;
	time_offset_mutex.unlock();

	time_t seconds = ts.tv_sec + timestamp_us / 1000000;
	int microsecs = ts.tv_nsec / 1000 + timestamp_us % 1000000;

	if ( microsecs >= 1000000 )
	{
		microsecs -= 1000000;
		seconds += 1;
	}

	struct tm t = *gmtime( &seconds );
	sprintf( buff, "%4d%02d%02d_%02d%02d%02d_%03d", t.tm_year+1900, t.tm_mon+1, t.tm_mday,
	         t.tm_hour,t.tm_min,t.tm_sec, microsecs/1000);

	return buff;
}

void SentinelCamera::outputThread()
{
    abortOutput = false;

	bool keyfound = false;
	std::ofstream videoFile;
	std::ofstream textFile;
	string oldMinute = "20220101_0000";

	storage = new unsigned char[STORAGE_SIZE];
	unsigned int offset = 0;

	OutputItem item;
	while (true)
	{
		{
			std::unique_lock<std::mutex> locker(output_mutex);
			output_cond_var.wait_for( locker, 200ms, [this]() {
				return abortOutput || !output_queue.empty();
			});

			if ( abortOutput && output_queue.empty() )
				break;

			if ( output_queue.empty() )
				continue;

			item = output_queue.front();
			output_queue.pop();
		}

		unsigned int remainder = STORAGE_SIZE - offset;
		if ( remainder >= item.bytes_used )
		{
			memcpy( storage+offset, item.mem, item.bytes_used );
		}
		else
		{
			memcpy( storage+offset, item.mem, remainder );
			memcpy( storage, (unsigned char*)item.mem+remainder, item.bytes_used-remainder );
		}

		StorageDescription sd;
		sd.offset = offset;
		sd.size = item.bytes_used;
		sd.key_frame = item.keyframe;

		storage_mutex.lock();
		storageMap[item.timestamp_us] = sd;
		if ( storageMap.size() > 150 )
			storageMap.erase( storageMap.begin() );
		storage_mutex.unlock();

		offset = (offset + item.bytes_used) % STORAGE_SIZE;

		keyfound = keyfound || item.keyframe != 0;
		string dateTime = dateTimeString(item.timestamp_us);
		string minuteString = dateTime.substr(0,13);

		if ( minuteString != oldMinute && item.keyframe != 0 && !archivePath.empty() )
		{
			oldMinute = minuteString;
			if ( videoFile.is_open() )
				videoFile.close();
			if ( textFile.is_open() )
				textFile.close();

			string hourString = minuteString.substr(0,11);

			std::filesystem::path folderPath = archivePath;
			folderPath.append("s");
			folderPath.concat( hourString );
			int e = mkdir( folderPath.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH );
        	if ( e == -1 && errno != EEXIST )
            	std::cerr << "Could not create archive folder: " << folderPath << std::endl;

			std::filesystem::path videoPath = folderPath;
			videoPath.append("s");
			videoPath.concat( minuteString );
			videoPath.concat( ".h264" );

			std::cout << videoPath << std::endl;

			std::filesystem::path textPath = videoPath;
			textPath.replace_extension(".txt");

			videoFile.open( videoPath, std::ios::binary );
			textFile.open( textPath );
		}

		if ( videoFile.is_open() )
			videoFile.write( (char *)item.mem, item.bytes_used );

		if ( textFile.is_open() )
			textFile << dateTime << std::setw(7) << item.bytes_used << std::endl;

		v4l2_buffer buf = {};
		v4l2_plane planes[VIDEO_MAX_PLANES] = {};
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = item.index;
		buf.length = 1;
		buf.m.planes = planes;
		buf.m.planes[0].bytesused = 0;
		buf.m.planes[0].length = item.length;
		if (xioctl(encoder_fd, VIDIOC_QBUF, &buf) < 0)
			throw std::runtime_error("failed to re-queue encoded buffer");
	}

	delete[] storage;
}

void SentinelCamera::checkThread()
{
	abortCheckThread = false;
	int previous = 0;

	referenceFrame = new unsigned char[CHECK_FRAME_SIZE];
	maskFrame      = new unsigned char[CHECK_FRAME_SIZE];

	memset(referenceFrame, 0xff, CHECK_FRAME_SIZE);
	memset(maskFrame, noise_level, CHECK_FRAME_SIZE);

	int64_t frameTime = 0;

	const int FRAMES_PER_HOUR = 60 * 60 * 30;

	bool triggered = false;
	bool untriggered = false;
	int eventDuration = 0;
	int frameCount = 0;
	int triggerCount = 0;
	int untriggerCount = 0;
	int rateLimitBank = FRAMES_PER_HOUR; // One hour worth of frames
	bool force = false;

	auto initiateTrigger = [&](){
		triggered = true;
		untriggered = false;
		eventDuration = 0;
		force = false;

		event_mutex.lock();
		eventQueue.push( frameTime );
		event_mutex.unlock();

		std::cout << "Initiate trigger at: " << dateTimeString(frameTime) << std::endl;
	};

	auto terminateTrigger = [&,this](){
		triggered = false;
		untriggered = false;

		event_mutex.lock();
		eventQueue.push( frameTime );
		event_mutex.unlock();

		// Temporarily de-sensitize
		memset(referenceFrame, 0xff, CHECK_FRAME_SIZE);

		rateLimitBank = std::min(0,rateLimitBank-FRAMES_PER_HOUR/max_events_per_hour);

		std::cout << "Terminate trigger at:" << dateTimeString(frameTime) << std::endl;
	};

	for (;;)
	{
		++frameCount;
		rateLimitBank = std::max(rateLimitBank+1,FRAMES_PER_HOUR);

		{
			std::unique_lock<std::mutex> locker(check_mutex);
			checkCondition.wait_for( locker, 200ms, [this]() {
				return (checkBufferHead != checkBufferTail) || abortCheckThread;
			});

			if ( abortCheckThread )
				break;

			if ( checkBufferHead == checkBufferTail )
				continue;

			previous = checkBufferTail;
			checkBufferTail = (checkBufferTail+1) % NUM_CHECK_BUFFERS;

			if ( force_event )
			{
				force = true;
				force_event = false;
			}
		}

		CheckBufferDescription& desc = checkBuffers[previous];
		frameTime = desc.timestamp_us;
		unsigned char* p = desc.mem;
		unsigned char* pend = p + CHECK_FRAME_SIZE;

		unsigned char* pRef = referenceFrame;
		unsigned char* pMask = maskFrame;

		int sum = 0;

		while ( p < pend )
		{
			int c = *p++;

			int test = c - *pRef - *pMask++;
			if ( test > 0 )
				sum += test;

			c *= 15;
			c += test;
			c >>= 4;

			*pRef++ = c;			
		}


		if ( !triggered )
		{
			bool tooMany = max_events_per_hour * rateLimitBank / FRAMES_PER_HOUR == 0;

			if ( force || frameCount == force_count )
				initiateTrigger();
			else if ( sum >= sumThreshold && !tooMany )
			{
				++triggerCount;
				if ( triggerCount >= 2 )
					initiateTrigger();
			}
			else
				triggerCount = 0;
		}
		else if ( triggered && !untriggered )
		{
			++eventDuration;

			if ( sum < sumThreshold )
			{
				++untriggerCount;
				if ( untriggerCount >= 2 )
					untriggered = true;
			}
			else
			{
				untriggerCount = 0;
				if ( eventDuration > 300 )
					terminateTrigger();
			}
		}
		else
		{
			++eventDuration;

			if ( sum < sumThreshold )
				++untriggerCount;

			if ( eventDuration > 60 && untriggerCount >= 15 )
				terminateTrigger();
			else if ( eventDuration > 300 )
				terminateTrigger();
		}

		eventCondition.notify_one();
	}

	delete[] referenceFrame;
	delete[] maskFrame;
}

void SentinelCamera::eventThread()
{
	abortEventThread = false;
	int64_t scan_time = 0;
	int64_t end_time = 0;
	bool keyframe_found = false;

	std::ofstream videoFile;
	std::ofstream textFile;

	int frame_count = 0;

	for (;;)
	{
		int64_t tempTime = 0;

		{
			std::unique_lock<std::mutex> locker(event_mutex);
			eventCondition.wait_for( locker, 100ms );

			if ( abortEventThread )
				break;

			if ( end_time == 0 && !eventQueue.empty() )
			{
				tempTime = eventQueue.front();
				eventQueue.pop();
			}
		}

		if ( scan_time == 0 && tempTime == 0 )
			continue;

		if ( scan_time == 0 && tempTime != 0 )
		{
			// Open file
			string timeString = dateTimeString( tempTime );
			std::filesystem::path videoPath = "new/s";
			videoPath.concat( timeString );
			videoPath.concat( ".h264" );
			std::filesystem::path textPath = videoPath;
			textPath.replace_extension( ".txt" );

			videoFile.open( videoPath, std::ios::binary );
			if ( !videoFile.is_open() )
				std::cerr << "Could not open: " << videoPath << std::endl;
			
			textFile.open( textPath );
			if ( !textFile.is_open() )
				std::cerr << "Could not open: " << textPath << std::endl;

			scan_time = tempTime - 2000000;
			frame_count = 0;
		}
		else if ( scan_time != 0 && tempTime != 0 )
			end_time = tempTime;

		if ( scan_time != 0 && end_time != 0 && scan_time > end_time )
		{
			// Close files
			if ( videoFile.is_open() )
				videoFile.close();
			if ( textFile.is_open() )
				textFile.close();
			scan_time = 0;
			end_time = 0;
			keyframe_found = false;
		}

		StorageDescription sd;
		sd.offset = 0;
		sd.size = 0;
		int64_t storageTime = 0;
		map<int64_t,StorageDescription>::const_iterator scan;

		if ( scan_time != 0 )
		{
			storage_mutex.lock();
			scan = storageMap.upper_bound( scan_time );
			if ( scan != storageMap.end() )
			{
				storageTime = scan->first;
				sd = scan->second;
			}
			storage_mutex.unlock();
		}

		if ( storageTime != 0 )
		{
			keyframe_found = keyframe_found || sd.key_frame;

			if ( keyframe_found && videoFile.is_open() && textFile.is_open() )
			{
				int64_t remainder = STORAGE_SIZE - sd.offset;
				char* ps = (char *)storage;
				if ( sd.size <= remainder )
					videoFile.write( ps+sd.offset, sd.size );
				else
				{
					videoFile.write( ps+sd.offset, remainder );
					videoFile.write( ps, sd.size-remainder );
				}

				string dateTime = dateTimeString( storageTime );
				textFile << std::setw(4) << ++frame_count 
				         << " " << dateTime 
						 << std::setw(7) << sd.size << std::endl;

				// std::cout << frame_count << std::endl;
			}

			scan_time = storageTime+1;
		}
	}
}

void SentinelCamera::readCalibrationParameters()
{
	calibrationParameters["V"]     = 0.002278; 
	calibrationParameters["S"]     = 0.639837; 
	calibrationParameters["D"]     = -0.000982;  
	calibrationParameters["a0"]    = -103.264; 			    
	calibrationParameters["E"]     = 253.174; 
	calibrationParameters["eps"]   = 1.053; 
	calibrationParameters["COPx"]  = 986.444; 
	calibrationParameters["COPy"]  = 539.240; 
	calibrationParameters["alpha"] = 1.570796; 
	calibrationParameters["flat"]  = 0.0;
	
	std::ifstream in("calibration.json");
	if (in.is_open() )
	{
		std::stringstream strStream;
    	strStream << in.rdbuf();	
    	std::string str = strStream.str();

		Parse( str.c_str(), calibrationParameters );
	}

    double alpha = calibrationParameters["alpha"];
    double flat  = calibrationParameters["flat"];
    double COPx  = calibrationParameters["COPx"];
    double COPy  = calibrationParameters["COPy"];

    double dilation = sqrt(1.0-flat);
    double K = COPx*sin(alpha) + COPy*cos(alpha);
    double L = COPy*sin(alpha) - COPx*cos(alpha);

    calibrationParameters["c"] = cos(alpha)*cos(alpha)*dilation + sin(alpha)*sin(alpha)/dilation;
    calibrationParameters["d"] = sin(alpha)*cos(alpha)*dilation - sin(alpha)*cos(alpha)/dilation;
    calibrationParameters["e"] = -(K*cos(alpha)*dilation*dilation - COPy*dilation + L*sin(alpha))/dilation;
    calibrationParameters["f"] = sin(alpha)*cos(alpha)*dilation - sin(alpha)*cos(alpha)/ dilation;
    calibrationParameters["g"] = sin(alpha)*sin(alpha)*dilation + cos(alpha)*cos(alpha)/dilation;
    calibrationParameters["h"] = -(K*sin(alpha)*dilation*dilation - COPx*dilation - L*cos(alpha))/dilation;
}

void SentinelCamera::calibrationFunction()
{
    double V     = calibrationParameters["V"];
    double S     = calibrationParameters["S"];
    double D     = calibrationParameters["D"];
    double a0    = calibrationParameters["a0"]  * M_PI / 180.0;
    double E     = calibrationParameters["E"]   * M_PI / 180.0;
    double eps   = calibrationParameters["eps"] * M_PI / 180.0;
    double COPx  = calibrationParameters["COPx"];
    double COPy  = calibrationParameters["COPy"];

    double c = calibrationParameters["c"];
    double d = calibrationParameters["d"];
    double e = calibrationParameters["e"];
    double f = calibrationParameters["f"];
    double g = calibrationParameters["g"];
    double h = calibrationParameters["h"];

    double px = calibrationParameters["px"];
    double py = calibrationParameters["py"];
    
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

    while ( azim >= 360.0 )
        azim -= 360.0;

    while ( azim < 0.0 )
        azim += 360.0;

    calibrationParameters["azim"] = azim;
    calibrationParameters["elev"] = elev;
}

void SentinelCamera::makeComposite( string filePath )
{
	unsigned char* composeBuffer = new unsigned char[1920*1088*3/2];

	auto fmax = [&](void* mem, string s) -> bool {	
		unsigned char* Y = (unsigned char *)mem;

		for ( int iy = 0; iy < 1080; ++iy )
		{
			for ( int ix = 0; ix < 1920; ++ix )
			{
				int index = iy*1920+ix;

				if ( Y[index] > composeBuffer[index] )
				{
					composeBuffer[index] = Y[index];
					int uindex = 1920*1088 + iy*1920/4 + ix/2;
					int vindex = uindex + 1920*1088/4;
					composeBuffer[uindex] = Y[uindex];
					composeBuffer[vindex] = Y[vindex];
				}
			}
		}

		return true;
	};

	memset(composeBuffer,0,1920*1088*3/2);

	createDecoder();
	runDecoder( filePath, fmax );
	stopDecoder();

	std::filesystem::path p = filePath;
	p.replace_extension(".jpg");

	encodeJPEG( composeBuffer, p );

	delete [] composeBuffer;
}

void SentinelCamera::makeAnalysis( string filePath )
{
	int* averageBuffer = new int[1920*1080];
	maskFrame = new unsigned char[CHECK_FRAME_SIZE];

	for ( int i = 0; i < 1920*1080; ++i )
		averageBuffer[i] = 0;

	int count = 0;
	int maxCount = 30;

	vector<string> timeVector;
	vector<double> pxVector;
	vector<double> pyVector;
	vector<int> sumVector;
	vector<int> countVector;

	auto add30 = [&](void* mem, string s) -> bool {
		unsigned char* ptr = (unsigned char*)mem;
		if ( ++count > maxCount )
			return false;
		for ( int i = 0; i < 1920*1080; ++i )
			averageBuffer[i] += ptr[i];
		return true;
	};

	auto centroid = [&](void* mem, string s) -> bool {
    	int sum = 0;
    	double sumx = 0.0;
    	double sumy = 0.0;
    	int tcount = 0;

		unsigned char* p = (unsigned char*)mem;

    	for (int iy = 0; iy < 1080; ++iy)
    	{
        	for (int ix = 0; ix < 1920; ++ix)
        	{
            	int index = 1920*iy + ix;

            	int test = p[index] - averageBuffer[index];
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
		timeVector.push_back(s);
		pxVector.push_back(px);
		pyVector.push_back(py);
		sumVector.push_back(sum);
		countVector.push_back(tcount);

		return true;
	};

	readMask();

	std::cout << "Create decoder" << std::endl;
	createDecoder();
	runDecoder( filePath, add30 );
	stopDecoder();
	std::cout << "Stop decoder" << std::endl;

    for ( int iy = 0; iy < 1080; ++iy )
    {
        for ( int ix = 0; ix < 1920; ++ix )
        {
            int index = 1920*iy + ix;
            int indexMask = 640*(iy/3)+(ix/3);

            averageBuffer[index] /= std::min(count,maxCount);
            averageBuffer[index] += maskFrame[indexMask];

            if ( moonx != 0 && moony != 0 )
            {
                int dx = ix - moonx;
                int dy = iy - moony;

                int dsq = dx*dx + dy+dy;

                if ( dsq < 400 )
                    averageBuffer[index] += 255;
            }
        }
    }

	std::cout << "Create decoder" << std::endl;
	createDecoder();
	runDecoder( filePath, centroid );
	stopDecoder();
	std::cout << "Stop decoder" << std::endl;

	delete [] averageBuffer;
	delete [] maskFrame;

	std::filesystem::path txtPath = filePath;
	txtPath.replace_extension(".csv");

	std::ofstream out( txtPath );
	if ( !out.is_open() )
		return;

	readCalibrationParameters();

	out.precision(1);
	for ( unsigned int i = 0; i < timeVector.size(); ++i )
	{
		calibrationParameters["px"] = pxVector[i];
		calibrationParameters["py"] = pyVector[i];

		calibrationFunction();

		string time = timeVector[i];
		int count = countVector[i];
		int sum = sumVector[i];
		double px = pxVector[i];
		double py = pyVector[i];

		double azim = sum == 0.0 ? 0 : calibrationParameters["azim"];
		double elev = sum == 0.0 ? 0 : calibrationParameters["elev"];

		out << time << "," << std::setw(6) << count << ",";
		out << std::setw(8) << sum << ",";
		out << std::fixed << std::setprecision(1) << std::setw(10) << px << ",";
		out << std::setw(10) << py << ",";
		out << std::setw(10) << azim << "," << std::setw(10) << elev << std::endl;
	}
}

void SentinelCamera::makeStarChart( string filePath )
{
	int* averageBuffer = new int[1920*1080];
	unsigned char* composeBuffer = new unsigned char[1920*1088*3/2];

	memset(averageBuffer, 0, sizeof(int)*1920*1080);
	memset(composeBuffer+1920*1088, 128, 1920*1088/2);

	std::filesystem::path textPath = filePath;
	textPath.replace_extension(".txt");

	std::ifstream in(textPath);
	if ( !in.is_open() )
	{
		std::cerr << "Cannot open: " << textPath << std::endl;
		return;
	}

	int count = 0;
	string unused;
	while ( std::getline( in, unused) )
		++count;

	int halfCount = count/2;

	count = 0;
	auto delta = [&](void* mem, string s) -> bool {
		unsigned char* p = (unsigned char*)mem;
		int factor = count++ < halfCount ? 1 : -1;

		for ( int i = 0; i < 1920*1080; ++i )
			averageBuffer[i] += p[i] * factor;

		return count < 2*halfCount;;
	};

	createDecoder();
	runDecoder( filePath, delta );
	stopDecoder();

	for ( int i = 0; i < 1920*1080; ++i )
	{
		double scaled = 200.0 * averageBuffer[i]/count;
		scaled = scaled >=   0 ? scaled : 0;
		scaled = scaled <= 250 ? scaled : 250;
		composeBuffer[i] = scaled;
	}

	std::filesystem::path p = filePath;
	p.replace_extension(".jpeg");

	encodeJPEG( composeBuffer, p );

	delete [] averageBuffer;
	delete [] composeBuffer;
}

void SentinelCamera::forceEvent()
{
	check_mutex.lock();
	force_event = true;
	check_mutex.unlock();
}

void sendOK( bool ok )
{
	if ( ok )
		std::cout << "=OK" << std::endl;
	else
		std::cout << "=No" << std::endl;
}

void runInteractive( bool mum )
{
    SentinelCamera sentinelCamera;

	std::string cmd;

	for (;;)
	{
		bool running = sentinelCamera.running;

		if ( ! mum )
			std::cout << "cmd: ";

		std::cout << std::flush;

		string line;
		std::getline( std::cin, line );

		std::istringstream iss(line);
		std::string cmd;

		iss >> cmd;

		if ( cmd == "quit" )
		{
			sentinelCamera.stop();
			std::cout << "Quit" << std::endl;
			return;
		}

		if ( cmd == "get_running" )
			std::cout << (running ? "=Yes" : "=No") << std::endl;
		else if ( cmd == "start" )
		{
			if ( running )
				sendOK( false );
			else
			{
				sentinelCamera.start();
				sendOK( true );
			}
		}
		else if ( cmd == "stop" )
		{
			if ( !running )
				sendOK( false );
			else
			{
				sentinelCamera.stop();
				sendOK( true );
			}
		}
		else if ( cmd == "force_trigger" )
		{
			if ( running )
			{
				sentinelCamera.forceEvent();
				sendOK( true );
			}
			else
				sendOK( false );
		}
		else if ( cmd == "compose" )
		{
			string path;
			if ( iss >> path )
			{
				sentinelCamera.makeComposite( path );
				sendOK( true );
			}
			else
				sendOK( false );
		}
		else if ( cmd == "analyze" )
		{
			std::cout << (sentinelCamera.running ? "=No" : "=OK") << std::endl;
			string path;
			if ( iss >> path )
			{
				sentinelCamera.makeAnalysis( path );
				sendOK( true );
			}
			else
				sendOK( false );
		}
		else if ( cmd == "set_moon" )
		{
			int mx, my;
			if ( iss >> mx >> my )
			{
				sentinelCamera.moonx = mx;
				sentinelCamera.moony = my;
				sendOK( true );
			}
			else
				sendOK( false );
		}
		else if ( cmd == "set_noise" )
		{
			int noise;
			if ( iss >> noise )
			{				
				sentinelCamera.noise_level = noise;
				sendOK( true );
			}
			else
				sendOK( false );
		}
		else if ( cmd == "set_sum_threshold" )
		{
			int threshold;
			if ( iss >> threshold )
			{
				sentinelCamera.sumThreshold = threshold;
				sendOK( true );
			}
			else
				sendOK( false );
		}
		else if ( cmd == "set_archive_path" )
		{
			string path;
			if ( (iss >> path) )
			{
				if ( path == "none" )
					sentinelCamera.archivePath = "";
				else
					sentinelCamera.archivePath = path;
				sendOK( true );
			}
			else
				sendOK( false );
		}
		else if ( cmd == "set_max_events_per_hour" )
		{
			int max_events;
			if ( iss >> max_events )
			{
				sentinelCamera.max_events_per_hour = max_events;
				sendOK( true );
			}
			else
				sendOK( false );
		}
	}
}

void runSingle( int frame_count, int force_count, int noise_level )
{
	SentinelCamera sentinelCamera;

	sentinelCamera.max_frame_count = frame_count;
	sentinelCamera.start();
	sentinelCamera.completeShutdown();
}

void runDecoderTest( string path )
{	
	SentinelCamera camera;
	camera.makeComposite( path );
}

void runAnalysisTest( string path )
{
	SentinelCamera camera;
	camera.makeAnalysis( path );
}

int main( int argc, char **argv )
{
	bool mum = false;
	bool interactive = false;
	bool decoder_test = false;
	bool analysis_test = false;
	int frame_count = 300;
	int force_count = 200;
	int noise_level = 50;
	string path;

	int opt;

	while ((opt = getopt(argc, argv, "a:isd:c:f:n:")) != -1)
	{
		switch ( opt )
		{
			case 'a': analysis_test = true; path = optarg; break;
			case 'i': interactive = true; break;
			case 's': interactive = true; mum = true; break;
			case 'd': decoder_test = true; path = optarg; break;
			case 'c': frame_count = std::stoi( optarg ); break;
			case 'f': force_count = std::stoi( optarg ); break;
			case 'n': noise_level = std::stoi( optarg ); break;
		}
	}

	if ( decoder_test )
		runDecoderTest( path );
	else if ( analysis_test )
		runAnalysisTest( path );
	else if ( interactive )
		runInteractive( mum );
	else
		runSingle( frame_count, force_count, noise_level );

    return EXIT_SUCCESS;
}