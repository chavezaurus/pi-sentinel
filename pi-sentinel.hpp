#ifndef _pi_sentinel_hpp_
#define _pi_sentinel_hpp_

//
// Sentinel Camera
//

#include <mutex>
#include <condition_variable>

#include <vector>
#include <deque>
#include <queue>
#include <map>
#include <thread>
#include <string>
#include <sys/mman.h>
#include <functional>

#include <linux/videodev2.h>

// using namespace libcamera;
using std::mutex;
using std::condition_variable;
using std::vector;
using std::deque;
using std::queue;
using std::map;
using std::thread;
using std::string;

struct BufferDescription
{
	void *mem;
	size_t size;
};

struct StorageDescription
{
    unsigned int offset;
    unsigned int size;
    bool key_frame;
};

struct StorageMeta
{
    unsigned int offset;
    unsigned int size;
    int64_t timestamp_us;
    int signalAmplitude;
    bool key_frame;
};

struct CheckBufferDescription
{
    unsigned char* mem;
    int64_t timestamp_us;
};

struct OutputItem
{
	void *mem;
	size_t bytes_used;
	size_t length;
	unsigned int index;
	bool keyframe;
	int64_t timestamp_us;
};

struct MappedBuffer 
{
    v4l2_buffer buffer = {};
    v4l2_plane plane = {};
    void *mmap = MAP_FAILED;
    size_t size = 0;

    ~MappedBuffer() { if (mmap != MAP_FAILED) munmap(mmap, size); }
};

typedef std::function<bool (void*,string)> ProcessType;

class SentinelCamera
{
private:

    // CameraManager* cm;
    // std::shared_ptr<Camera> camera;
    // std::unique_ptr<CameraConfiguration> config;
    // FrameBufferAllocator *allocator;
    int id;
    int encoder_fd;
    int decoder_fd;
    int device_fd;
    unsigned int n_buffers;

    // deque<Request*> completedRequests;
    mutex requestMutex;
    condition_variable requestCondition;
    bool abortRequestThread;
    bool abortCheckThread;
    bool abortEventThread;
    bool abortDecoderThread;
    bool abortPoll;
    bool abortOutput;

	// vector<std::unique_ptr<Request>> requests;
    thread request_thread;
    thread poll_thread;
    thread output_thread;
    thread check_thread;
    thread event_thread;
    thread device_thread;
    thread decoder_thread;

    mutex input_buffers_available_mutex;
    queue<int> input_buffers_available;

    queue<OutputItem> output_queue;
    mutex output_mutex;

    condition_variable output_cond_var;

    static constexpr int NUM_DEVICE_BUFFERS = 4;
    static constexpr int NUM_OUTPUT_BUFFERS = 6;
    static constexpr int NUM_CAPTURE_BUFFERS = 12;
    static constexpr int NUM_CHECK_BUFFERS = 6;
    static constexpr int CHECK_FRAME_SIZE = 640 * 360;
    static constexpr int STORAGE_SIZE = 1024 << 13;
    static constexpr int NUM_STORAGE_META = 150;

    static vector<SentinelCamera*> cameraVector;

    BufferDescription buffers[NUM_CAPTURE_BUFFERS];
    BufferDescription deviceBuffers[NUM_DEVICE_BUFFERS];

    StorageMeta storageMetas[NUM_STORAGE_META];
    int metaWriteIndex;
    int metaCheckIndex;

    mutex meta_write_mutex;
    mutex meta_check_mutex;

    map<int,void*> mappedBuffers;

    CheckBufferDescription checkBuffers[NUM_CHECK_BUFFERS];
    int checkBufferHead;
    int checkBufferTail;
    mutex check_mutex;
    condition_variable checkCondition;
    bool force_event;

    mutex time_offset_mutex;
    struct timespec time_offset;

    mutex storage_mutex;
    unsigned char* storage;
    map<int64_t,StorageDescription> storageMap;

    mutex event_mutex;
    condition_variable eventCondition;
    queue<int64_t> eventQueue;

    condition_variable decoder_cond_var;

    unsigned char* referenceFrame;
    unsigned char* maskFrame;

    map<string,double> calibrationParameters;

public:

    SentinelCamera();
    ~SentinelCamera();

    void requestThread();
    void pollThread();
    void outputThread();
    void checkThread();
    void checkThread2();
    void eventThread();
    void startDeviceCapture();
    void stopDeviceCapture();
    void deviceCaptureThread();
    void decoderThread();

    int signalAmplitude( unsigned char* pstart );
    void start();
    void start2();
    void initiateShutdown();
    void completeShutdown();
    void stop();
    int xioctl(int fd, int ctl, void *arg);
    void createEncoder();
    void createDecoder();
    void encodeBuffer(int fd, size_t size, int64_t timestamp_us);
    void fillCheckBuffer( int fd, int64_t timestamp_us );
    void syncTime();
    void runDecoder( string videoFilePath, ProcessType process );
    void stopDecoder();
    void encodeJPEG(void * mem, const string& fileName );
    void readCalibrationParameters();
    void calibrationFunction();
    void readMask();
    void makeComposite( string filePath );
    void makeAnalysis( string filePath );
    void makeStarChart( string filePath );
    void forceEvent();

    void openDevice( string devicePath );
    void closeDevice();

    void initDevice();
    void uninitDevice();

    bool isIDR(const unsigned char *p, int size);


    std::vector<std::unique_ptr<MappedBuffer>> map_decoder_buffers(v4l2_buf_type const type);
    string secondsString( int64_t timestamp_us );
    string dateTimeString( int64_t timestamp_us );

    // static void requestComplete(Request* request);

    bool running;
    int noise_level;
    int moonx;
    int moony;
    int max_frame_count;
    int max_events_per_hour;
    int force_count;
    string archivePath;

    mutex zenith_mutex;
    double zenithAmplitude;
	
    int sumThreshold;
};

#endif // _pi_sentinel_hpp_
