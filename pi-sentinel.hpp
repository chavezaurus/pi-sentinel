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

struct StorageMeta
{
    unsigned int offset;
    unsigned int size;
    int64_t timestamp_us;
    int signalAmplitude;
    bool key_frame;
};

struct MappedBuffer 
{
    v4l2_buffer buffer = {};
    v4l2_plane plane = {};
    void *mmap = MAP_FAILED;
    size_t size = 0;

    ~MappedBuffer() { if (mmap != MAP_FAILED) munmap(mmap, size); }
};

typedef std::function<bool (void*,string s)> ProcessType;

class SentinelCamera
{
private:

    int device_fd;
    unsigned int n_buffers;

    bool abortDeviceThread;
    bool abortCheckThread;
    bool abortDecoderThread;
    bool abortArchiveThread;
    bool abortMp4Thread;
    bool abortDecompressThread;

    thread check_thread;
    thread device_thread;
    thread decoder_thread;
    thread archive_thread;
    thread mp4_thread;
    thread decompress_thread;

    queue<string> mp4Queue;

    queue<int> input_buffers_available;

    static constexpr int NUM_DEVICE_BUFFERS = 4;
    static constexpr int NUM_OUTPUT_BUFFERS = 6;
    static constexpr int NUM_CAPTURE_BUFFERS = 12;
    static constexpr int CHECK_FRAME_SIZE = 640 * 360;
    static constexpr int STORAGE_SIZE = 1024 << 13;
    static constexpr int NUM_STORAGE_META = 150;

    BufferDescription buffers[NUM_CAPTURE_BUFFERS];
    BufferDescription deviceBuffers[NUM_DEVICE_BUFFERS];

    StorageMeta storageMetas[NUM_STORAGE_META];
    int metaWriteIndex;
    int metaCheckIndex;

    mutex meta_write_mutex;
    mutex meta_check_mutex;
    mutex mp4_mutex;

    condition_variable checkCondition;
    bool force_event;

    mutex time_offset_mutex;
    struct timespec time_offset;

    unsigned char* storage;

    condition_variable decoder_cond_var;
    condition_variable archive_cond_var;
    condition_variable mp4_cond_var;
    condition_variable decompress_cond_var;

    unsigned char* referenceFrame;
    unsigned char* maskFrame;

    map<string,double> calibrationParameters;

public:

    SentinelCamera();
    ~SentinelCamera();

    void measureFrameRate(unsigned microseconds);
    void checkThread();
    void archiveThread();
    void startDeviceCapture();
    void stopDeviceCapture();
    void deviceCaptureThread();
    void decoderThread();
    void mp4Thread();
    void decompressThread();

    int signalAmplitude( unsigned char* pstart );
    double zenithAmplitude( unsigned char* pstart );
    void start();
    void initiateShutdown();
    void completeShutdown();
    void stop();
    int xioctl(int fd, int ctl, void *arg);
    void createEncoder( int& encoder_fd);
    void createDecoder( int& decoder_fd);
    void syncTime();
    void processDecoded( string videoFilePath, ProcessType process );
    void stopDecoder( int& decoder_fd );
    void encodeJPEG(void * mem, const string& fileName );
    void readCalibrationParameters();
    void calibrationFunction();
    void readMask( unsigned char* mask );
    void overlayMask( unsigned char* frame );
    void makeComposite( string filePath );
    void makeAnalysis( string filePath );
    void makeStarChart( string filePath );
    void forceEvent();
    void mjpegToH264( string filePath );

    void openDevice( string devicePath );
    void closeDevice();

    void initDevice();
    void uninitDevice();

    bool isIDR(const unsigned char *p, int size);

    std::vector<std::unique_ptr<MappedBuffer>> map_decoder_buffers(int decoder_fd, v4l2_buf_type const type);
    string dateTimeString( int64_t timestamp_us );

    bool running;
    bool mjpeg;
    int noise_level;
    int moonx;
    int moony;
    int max_frame_count;
    int max_events_per_hour;
    int force_count;
    string archivePath;
    string dev_name;
    string socket_name;

    mutex zenith_mutex;
    double averageZenithAmplitude;
    double frameRate;
    double gpsTimeOffset;
	
    int sumThreshold;
};

#endif // _pi_sentinel_hpp_
