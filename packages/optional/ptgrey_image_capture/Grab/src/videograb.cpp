/*
 * TODO: Add apropriate License and copyright header
 */

#include "videograb.h"
#include "ros/ros.h"
#include <string>
#include <iostream>
#include <sensor_msgs/Image.h>
#include <std_msgs/UInt8.h>
#include "frameptr.h"
#include "ptgreyhelper.h"
#include <boost/filesystem/fstream.hpp>
#include <boost/thread.hpp>
#include <unistd.h>
extern "C" {
#include "libavhelper.h"
}

namespace videograb {
videograb *globalVideoGrab;

/*
        void sigHandler(const boost::system::error_code& error, int signal_number) {
                // shutdown read and write threads too
                exit(-1);
                ros::shutdown();
        }
 */

class transfer_frame_priv {
public:
    uint64_t number;
    size_t size;
};

class videograb_priv {
public:
    int argc;
    char * *argv;
    ros::NodeHandle *nodehandle;
    boost::posix_time::ptime epoch;
    boost::posix_time::ptime start;
    boost::posix_time::ptime current;
    boost::mutex serial_Mutex;
    boost::mutex ROSinfo_Mutex;
    int width;
    int height;
    int framerate;
    std::string videofilename;
    std::string nameSpace;
    int threads;
    ptgreyhelper *helper;
    ros::Publisher frame_pub;
    boost::filesystem::ofstream timesfile;
    boost::thread *thread;
    int pipe[2];
    uint8_t *bufferqueue[64];
    volatile int bufferfill=0;
    volatile int bufferread=0;
    volatile int bufferwrite=0;
    volatile int recording = 0;
    
    // ...
    void run()
    {
        transfer_frame_priv transfer;
        while (thread) {
            read(pipe[0],&transfer,sizeof(transfer));
	    libavhelper_save_frame(bufferqueue[bufferread++],transfer.number);
	    bufferread=bufferread%64;
	    bufferfill--;
        }
    }
    
    void recordCallback(const std_msgs::UInt8::ConstPtr & msg)
    {
	recording=msg->data;
    }
    
};

videograb::videograb(int argc, char * *argv)
{
    globalVideoGrab = this;
    instance = new videograb_priv;
    if (argc<5) {
        std::cerr << "Usage: " << std::string(argv[0]) << " <namespace> <video folder> <framerate in fps> <threads used for encoding> [ROS arguments*]\n";
	exit(1);
    }
    instance->framerate=std::stoi(std::string(argv[3]));
    instance->threads=std::stoi(std::string(argv[4]));
    instance->epoch = boost::posix_time::ptime(boost::gregorian::date(1970,1,1));
    instance->start = boost::posix_time::microsec_clock::universal_time();
    instance->argc  = argc-4;
    instance->argv  = new char *[instance->argc];
    instance->argv[0] = argv[0];
    int t;
    for (t=5;t<argc;t++) {
       instance->argv[t-4]=argv[t];
    }
    boost::posix_time::time_duration diff = instance->start - instance->epoch;
    instance->videofilename = static_cast<std::ostringstream*>( &(std::ostringstream() << argv[2] << "/capture_" << diff.total_microseconds() << ".avi"))->str();
    instance->nameSpace = argv[1];
    instance->helper = new ptgreyhelper(instance->framerate);
    libavhelper_init_write(instance->videofilename.c_str(),"avi","ffvhuff",instance->helper->getWidth(),instance->helper->getHeight(),instance->framerate,AV_PIX_FMT_RGB24,instance->threads);
    ros::init(instance->argc,instance->argv,"videograb");
    instance->nodehandle = new ros::NodeHandle;
    instance->frame_pub = instance->nodehandle->advertise<sensor_msgs::Image>(instance->nameSpace + "/video",10);
    instance->timesfile.open(instance->videofilename + ".times");
    pipe(instance->pipe);

    // allocate loads of memory
    for (int t=0;t<64;t++) {
    	instance->bufferqueue[t] = new uint8_t[instance->helper->getWidth()*instance->helper->getHeight()*3];
    }
    instance->thread = new boost::thread(boost::bind(&videograb_priv::run, instance));
}

videograb::~videograb()
{
    if (instance->thread) {
        delete instance->thread;
    }
    if (instance->nodehandle) {
        delete instance->nodehandle;
    }
    if (instance->helper) {
	delete instance->helper;
    }
    delete instance;
    libavhelper_close_write();
}

int videograb::run(void)
{
    ros::init(instance->argc, instance->argv, "videograb");

    instance->nodehandle = new ros::NodeHandle();
    // boost::asio::signal_set signals(instance->io_service, SIGINT, SIGTERM);


    // do everything here

    ros::AsyncSpinner spinner(4);
    spinner.start();
  
    int64_t frames=0;
    transfer_frame_priv transfer;

    ros::Subscriber subscriber1 = instance->nodehandle->subscribe("/global/record",10, &videograb_priv::recordCallback, instance);

    int first=1;
    instance->bufferwrite=63;
    while (ros::ok()) {
	frameptr imagebuffer=instance->helper->getFrame(instance->bufferqueue[(instance->bufferwrite+1)%64],3*instance->helper->getWidth()*instance->helper->getHeight());
        if (!imagebuffer.buffer) {
		continue;
	}

	// send image to ros
	boost::posix_time::time_duration diff = imagebuffer.timestamp - instance->epoch;
	sensor_msgs::Image msg;
	msg.header.seq=imagebuffer.number;
	msg.header.stamp.sec=diff.total_seconds();
	msg.header.stamp.nsec=1000 * (diff.total_microseconds() % 1000000);
	msg.header.frame_id= instance->nameSpace.substr(1,instance->nameSpace.length()-1) + "_camera_rgb_optical_link";
	msg.height=instance->helper->getHeight();
	msg.width=instance->helper->getWidth();
	msg.encoding="bgr8";
	msg.step=instance->helper->getWidth()*3;
	msg.data.resize(msg.height*msg.step);
	memcpy((char*)(&msg.data[0]),&imagebuffer.buffer[0],msg.height*msg.step);
	instance->frame_pub.publish(msg);

	if (instance->recording || first) {	
		if (instance->bufferfill<63) {
			instance->timesfile << imagebuffer.number << '\t' << diff.total_microseconds() << std::endl;

			// encode video
			transfer.number = imagebuffer.number;
			transfer.size = 3*instance->helper->getWidth()*instance->helper->getHeight();
			instance->bufferfill++;
			instance->bufferwrite++;
			//std::memcpy(instance->bufferqueue[instance->bufferwrite++],imagebuffer.buffer,transfer.size);
			instance->bufferwrite=instance->bufferwrite%64;
			write(instance->pipe[1],&transfer,sizeof(transfer));
			//write(instance->pipe[1],imagebuffer.buffer,transfer.size);
			first = 0;
		} else {
			std::cerr << "Videofile lost frame!" << std::endl;
		}
	}
	
	diff=imagebuffer.timestamp-instance->start;

        if (frames%instance->framerate==0) {
		rosinfoPrint( static_cast<std::ostringstream*>( &(std::ostringstream() <<  "Frame: " << frames << "/" << imagebuffer.number << ", Average rate: " << (double)(1000000.*frames)/diff.total_microseconds() << ", Skipped frames: " << (imagebuffer.number-frames) << std::endl ))->str().c_str());
	}
	
	frames++;
    }
    //ros::waitForShutdown();

    // join the other threads
    return 0;
}

boost::posix_time::ptime *videograb::getStart(void)
{
    return &instance->start;
}

boost::posix_time::ptime *videograb::getCurrent(void)
{
    return &instance->current;
}

void videograb::rosinfoPrint(const char *bla)
{
    instance->ROSinfo_Mutex.lock();
    ROS_INFO("%s", bla);
    instance->ROSinfo_Mutex.unlock();
}
}
