#include "Include/OpenNI.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

    struct Kinect{
openni::Device device;        // Software object for the physical device i.e. 
openni::VideoStream ir;       // IR VideoStream Class Object
openni::VideoFrameRef irf;    //IR VideoFrame Class Object
openni::VideoMode vmode;      // VideoMode Object
openni::Status rc;
};
class StreamListener : public openni::VideoStream::NewFrameListener
{
public:
  StreamListener(struct Kinect *kinect_input)
  {
    kinect=kinect_input;
  }
  virtual void onNewFrame(openni::VideoStream& stream)
  {
    kinect->rc = (kinect->ir).readFrame(&(kinect->irf));
    (kinect->vmode) = (kinect->ir).getVideoMode();
    const uint16_t* imgBuf = (const uint16_t*)(kinect->irf).getData(); 
    h=(kinect->irf).getHeight(); 
    w=(kinect->irf).getWidth();
    frame.create(h, w, CV_16U); // Create the OpenCV Mat Matrix Class Object 
                                // to receive the IR VideoFrames
    memcpy(frame.data, imgBuf, h*w*sizeof(uint16_t)); 
                      // Copy the ir data from memory imgbuf -> frame.data 
                      // using memcpy (a string.h) function
    frame.convertTo(frame, CV_8U); 
                      // OpenCV displays 8-bit data (I'm not sure why?) 
                      // So, convert from 16-bit to 8-bit
    cv::namedWindow("ir", 1);   // Create a named window
    cv::imshow("ir", frame);
      }
    private:
      struct Kinect *kinect;
      cv::Mat frame;
      int h,w;
    };


int main(int argc, char const *argv[])
{
  struct Kinect myKinect;
  StreamListener stream(&myKinect);
  myKinect.rc = openni::STATUS_OK;
  myKinect.rc = openni::OpenNI::initialize();  
    if (myKinect.rc != openni::STATUS_OK)
  {
    printf("Cannot initialize the API\n");
    return FALSE;
  }
  myKinect.rc = (myKinect.device).open(openni::ANY_DEVICE);
    if (myKinect.rc != openni::STATUS_OK)
  {
    printf("Cannot find any Kinect\n");
    return FALSE;
  }
  myKinect.rc = (myKinect.ir).create(myKinect.device, openni::SENSOR_COLOR);
  //VideoStream **tab;
  //*tab = &(myKinect.ir);
  if (myKinect.rc != openni::STATUS_OK)
  {
    printf("Cannot initialize the Kinect");
    return FALSE;
  }
  myKinect.ir.addNewFrameListener(&stream);
  myKinect.rc = myKinect.ir.start(); 
  if (myKinect.rc != openni::STATUS_OK)
  {
    printf("Cannot start frame grabbing");
    return FALSE;
  }
  while(1)
  {
    char key = cv::waitKey(10);
    if(key==27) break;
    //OpenNI::waitForAnyStream(tab,1,NULL);

  }
  return 0;
}