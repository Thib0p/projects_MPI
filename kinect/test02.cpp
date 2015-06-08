#include "Include/OpenNI.h"
#include <visp/vpDisplay.h>
//#include <visp/vpDisplayX.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpServo.h>
#include <visp/vpSimulatorCamera.h>
#include <visp/vpImage.h>
#include <visp/vpImageConvert.h>
#include <visp/vpPoint.h>
#include <visp/vpException.h>
#include <visp/vpDot2.h>
#include <visp/vpTemplateTrackerSSDInverseCompositional.h>
#include <visp/vpTemplateTrackerWarpHomography.h>

int opticaldone=0;
struct Kinect{
openni::Device device;        // Software object for the physical device i.e. 
openni::VideoStream ir,depth;       // IR VideoStream Class Object
openni::VideoFrameRef irf,depthr;    //IR VideoFrame Class Object
openni::VideoMode vmode;      // VideoMode Object
openni::Status rc;
};
struct Visp{
  vpImage<unsigned char> I ;
  vpImage<vpRGBa> Irgb;
  vpImagePoint p;
  vpDot2 tracker;
  int pointSet;
  cv::Point pointcv;
  std::vector<cv::Point2f> features_prev, features_next;
  std::vector<uchar> status;
  const openni::DepthPixel* depth;
  bool depth_ready;
  vpTemplateTrackerWarpHomography warp;
  vpTemplateTrackerSSDInverseCompositional tracker(&warp);
  //vpDisplayX display;
};
void on_mouse( int e, int x, int y, int d, void *ptr )
{
  if(e == cv::EVENT_LBUTTONDOWN)
  {
    Visp *v = (Visp*)ptr;
    //if((*v).pointSet==0)
    //{
    
    //(*v).p.set_ij(y,x);
    (*v).pointSet=1;
    /*(*v).tracker.initTracking((*v).I,(*v).p);
    (*v).p = (*v).tracker.getCog();
    (*v).pointcv.x = floor((*v).p.get_i());
    (*v).pointcv.y = floor((*v).p.get_j());*/
    (*v).features_next.push_back(cv::Point2f(x, y));
    printf("x: %d               y: %d\n",x,y);
  //}
  }
}

class StreamListener : public openni::VideoStream::NewFrameListener
{
public:
  StreamListener(struct Kinect *kinect_input,struct Visp *visp_input)
  {
    kinect=kinect_input;
    visp=visp_input;
    initialized=0;
  }
  virtual void onNewFrame(openni::VideoStream& stream)
  {
    kinect->rc = (kinect->ir).readFrame(&(kinect->irf));
    const openni::RGB888Pixel* imgBuf = (const openni::RGB888Pixel*)(kinect->irf).getData(); 
    h=(kinect->irf).getHeight(); 
    w=(kinect->irf).getWidth();
    if(initialized==0)
    {
    initFromPoints   ( visp->I,
    const std::vector< vpImagePoint > &   points2D_list,
    const std::vector< vpPoint > &    points3D_list 
  )   
    }
    if(initialized==1)
    {
      frame_old = frame_gray.clone();
    }
    visp->features_prev = visp->features_next;
    frame.create(h, w, CV_8UC3); 
    memcpy(frame.data, imgBuf, 3*h*w*sizeof(uint8_t)); 
                      // Copy the ir data from memory imgbuf -> frame.data 
                      // using memcpy (a string.h) function
    frame.convertTo(frame, CV_8U); 
    cv::cvtColor(frame,frame,CV_BGR2RGB);
    for (int i = 0; i < h; ++i)
    {
      for (int j = 0; j < w; ++j)
      {
       (visp->Irgb)[i][j].R = frame.data[3*(w*i +j)+0];
       (visp->Irgb)[i][j].G = frame.data[3*(w*i +j)+1];
       (visp->Irgb)[i][j].B = frame.data[3*(w*i +j)+2];
     }
   }

   cvtColor( frame, frame_gray, CV_BGR2GRAY ); 
   if(visp->pointSet==1)
   {
    try
    {
      if((visp->features_prev).size()>0)
     cv::calcOpticalFlowPyrLK(
      frame_old, frame_gray, // 2 consecutive images
      visp->features_prev, // input point positions in first im
      visp->features_next, // output point positions in the 2nd
      visp->status,    // tracking success
      err      // tracking error
      );
     opticaldone=1;
   }
   catch(...)
   {

   }
 }

      /*cv::goodFeaturesToTrack(frame_gray,// the image 
      visp->features_next,   // the output detected features
      10,  // the maximum number of features 
      0.2,     // quality level
      10     // min distance between two features
    );*/

    for (int i = 0; i < (visp->features_next).size(); ++i)
    {
      cv::Point a;
      a.x=floor(visp->features_next[i].x);
      a.y=floor(visp->features_next[i].y);
      circle(frame,a,3, cv::Scalar( 0, 0, 255 ),-1, 8 );
      if(opticaldone==1){
      if(visp->status[i]==1)
      {
        if(visp->depth_ready==1)
          printf("%d\n",visp->depth[a.y*w+a.x]);
      }
      else
      {
        (visp->features_next).erase((visp->features_next).begin()+i);
        printf("This point can not be tracked");
      }
    }  // Create a named window
  }
    cv::imshow("Color", frame);
    cv::setMouseCallback("Color",on_mouse, visp); 
    initialized=1;
  }
private:
  struct Kinect *kinect;
  struct Visp *visp;
  cv::Mat frame,frame_old,frame_gray;
  int h,w;
  int initialized;
  cv::Mat err;
};
















class StreamListenerd : public openni::VideoStream::NewFrameListener
{
public:
  StreamListenerd(struct Kinect *kinect_input,struct Visp *visp_input)
  {
    kinect=kinect_input;
    visp=visp_input;
    initialized=0;
  }
  virtual void onNewFrame(openni::VideoStream& stream)
  {
    kinect->rc = (kinect->depth).readFrame(&(kinect->depthr));
    visp->depth = (const openni::DepthPixel*)(kinect->depthr).getData(); 
    visp->depth_ready=1;
    h=(kinect->depthr).getHeight(); 
    w=(kinect->depthr).getWidth();

                      // Copy the ir data from memory imgbuf -> frame.data 
                      // using memcpy (a string.h) function
  }
private:
  struct Kinect *kinect;
  struct Visp *visp;
  cv::Mat frame,frame_old,frame_gray;
  int h,w;
  int initialized;
  cv::Mat err;
  std::vector<uchar> status;
};












int main(int argc, char const *argv[])
{
  struct Kinect myKinect;
  struct Visp myVisp;
  cv::namedWindow("Color", 1); 
  myVisp.tracker.setSampling(2, 2);
  myVisp.tracker.setLambda(0.001);
  myVisp.tracker.setIterationMax(200);
  myVisp.tracker.setPyramidal(2, 1);

  //VideoCapture capture( CV_CAP_OPENNI );
  myVisp.pointSet=0;
  myVisp.Irgb.resize(480,640);
  vpImage<unsigned char> I ;
  vpImage<vpRGBa> Irgb;
  StreamListener stream(&myKinect,&myVisp);
  StreamListenerd streamDepth(&myKinect,&myVisp);
  myKinect.rc = openni::STATUS_OK;
  myKinect.rc = openni::OpenNI::initialize();  
  if (myKinect.rc != openni::STATUS_OK)
  {
    printf("Cannot initialize the API\n");
    printf("%s",openni::OpenNI::getExtendedError());
    return FALSE;
  }
  myKinect.rc = (myKinect.device).open(openni::ANY_DEVICE);
  if (myKinect.rc != openni::STATUS_OK)
  {
    printf("Cannot find any Kinect\n");
    return FALSE;
  }

  myKinect.rc = (myKinect.ir).create(myKinect.device, openni::SENSOR_COLOR);
  myKinect.rc = (myKinect.depth).create(myKinect.device, openni::SENSOR_DEPTH);
  (myKinect.vmode)  = (myKinect.ir).getVideoMode();
  (myKinect.vmode).setResolution(640,480);
  (myKinect.vmode).setFps(30);
  (myKinect.ir).setVideoMode(myKinect.vmode);


  (myKinect.vmode)  = (myKinect.depth).getVideoMode();
  (myKinect.vmode).setResolution(640,480);
  (myKinect.vmode).setFps(30);
  (myKinect.depth).setVideoMode(myKinect.vmode);

  if (myKinect.rc != openni::STATUS_OK)
  {
    printf("Cannot initialize the Kinect");
    return FALSE;
  }
  (myKinect.ir).addNewFrameListener(&stream);
  (myKinect.depth).addNewFrameListener(&streamDepth);
  myKinect.rc = (myKinect.ir).start(); 
  myKinect.rc = (myKinect.depth).start(); 
  if (myKinect.rc != openni::STATUS_OK)
  {
    printf("Cannot start frame grabbing");
    return FALSE;
  }
  while(1)
  {
    char key = cv::waitKey(10);
    if(key==27) break;

  }
  openni::OpenNI::shutdown();
  return 0;
}