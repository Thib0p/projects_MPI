#include "Include/OpenNI.h"
#include <visp/vpDisplay.h>
#include <visp/vpDisplayX.h>
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
#include <visp/vpTime.h>
#include <visp/vpVideoWriter.h>


int initialized2=0;
int opticaldone=0;
struct Kinect{
openni::Device device;        // Software object for the physical device i.e. 
openni::VideoStream ir,depth;       // IR VideoStream Class Object
openni::VideoFrameRef irf,depthr;    //IR VideoFrame Class Object
openni::VideoMode vmode;      // VideoMode Object
int rc;
};
struct Visp{
  vpImage<unsigned char> I ;
  vpImage<vpRGBa> Irgb;
  vpImagePoint p;
  int pointSet;
  cv::Point pointcv;
  const openni::DepthPixel* depth;
  bool depth_ready;
  vpTemplateTrackerWarpHomography warp;
  vpTemplateTrackerSSDInverseCompositional *tracker;
  std::vector< vpImagePoint >    points2D_list;
  vpColVector    newPoints;
  std::vector< vpPoint >     points3D_list; 
  vpDisplayX display;
  std::vector<cv::Point2f> features_prev, features_next;
  std::vector<uchar> status;
  vpVideoWriter writer;
};


class StreamListener : public openni::VideoStream::NewFrameListener
{
public:
  StreamListener(struct Kinect *kinect_input,struct Visp *visp_input)
  {
    kinect=kinect_input;
    visp=visp_input;
    initialized=0;
    (visp->writer).setBitRate(1000000);
#  if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(54,51,110) // libavcodec 54.51.100
    (visp->writer).setCodec(CODEC_ID_MPEG2VIDEO);
#  else
    (visp->writer).setCodec(AV_CODEC_ID_MPEG2VIDEO);
#  endif
//    (visp->writer).setCodec(AV_CODEC_ID_MPEG2VIDEO);
    (visp->writer).setFileName("tracking_planes.mpg");
  }
  virtual void onNewFrame(openni::VideoStream& stream)
  {
    vpImagePoint buf;
    kinect->rc = (kinect->ir).readFrame(&(kinect->irf));
    const openni::RGB888Pixel* imgBuf = (const openni::RGB888Pixel*)(kinect->irf).getData(); 
    h=(kinect->irf).getHeight(); 
    w=(kinect->irf).getWidth();
    



    /*After the plane that will be tracked is defined, the tracking is initialized*/
    if(visp->pointSet==4)
    {
      if(initialized2==0)
      {
        for (int i = 0; i < 3; ++i)
        {
          (visp->points2D_list).push_back(vpImagePoint((visp->features_next)[i].y,(visp->features_next)[i].x));
        }
        (visp->points2D_list).push_back(vpImagePoint((visp->features_next)[2].y,(visp->features_next)[2].x));
        (visp->points2D_list).push_back(vpImagePoint((visp->features_next)[3].y,(visp->features_next)[3].x));
        (visp->points2D_list).push_back(vpImagePoint((visp->features_next)[0].y,(visp->features_next)[0].x));
        (visp->tracker)->initFromPoints(visp->I,
          visp->points2D_list 
          );
        initialized2=1;   
      }
    }








/* As soon as an image is available from the kineect, it is converted to a gray one */
    if(initialized==1 && initialized2 ==0)
    {
      frame_old = frame_gray.clone();
    }


    frame.create(h, w, CV_8UC3); 
    memcpy(frame.data, imgBuf, 3*h*w*sizeof(uint8_t)); 
                      // Copy the ir data from memory imgbuf -> frame.data 
                      // using memcpy (a string.h) function
    frame.convertTo(frame, CV_8U); 
    cv::cvtColor(frame,frame,CV_BGR2RGB);
   /* No need to convert the gray image once the plane has been defined so the conversion is desactivated as soon as the 4 points are defined */
   if(initialized2 ==0)
   {
    cvtColor( frame, frame_gray, CV_BGR2GRAY ); 
  }
  if(visp->pointSet<4)
  {
    try
    {

      visp->features_prev = visp->features_next;
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
/* the clicked points are shown on the image*/
  if(visp->pointSet<4)
  {
  for (int i = 0; i < (visp->features_next).size(); ++i)
  {
    cv::Point a;
    a.x=floor((visp->features_next)[i].x);
    a.y=floor((visp->features_next)[i].y);
    circle(frame,a,3, cv::Scalar( 255, 255, 255 ),-1, 8 );
    /*if(opticaldone==1){
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
    } */
    }
  }
        for (int i = 0; i < h; ++i)
    {
      for (int j = 0; j < w; ++j)
      {
       (visp->Irgb)[i][j].R = frame.data[3*(w*i +j)+0];
       (visp->Irgb)[i][j].G = frame.data[3*(w*i +j)+1];
       (visp->Irgb)[i][j].B = frame.data[3*(w*i +j)+2];
     }
   }
   vpImageConvert::convert((visp->Irgb), (visp->I));
    if(initialized==0)
    {
      visp->display.init(visp->I, 100, 100, "Tracker");
      visp->writer.open(visp->I);
    }

  vpDisplay::display(visp->I);


      if(initialized2==1)
      {
        (visp->tracker)->track(visp->I);
            vpColVector p = (visp->tracker)->getp();
    vpHomography H = visp->warp.getHomography(p);
    std::cout << "Homography: \n" << H << std::endl;
        (visp->tracker)->display(visp->I, vpColor::red);

      }

  
  (visp->writer).saveFrame(visp->I);
  vpDisplay::flush(visp->I);
    if (vpDisplay::getClick(visp->I,buf, false))
    {
    visp->pointSet++;
    visp->features_next.push_back(cv::Point2f(buf.get_u(), buf.get_v()));
    printf("x: %f               y: %f\n",buf.get_u(),buf.get_v());
    }
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
  myVisp.tracker = new vpTemplateTrackerSSDInverseCompositional(&(myVisp.warp));
  myVisp.tracker->setSampling(2, 2);
  myVisp.tracker->setLambda(0.001);
  myVisp.tracker->setIterationMax(200);
  myVisp.tracker->setPyramidal(2, 1);
  myVisp.points3D_list.push_back(vpPoint());
  myVisp.points3D_list.push_back(vpPoint());
  myVisp.points3D_list.push_back(vpPoint());
  myVisp.points3D_list.push_back(vpPoint());
  myVisp.points3D_list[0].setWorldCoordinates (0,0,0);
  myVisp.points3D_list[1].setWorldCoordinates (1,0,0);
  myVisp.points3D_list[2].setWorldCoordinates (1,1,0);
  myVisp.points3D_list[3].setWorldCoordinates (0,1,0);
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