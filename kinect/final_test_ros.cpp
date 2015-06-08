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
#include <visp/vpTemplateTrackerSSDInverseCompositional.h>
#include <visp/vpTemplateTrackerWarpHomography.h>
#include <visp/vpTime.h>
#include <visp/vpVideoWriter.h>
#include <visp/vpDot2.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpPose.h>


void toQuat(double *x, double *y, double *z, double *w, vpHomogeneousMatrix &cMo);
void computePose(std::vector<vpPoint> &point, const std::vector<vpDot2> &dot,
                 const vpCameraParameters &cam, int init, vpHomogeneousMatrix &cMo);
int initialized2=0;
int init_pose=0;
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
  std::vector<vpPoint> realWorldpoints;
  vpHomogeneousMatrix cMo;
  vpCameraParameters *cam;
  std::vector<vpDot2> imageDots;
  double x,y,z,w;
  // Instantiate and get the reference zone
vpTemplateTrackerZone zone_ref;
// Instantiate a warped zone
vpTemplateTrackerZone zone_warped;
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
        visp->zone_ref = visp->tracker->getZoneRef();
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
  visp->warp.warpZone(visp->zone_ref, p, visp->zone_warped);
  vpTemplateTrackerTriangle triangle0,triangle1;
  visp->zone_warped.getTriangle(0, triangle0);
  visp->zone_warped.getTriangle(1, triangle1);
  std::vector<vpImagePoint> corner0,corner1;
  triangle0.getCorners( corner0 );
  triangle1.getCorners( corner1 );
  if(init_pose==0)
  {
    visp->imageDots.push_back(vpDot2(corner0[0]));
    visp->imageDots.push_back(vpDot2(corner0[1]));
    visp->imageDots.push_back(vpDot2(corner0[2]));
    visp->imageDots.push_back(vpDot2(corner1[1]));
  }
  visp->imageDots[0] = corner0[0];
  visp->imageDots[1
    ] = corner0[1];
  visp->imageDots[2] = corner0[2];
  visp->imageDots[3] = corner1[1];
  computePose(visp->realWorldpoints, visp->imageDots, *(visp->cam), init_pose, visp->cMo);
  toQuat(&(visp->x),&(visp->y),&(visp->z),&(visp->w),visp->cMo);
  vpDisplay::displayFrame(visp->I, visp->cMo, *(visp->cam), 0.05, vpColor::none, 3);

  (visp->tracker)->display(visp->I, vpColor::red);
  init_pose=1;

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
  double L = 0.265,l=0.20;
  myVisp.tracker = new vpTemplateTrackerSSDInverseCompositional(&(myVisp.warp));
  myVisp.tracker->setSampling(2, 2);
  myVisp.tracker->setLambda(0.001);
  myVisp.tracker->setIterationMax(200);
  myVisp.tracker->setPyramidal(2, 1);
  myVisp.points3D_list.push_back(vpPoint());
  myVisp.points3D_list.push_back(vpPoint());
  myVisp.points3D_list.push_back(vpPoint());
  myVisp.points3D_list.push_back(vpPoint());
  myVisp.realWorldpoints.push_back(vpPoint());
  myVisp.realWorldpoints.push_back(vpPoint());
  myVisp.realWorldpoints.push_back(vpPoint());
  myVisp.realWorldpoints.push_back(vpPoint());
  myVisp.realWorldpoints[0].setWorldCoordinates (-l/2, -L/2, 0);
  myVisp.realWorldpoints[1].setWorldCoordinates ( l/2, -L/2, 0);
  myVisp.realWorldpoints[2].setWorldCoordinates ( l/2,  L/2, 0);
  myVisp.realWorldpoints[3].setWorldCoordinates (-l/2, L/2, 0);
  myVisp.cam = new vpCameraParameters(525, 525, 320, 240);
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

void computePose(std::vector<vpPoint> &point, const std::vector<vpDot2> &dot,
                 const vpCameraParameters &cam, int init, vpHomogeneousMatrix &cMo)
{
  vpPose pose;     double x=0, y=0;
  for (unsigned int i=0; i < point.size(); i ++) {
    vpImagePoint buf(dot[i].getCog());
   // printf("x:%f y:%f\n",buf.get_j(), buf.get_i());
    vpPixelMeterConversion::convertPoint(cam, dot[i].getCog(), x, y);
    point[i].set_x(x);
    point[i].set_y(y);
    pose.addPoint(point[i]);
  }
  if (init == 0) {
    vpHomogeneousMatrix cMo_dem;
    vpHomogeneousMatrix cMo_lag;
  pose.computePose(vpPose::DEMENTHON, cMo_dem);
  pose.computePose(vpPose::LAGRANGE, cMo_lag);
  double residual_dem = pose.computeResidual(cMo_dem);
    double residual_lag = pose.computeResidual(cMo_lag);
    if (residual_dem < residual_lag)
      cMo = cMo_dem;
    else
      cMo = cMo_lag;
  }
  pose.computePose(vpPose::VIRTUAL_VS, cMo);
  cMo.vpMatrix::print (std::cout, 4);
;
}

void toQuat(double *x, double *y, double *z, double *w, vpHomogeneousMatrix &cMo)
{
  double trace =cMo[0][0]+cMo[1][1]+cMo[2][2];
  double S;
  if(trace>=0)
  {
S=0.5/sqrt(trace);
*x=(cMo[2][1]-cMo[1][2])*S;
*y=(cMo[0][2]-cMo[2][0])*S;
*z=(cMo[1][0]-cMo[0][1])*S;
*w=0.25/S;
  }
  int max;
  for (int i = 1; i < 3; ++i)
  {
    if(cMo[i][i]>=cMo[max][max])
    {
      max=i;
    }
  }
  switch(max)
  {
    case 0:
    S=sqrt(1+cMo[0][0]-cMo[1][1]-cMo[2][2])*2;
    *x=0.25*S;
    *y =(cMo[0][1]+cMo[1][0])/S;
    *z = (cMo[0][2]+cMo[2][0])/S;
    *w = (cMo[1][2]+cMo[2][1])/S;
    break;
    case 1:
    S=sqrt(1-cMo[0][0]+cMo[1][1]-cMo[2][2])*2;
    *x=(cMo[0][1]+cMo[1][0])/S;
    *y =0.25*S;
    *z = (cMo[1][2]+cMo[2][1])/S;
    *w = (cMo[0][2]+cMo[2][0])/S;
    break;
    case 2:
    S=sqrt(1-cMo[0][0]-cMo[1][1]+cMo[2][2])*2;
    *x=(cMo[0][2]+cMo[2][0])/S;
    *y =(cMo[1][2]+cMo[2][1])/S;
    *z = 0.25*S;
    *w = (cMo[0][1]+cMo[1][0])/S;
    break;
  }
}