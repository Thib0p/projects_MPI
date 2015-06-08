#include "Include/OpenNI.h"
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpVideoReader.h>
#include <visp/vpTemplateTrackerSSDInverseCompositional.h>
#include <visp/vpTemplateTrackerWarpHomography.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
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
	vpTemplateTrackerWarpHomography warp;
	vpTemplateTrackerSSDInverseCompositional tracker;
	std::vector< vpImagePoint >  points2D_list;
	vpColVector    newPoints;
  //vpDisplayX display;
};


/*class StreamListener : public openni::VideoStream::NewFrameListener
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
		vpImageConvert::convert((visp->Irgb), (visp->I));
	}
private:
	struct Kinect *kinect;
	struct Visp *visp;
	cv::Mat frame,frame_old,frame_gray;
	int h,w;
	int initialized;
	cv::Mat err;
};*/




int main(int argc, char** argv)
{
	int h,w;
	struct Kinect myKinect;
  	struct Visp myVisp;
#if defined(VISP_HAVE_X11)
	vpDisplayX display;
#elif defined(VISP_HAVE_GDI)
	vpDisplayGDI display;
#elif defined(VISP_HAVE_OPENCV)
	vpDisplayOpenCV display;
#else
	std::cout << "No image viewer is available..." << std::endl;
#endif
	kinect->rc = (kinect->ir).readFrame(&(kinect->irf));
	const openni::RGB888Pixel* imgBuf = (const openni::RGB888Pixel*)(kinect->irf).getData(); 
	h=(kinect->irf).getHeight(); 
	w=(kinect->irf).getWidth();
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
	vpImageConvert::convert((visp->Irgb), (visp->I));
	display.init(I, 100, 100, "My Plane tracker");
	vpDisplay::display(I);
	vpDisplay::flush(I);
	vpTemplateTrackerWarpHomography warp;
	vpTemplateTrackerSSDInverseCompositional tracker(&warp);
	tracker.setSampling(2, 2);
	tracker.setLambda(0.001);
	tracker.setIterationMax(200);
	tracker.setPyramidal(2, 1);
	tracker.initClick(I);
	while(1){
		/*g.acquire(I);
		vpDisplay::display(I);
		tracker.track(I);
		vpColVector p = tracker.getp();
		vpHomography H = warp.getHomography(p);
		std::cout << "Homography: \n" << H << std::endl;
		tracker.display(I, vpColor::red);
		if (vpDisplay::getClick(I, false))
			break;
		vpDisplay::flush(I);*/
	char key = cv::waitKey(10);
    	if(key==27) break;
	}
}