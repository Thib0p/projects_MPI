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
int main(int argc, char** argv)
{
#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100) || defined(VISP_HAVE_FFMPEG)
  std::string videoname = "bruegel.mpg";
  for (int i=0; i<argc; i++) {
    if (std::string(argv[i]) == "--videoname")
      videoname = std::string(argv[i+1]);
    else if (std::string(argv[i]) == "--help") {
      std::cout << "\nUsage: " << argv[0] << " [--name <video name>] [--help]\n" << std::endl;
      return 0;
    }
  }
  std::cout << "Video name: " << videoname << std::endl;
  vpImage<unsigned char> I;
  vpVideoReader g;
  g.setFileName(videoname);
  g.open(I);
#if defined(VISP_HAVE_X11)
  vpDisplayX display;
#elif defined(VISP_HAVE_GDI)
  vpDisplayGDI display;
#elif defined(VISP_HAVE_OPENCV)
  vpDisplayOpenCV display;
#else
  std::cout << "No image viewer is available..." << std::endl;
#endif
  display.init(I, 100, 100, "Template tracker");
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
    g.acquire(I);
    vpDisplay::display(I);
    tracker.track(I);
    vpColVector p = tracker.getp();
    vpHomography H = warp.getHomography(p);
    std::cout << "Homography: \n" << H << std::endl;
    tracker.display(I, vpColor::red);
    if (vpDisplay::getClick(I, false))
      break;
    vpDisplay::flush(I);
        char key = cv::waitKey(10);
    if(key==27) break;
  }
#else
  (void)argc;
  (void)argv;
#endif
}