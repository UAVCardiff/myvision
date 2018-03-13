//! \livecam-apriltag-detector-working.cpp
//! [Include]

#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/detection/vpDetectorAprilTag.h>
//! [Include]
#include <visp3/gui/vpDisplayOpenCV.h>
#ifdef VISP_HAVE_V4L2
#include <visp3/sensor/vpV4l2Grabber.h>
#endif
#include <visp3/vision/vpPose.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/gui/vpDisplayX.h>

void computePose(std::vector<vpPoint> &point, const std::vector<vpImagePoint> &ip, const vpCameraParameters &cam,
                 bool init, vpHomogeneousMatrix &cMo)
{
  vpPose pose;
  double x = 0, y = 0;
  for (unsigned int i = 0; i < point.size(); i++) {
    vpPixelMeterConversion::convertPoint(cam, ip[i], x, y);
    point[i].set_x(x);
    point[i].set_y(y);
    pose.addPoint(point[i]);
  }

  if (init == true) {
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
}


int main(int argc, const char **argv)
{
#if defined(VISP_HAVE_APRILTAG) && defined(VISP_HAVE_OPENCV) &&  \
    (defined(VISP_HAVE_V4L2) || defined(VISP_HAVE_OPENCV))
  
  int opt_device = 1;
  vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
  vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
  //double tagSize = 0.112;
  double tagSize = 0.264;
  float quad_decimate = 1.0;
  int nThreads = 1;
  std::string intrinsic_file = "camera.xml";
  std::string camera_name = "Camera";
  bool display_tag = false;

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--pose_method" && i + 1 < argc) {
      poseEstimationMethod = (vpDetectorAprilTag::vpPoseEstimationMethod)atoi(argv[i + 1]);
    } else if (std::string(argv[i]) == "--tag_size" && i + 1 < argc) {
      tagSize = atof(argv[i + 1]);
    } else if (std::string(argv[i]) == "--input" && i + 1 < argc) {
      opt_device = atoi(argv[i + 1]);
    } else if (std::string(argv[i]) == "--quad_decimate" && i + 1 < argc) {
      quad_decimate = (float)atof(argv[i + 1]);
    } else if (std::string(argv[i]) == "--nthreads" && i + 1 < argc) {
      nThreads = atoi(argv[i + 1]);
    } else if (std::string(argv[i]) == "--intrinsic" && i + 1 < argc) {
      intrinsic_file = std::string(argv[i + 1]);
    } else if (std::string(argv[i]) == "--camera_name" && i + 1 < argc) {
      camera_name = std::string(argv[i + 1]);
    } else if (std::string(argv[i]) == "--display_tag") {
      display_tag = true;
    } else if (std::string(argv[i]) == "--tag_family" && i + 1 < argc) {
      tagFamily = (vpDetectorAprilTag::vpAprilTagFamily)atoi(argv[i + 1]);
    } else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "Usage: " << argv[0]
                << std::endl << " [--input <camera input>]" 
		<< std::endl << " [--tag_size <tag_size in m>]" 
		<< std::endl << " [--quad_decimate <quad_decimate>]" 
		<< std::endl << " [--nthreads <nb>]" 
		<< std::endl << " [--intrinsic <intrinsic file>]" 
		<< std::endl << " [--camera_name <camera name>]" 
		<< std::endl << " [--pose_method <method>] \n\t0: HOMOGRAPHY_VIRTUAL_VS \n\t1: DEMENTHON_VIRTUAL_VS \n\t2: LAGRANGE_VIRTUAL_VS \n\t3: BEST_RESIDUAL_VIRTUAL_VS"
                << std::endl << " [--tag_family <family>] \n\t0: TAG_36h11 \n\t1: TAG_36h10 \n\t2: TAG_36ARTOOLKIT \n\t3: TAG_25h9 \n\t4: TAG_25h7 \n\t5: TAG_16h5"
                << std::endl << " [--display_tag]" 
		<< std::endl << " [--help]"
                << std::endl;
      return EXIT_SUCCESS;
    }
  }

  vpCameraParameters cam;
  cam.initPersProjWithoutDistortion(625.6486569324, 620.9115718852, 322.4685677603, 237.7820607971);
#ifdef VISP_HAVE_XML2
  vpXmlParserCamera parser;
  if (!intrinsic_file.empty() && !camera_name.empty())
    parser.parse(cam, intrinsic_file, camera_name, vpCameraParameters::perspectiveProjWithoutDistortion);
#endif

  std::cout << "cam:\n" << cam << std::endl;
  std::cout << "poseEstimationMethod: " << poseEstimationMethod << std::endl;
  std::cout << "tagFamily: " << tagFamily << std::endl;


  try {
    vpImage<unsigned char> I;
    
#if defined(VISP_HAVE_V4L2)
  vpV4l2Grabber g;
  std::ostringstream device;
  device << "/dev/video" << opt_device;
  g.setDevice(device.str());
  g.setScale(1);
  //g.setWidth(1280);
  //g.setHeight(720);
  g.acquire(I);
#elif defined(VISP_HAVE_OPENCV)
  cv::VideoCapture cap(opt_device); // open the default camera
  if (!cap.isOpened()) {            // check if we succeeded
    std::cout << "Failed to open the camera" << std::endl;
    return EXIT_FAILURE;
  }
  cv::Mat frame;
  cap >> frame; // get a new frame from camera
  vpImageConvert::convert(frame, I);
#endif

  // 3D model of the AprilTag: here we consider a 26.4cm by 26.4cm AprilTag
  std::vector<vpPoint> point;
  point.push_back(vpPoint(-0.132, -0.132, 0)); // AprilTag point 0 3D coordinates in plane Z=0
  point.push_back(vpPoint(0.132, -0.132, 0)); // AprilTag point 1 3D coordinates in plane Z=0
  point.push_back(vpPoint(0.132, 0.132, 0)); // AprilTag point 2 3D coordinates in plane Z=0
  point.push_back(vpPoint(-0.132, 0.132, 0)); // AprilTag point 3 3D coordinates in plane Z=0
  

  /*// 3D model of the AprilTag: here we consider a 11.2cm by 11.2cm AprilTag
  std::vector<vpPoint> point;
  point.push_back(vpPoint(-0.056, -0.056, 0)); // AprilTag point 0 3D coordinates in plane Z=0
  point.push_back(vpPoint(0.056, -0.056, 0)); // AprilTag point 1 3D coordinates in plane Z=0
  point.push_back(vpPoint(0.056, 0.056, 0)); // AprilTag point 2 3D coordinates in plane Z=0
  point.push_back(vpPoint(-0.056, 0.056, 0)); // AprilTag point 3 3D coordinates in plane Z=0*/

  vpHomogeneousMatrix cMo;
  bool init = true;

#if defined(VISP_HAVE_X11)
  vpDisplayX d(I);
#elif defined(VISP_HAVE_OPENCV)
  vpDisplayOpenCV d(I);
#endif

  vpDetectorBase *detector = new vpDetectorAprilTag(tagFamily);

  dynamic_cast<vpDetectorAprilTag *>(detector)->setAprilTagQuadDecimate(quad_decimate);
  dynamic_cast<vpDetectorAprilTag *>(detector)->setAprilTagPoseEstimationMethod(poseEstimationMethod);
  dynamic_cast<vpDetectorAprilTag *>(detector)->setAprilTagNbThreads(nThreads);
  dynamic_cast<vpDetectorAprilTag *>(detector)->setDisplayTag(display_tag);

  std::vector<double> time_vec;
  for (;;) {
#if defined(VISP_HAVE_V4L2)
    g.acquire(I);
#elif defined(VISP_HAVE_OPENCV)
    cap >> frame; // get a new frame from camera
    vpImageConvert::convert(frame, I);
#endif

    vpDisplay::display(I);

    //Display Grid Lines
    vpDisplay::displayCross(I, I.getHeight() / 2, I.getWidth() / 2, I.getWidth(), vpColor::blue, 1);
    vpDisplay::displayCross(I, I.getHeight() / 4, I.getWidth() / 4, I.getWidth(), vpColor::red, 1);
    vpDisplay::displayCross(I, I.getHeight() * 3 / 4, I.getWidth() / 4, I.getWidth(), vpColor::red, 1);
    vpDisplay::displayCross(I, I.getHeight() / 4, I.getWidth() * 3 / 4, I.getWidth(), vpColor::red, 1);
    vpDisplay::displayCross(I, I.getHeight() * 3 / 4, I.getWidth() * 3 / 4, I.getWidth(), vpColor::red, 1);
    vpDisplay::displayLine(I, 0, I.getWidth() / 8, I.getHeight(), I.getWidth() / 8, vpColor::red, 1);
    vpDisplay::displayLine(I, 0, I.getWidth() * 3 / 8, I.getHeight(), I.getWidth() * 3 / 8, vpColor::red, 1);
    vpDisplay::displayLine(I, 0, I.getWidth() * 5 / 8, I.getHeight(), I.getWidth() * 5 / 8, vpColor::red, 1);
    vpDisplay::displayLine(I, 0, I.getWidth() * 7 / 8, I.getHeight(), I.getWidth() * 7 / 8, vpColor::red, 1);
    vpDisplay::displayLine(I, I.getHeight() / 8, 0, I.getHeight() / 8, I.getWidth(), vpColor::red, 1);
    vpDisplay::displayLine(I, I.getHeight() * 3 / 8, 0, I.getHeight() * 3 / 8, I.getWidth(), vpColor::red, 1);
    vpDisplay::displayLine(I, I.getHeight() * 5 / 8, 0, I.getHeight() * 5 / 8, I.getWidth(), vpColor::red, 1);
    vpDisplay::displayLine(I, I.getHeight() * 7 / 8, 0, I.getHeight() * 7 / 8, I.getWidth(), vpColor::red, 1);

    double t = vpTime::measureTimeMs();
    std::vector<vpHomogeneousMatrix> cMo_vec;
    dynamic_cast<vpDetectorAprilTag *>(detector)->detect(I, tagSize, cam, cMo_vec);
    t = vpTime::measureTimeMs() - t;
    time_vec.push_back(t);

    std::stringstream ss;
    ss << "Detection time: " << t << " ms for " << detector->getNbObjects() << " tags";
    vpDisplay::displayText(I, 40, 20, ss.str(), vpColor::red);

    bool status = detector->detect(I);

    if (status) {
      for (size_t i = 0; i < detector->getNbObjects(); i++) {
        std::vector<vpImagePoint> p = detector->getPolygon(i);
        //vpRect bbox = detector->getBBox(i);
        //vpDisplay::displayRectangle(I, bbox, vpColor::green);

        for (size_t j = 0; j < p.size(); j++) {
          vpDisplay::displayCross(I, p[j], 14, vpColor::red, 3);
          std::ostringstream number;
          number << j;
          vpDisplay::displayText(I, p[j] + vpImagePoint(15, 5), number.str(), vpColor::blue);
        }

        computePose(point, p, cam, init, cMo); // resulting pose is available in cMo var
        std::cout << "Pose translation (meter): " << cMo.getTranslationVector().t() << std::endl
                  << "Pose rotation (quaternion): " << vpQuaternionVector(cMo.getRotationMatrix()).t() << std::endl;
                  
      }

    }
    //! [Display camera pose for each tag]
    for (size_t i = 0; i < cMo_vec.size(); i++) {
      vpDisplay::displayFrame(I, cMo_vec[i], cam, tagSize / 2, vpColor::none, 3);
    }
    vpDisplay::displayText(I, 20, 20, "Click to quit.", vpColor::red);
    vpDisplay::flush(I);
    if (vpDisplay::getClick(I, false))
      break;
    vpTime::wait(40);
  }

    std::cout << "Benchmark computation time" << std::endl;
    std::cout << "Mean / Median / Std: " << vpMath::getMean(time_vec) << " ms"
              << " ; " << vpMath::getMedian(time_vec) << " ms"
              << " ; " << vpMath::getStdev(time_vec) << " ms" << std::endl;

  delete detector;
  } catch (const vpException &e) {
      std::cerr << "Catch an exception: " << e.getMessage() << std::endl;
    }

  return EXIT_SUCCESS;
#else
  (void)argc;
  (void)argv;
  return 0;
#endif
}
