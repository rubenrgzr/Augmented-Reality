/**
 * @file SURF_Homography
 * @brief SURF detector + descriptor + FLANN Matcher + FindHomography
 * @author A. Huaman
 */

#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/features2d.hpp"

using namespace std;
using namespace cv;


/*
ORB::ORB(
         int nfeatures=500,
         float scaleFactor=1.2f,
         int nlevels=8,
         int edgeThreshold=31,
         int firstLevel=0,
         int WTA_K=2,
         int scoreType=ORB::HARRIS_SCORE,
         int patchSize=31
         )
*/
int main( int argc, char** argv )
{
  if( argc != 3 )
  {  return -1; }

  Mat img_object = imread( argv[1], IMREAD_GRAYSCALE );
  Mat img_scene = imread( argv[2], IMREAD_GRAYSCALE );

  if( !img_object.data || !img_scene.data )
  { std::cout<< " --(!) Error reading images " << std::endl; return -1; }

    //OrbDescriptorExtractor detector;
    OrbFeatureDetector detector(1000);

    std::vector<KeyPoint> keypoints_object, keypoints_scene;

    detector.detect( img_object, keypoints_object );

    printf("original image:%d keypoints are found.\n", (int)keypoints_object.size());

    for (int i=0; i<keypoints_object.size(); i++){
      KeyPoint kp = keypoints_object[i];
      circle(img_object, kp.pt, cvRound(1), Scalar(255,255,0), 1, 8, 0);//
    }

    namedWindow("mywindow");
while (1) {
     // Display mathing image
    imshow("mywindow", img_object);
    int c = waitKey(2);
    if (c == '\x1b')
      break;

}//while

  return 0;
}
