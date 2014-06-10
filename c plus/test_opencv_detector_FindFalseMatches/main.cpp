#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <cstdio>
#include <stdlib.h>
using namespace std;
using namespace cv;

const double THRESHOLD = 300;


int main(int argc, char** argv) {
  if (argc < 2) {
    cerr << "Too few arguments" << endl;
    return -1;
  }

  const char* filename = argv[1];

  printf("load file:%s\n", filename);
  // initialize detector and extractor
  FeatureDetector* detector;
  detector = new SiftFeatureDetector(
                                 /*    100, // nFeatures
                                     5, // nOctaveLayers
                                     0.3, // contrastThreshold
                                     5, //edgeThreshold
                                     1.6 //sigma
                                     */
                                     );

  DescriptorExtractor* extractor;
  extractor = new SiftDescriptorExtractor();

  // Compute keypoints and descriptor from the source image in advance
  vector<KeyPoint> keypoints2;
  Mat descriptors2;

  Mat originalGrayImage = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
  if (!originalGrayImage.data) {
    cerr << "gray image load error" << endl;
    return -1;
  }
  Mat originalColorImage = imread(filename, CV_LOAD_IMAGE_ANYCOLOR|CV_LOAD_IMAGE_ANYDEPTH);
  if (!originalColorImage.data) {
    cerr << "color image open error" << endl;
    return -1;
  }

  //equalizeHist( originalGrayImage, originalGrayImage );

  detector->detect(originalGrayImage, keypoints2);
  extractor->compute(originalGrayImage, keypoints2, descriptors2);

  printf("original image:%d keypoints are found.\n", (int)keypoints2.size());

  VideoCapture capture(0);
  capture.set(CV_CAP_PROP_FRAME_WIDTH,800);
  capture.set(CV_CAP_PROP_FRAME_HEIGHT,600);

  //namedWindow("mywindow");

  Mat frame;
  while (1) {
    capture >> frame;



    // load gray scale image from camera
    Size size = frame.size();
    Mat grayFrame(size, CV_8UC1);

    cvtColor(frame, grayFrame, CV_BGR2GRAY);
    if (!grayFrame.data) {
      cerr << "cannot find image file1" << endl;
      exit(-1);
    }
   // equalizeHist( grayFrame, grayFrame );
    //changeIntensity(grayFrame,2, 10);
    // Create a image for displaying mathing keypoints
    Size sz = Size(size.width + originalColorImage.size().width, size.height + originalColorImage.size().height);
    Mat matchingImage = Mat::zeros(sz, CV_8UC3);


    // Draw camera frame
    Mat roi1 = Mat(matchingImage, Rect(0, 0, size.width, size.height));
    frame.copyTo(roi1);
    // Draw original image
    Mat roi2 = Mat(matchingImage, Rect(size.width, size.height, originalColorImage.size().width, originalColorImage.size().height));
    originalColorImage.copyTo(roi2);

    vector<KeyPoint> keypoints1;
    Mat descriptors1;
    vector<DMatch> matches;

    // Detect keypoints
    detector->detect(grayFrame, keypoints1);
    extractor->compute(grayFrame, keypoints1, descriptors1);


    /*for (int i=0; i<keypoints1.size(); i++){
      KeyPoint kp = keypoints1[i];
      circle(matchingImage, kp.pt, cvRound(kp.size*0.25), Scalar(255,255,0), 1, 8, 0);
    }*/

      FlannBasedMatcher matcher;

      matcher.match( descriptors2, descriptors1, matches );

      double max_dist = 0; double min_dist = 100;


      std::vector< DMatch > good_matches,good_matches2;


      vector<DMatch> matches2;
      matcher.match( descriptors1, descriptors2, matches2 );

        /*Separar los buenos puntos usando RANSSAC*/
            vector<Point2f> points1Raw; //Raw points from Keypoints
            vector<Point2f> points1; //Undistorted points
            vector<Point2f> points2Raw;
            vector<Point2f> points2;

            for(int k=0; k<matches.size(); k++) {
                points1Raw.push_back(keypoints2[matches[k].queryIdx].pt);
                points2Raw.push_back(keypoints1[matches[k].trainIdx].pt);
            };

            vector<uchar> states;

            Mat f = findFundamentalMat(points1Raw, points2Raw, FM_RANSAC, 3, 0.99, states);
            //cout<<"XXXXXX"<<endl;
            for(int k=0; k<matches.size(); k++){
                //siftTotalM++;
                if((bool)states[k]){
                    //siftOk++;
                    good_matches2.push_back(matches[k]);
                }
            }

      /*comparar que matches si son iguales*/

      cout<<"M1: "<<matches.size()<<" M2: "<<matches2.size()<<endl;
      int ok=0;
      for( int i = 0; i < good_matches2.size(); i++ ){

         // KeyPoint kp2 = keypoints2[matches[i].trainIdx

          ok=0;
          for(int j=0;j<matches2.size();j++){
            if( good_matches2[i].trainIdx == matches2[j].queryIdx){
                /*ver que los queryIndex sean los mismos*/
                if(good_matches2[i].queryIdx == matches2[j].trainIdx){
                    ok=1;
                    break;
                }
            }
          }
          //if( matches[i].distance < 250 && rad <10 && rad2 <10 )
          // m.distance < 0.8*n.distance
          if(ok==1 && sqrt(good_matches2[i].distance)<10)
            good_matches.push_back(good_matches2[i]);
      }
      cout<<"GOOD Matches "<<good_matches.size()<<endl;
    //-- Draw only "good" matches
    Mat img_matches,img_matches2;

    /*drawMatches( originalGrayImage, keypoints2,trans1, keypoints3,
               good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(),DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS); */


   // imshow("1", img_matches);

    drawMatches( originalGrayImage, keypoints2,grayFrame, keypoints1,
               good_matches, img_matches2, Scalar::all(-1), Scalar::all(-1),
               vector<char>(),DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);


    imshow("2", img_matches2);

    int c = waitKey(2);
    if (c == '\x1b')
      break;
  }

  return 0;
}

