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

/**
 * Calculate euclid distance
 */
double euclidDistance(Mat& vec1, Mat& vec2) {
  double sum = 0.0;
  int dim = vec1.cols;
  for (int i = 0; i < dim; i++) {
    sum += (vec1.at<uchar>(0,i) - vec2.at<uchar>(0,i)) * (vec1.at<uchar>(0,i) - vec2.at<uchar>(0,i));
  }
  return sqrt(sum);
}

/**
 * Find the index of nearest neighbor point from keypoints.
 */
int nearestNeighbor(Mat& vec, vector<KeyPoint>& keypoints, Mat& descriptors) {
  int neighbor = -1;
  double minDist = 1e6;

  for (int i = 0; i < descriptors.rows; i++) {
    KeyPoint pt = keypoints[i];
    Mat v = descriptors.row(i);
    double d = euclidDistance(vec, v);
    //printf("%d %f\n", v.cols, d);
    if (d < minDist) {
      minDist = d;
      neighbor = i;
    }
  }

  if (minDist < THRESHOLD) {
    return neighbor;
  }

  return -1;
}

/**
 * Find pairs of points with the smallest distace between them
 */
void findPairs(vector<KeyPoint>& keypoints1, Mat& descriptors1,
               vector<KeyPoint>& keypoints2, Mat& descriptors2,
               vector<Point2f>& srcPoints, vector<Point2f>& dstPoints) {
  for (int i = 0; i < descriptors1.rows; i++) {
    KeyPoint pt1 = keypoints1[i];
    Mat desc1 = descriptors1.row(i);
    int nn = nearestNeighbor(desc1, keypoints2, descriptors2);
    if (nn >= 0) {
      KeyPoint pt2 = keypoints2[nn];
      srcPoints.push_back(pt1.pt);
      dstPoints.push_back(pt2.pt);
    }
  }
}

void changeIntensity(Mat &dst,double alpha, double beta){
    //cvtColor(src, dst, CV_BGR2YCrCb);
    /// Do the operation new_image(i,j) = alpha*image(i,j) + beta
    for( int y = 0; y < dst.rows; y++ ){
        for( int x = 0; x < dst.cols; x++ ){
               // for( int c = 0; c < 3; c++ ){
                    dst.at<uchar>(y,x) =
                        saturate_cast<uchar>( alpha*( dst.at<uchar>(y,x) ) + beta );
                //}
        }
    }

}


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
                                   //  0, // nFeatures
                                   //  5, // nOctaveLayers
                                   //  0.3, // contrastThreshold
                                   //  5, //edgeThreshold
                                   //  1.6 //sigma
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

   // printf("image1:%zd keypoints are found.\n", keypoints1.size());

    for (int i=0; i<keypoints1.size(); i++){
      KeyPoint kp = keypoints1[i];
      circle(matchingImage, kp.pt, cvRound(kp.size*0.25), Scalar(255,255,0), 1, 8, 0);
    }
    // Find nearest neighbor pairs
   /* vector<Point2f> srcPoints;
    vector<Point2f> dstPoints;

    findPairs(keypoints1, descriptors1, keypoints2, descriptors2, srcPoints, dstPoints);
    printf("%zd keypoints are matched.\n", srcPoints.size());

    char text[256];
    sprintf(text, "%f/%fd keypoints matched.", srcPoints.size(), keypoints2.size());
    putText(matchingImage, text, Point(0, cvRound(size.height + 30)), FONT_HERSHEY_SCRIPT_SIMPLEX, 1, Scalar(0,0,255));
    */
    /*
    // Draw line between nearest neighbor pairs
    for (int i = 0; i < (int)srcPoints.size(); ++i) {
      Point2f pt1 = srcPoints[i];
      Point2f pt2 = dstPoints[i];
      Point2f from = pt1;
      Point2f to   = Point(size.width + pt2.x, size.height + pt2.y);
      line(matchingImage, from, to, Scalar(0, 255, 255));
    }
    */
    //-- Step 3: Matching descriptor vectors using FLANN matcher

      FlannBasedMatcher matcher;
    //  BFMatcher matcher(cv::NORM_HAMMING);
//      std::vector< DMatch > matches;

    //if(descriptors1.rows<descriptors2.rows){

      matcher.match( descriptors2, descriptors1, matches );
    //}
  /*  else{
      caseMatch=2;
      matcher.match( descriptors2, descriptors1, matches );
    }*/
      double max_dist = 0; double min_dist = 100;

      //-- Quick calculation of max and min distances between keypoints

      for( int i = 0; i < descriptors2.rows; i++ )
      { double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
      }

     /* printf("-- Max dist : %f \n", max_dist );
      printf("-- Min dist : %f \n", min_dist );*/

    //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
  //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
  //-- small)
  //-- PS.- radiusMatch can also be used here.
  std::vector< DMatch > good_matches;

/*
  for( int i = 0; i < matches.size(); i++ ){
    /
       Aplicar 1 o 2 ligeras transformaciones a la imagen, matchear y buscar que pares de puntos son equivalentes
       o parecidos
    /
    if( sqrt(matches[i].distance) <= 17 ){ //40 funciono ok
        good_matches.push_back( matches[i]);
    }
  }*/
//--draw all de points
for (int i=0; i<keypoints2.size()  ; i++){
    KeyPoint kp = keypoints2[i];
    //pintar un circulo en los kepoints detectados
    circle(originalGrayImage, kp.pt, cvRound(1), Scalar(255,255,0), 1, 8, 0);//
}
for (int i=0; i<keypoints1.size()  ; i++){
    KeyPoint kp = keypoints1[i];
    //pintar un circulo en los kepoints detectados
    circle(grayFrame, kp.pt, cvRound(1), Scalar(255,255,0), 1, 8, 0);//
}

    /*Aplicar una ligera transformacion a la imagen grayFrame, y luego obtener sus descriptores
      hacer el match con originalGrayImage y verificar cuales son los mismos en el primer match y en el segundo*/


      vector<KeyPoint> keypoints3,keypoints4;
      Mat descriptors3,descriptors4;
      /*clonar la imagen actual*/
      Mat trans1 = grayFrame.clone();
      Mat trans2 = grayFrame.clone();
      /*transformar*/
      //equalizeHist( trans1, trans1 );
      changeIntensity(trans1,1,-50);
      changeIntensity(trans2,1,50);

      detector->detect(trans1, keypoints3);
      detector->detect(trans2, keypoints4);
      extractor->compute(trans1, keypoints3, descriptors3);
      extractor->compute(trans2, keypoints4, descriptors4);
      vector<DMatch> matches2,matches3;
      matcher.match( descriptors2, descriptors3, matches2 );
      matcher.match( descriptors2, descriptors4, matches3 );
      /*comparar que matches si son iguales*/
      cout<<"M1: "<<matches.size()<<" M2: "<<matches2.size()<<endl;
      for( int i = 0; i < matches.size(); i++ ){
          /*KeyPoint kp1 = keypoints2[i];
          KeyPoint kp2 = keypoints2[i];*/
          //cout<<"I1: "<<matches[i].queryIdx<<" I2: "<<matches2[i].queryIdx<<endl;
          /*calcular la distancia entre los 2 puntos*/
          KeyPoint kp1 = keypoints1[matches[i].trainIdx];
          KeyPoint kp2 = keypoints3[matches2[i].trainIdx];
          KeyPoint kp3 = keypoints4[matches3[i].trainIdx];
          double rad = sqrt(pow(kp1.pt.x - kp2.pt.x,2)+pow(kp1.pt.y - kp2.pt.y,2));
          double rad2 = sqrt(pow(kp1.pt.x - kp3.pt.x,2)+pow(kp1.pt.y - kp3.pt.y,2));
          if( matches[i].distance < 250 && rad <10 && rad2 <10 )
            good_matches.push_back(matches[i]);
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

