#include <stdlib.h>     /* atoi */
#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include <fstream>
#include <sstream>
#include <ctime>
#include <cstdlib>

using namespace std;
using namespace cv;
/*lee un archivo con cierto formato y extrae los keypoints y su descriptor*/
void read(vector<KeyPoint> &keypoints, Mat &descriptor, char* fileName){
    ifstream infile(fileName);
    string line;

   /* if(infile.good()){
            cout<<"El archivo se abrio correctamente"<<endl;
    }else{
        cout<<"Problema al abrir el archivo"<<endl;
    }*/
    //1 entero que indica el numero de keypoints
    //2 enteros para 1 keypoint, 32 para el descriptor
    getline(infile, line);
    istringstream iss(line);
    //las siguientes n*m lineas tienes los valores correspondientes a los pixeles
    int descriptorSize = 128;
    int keypointsSize = 0;
    float x,y;
    keypoints.resize(keypointsSize);
    iss>>keypointsSize;
   /* cout<<"Key point size: "<<keypointsSize<<endl;
    cout<<fileName<<endl;*/
    //descriptor.create(Size(descriptorSize, keypointsSize),CV_8U); //importante poner un tipo de dato a doc
    descriptor.create(Size(descriptorSize, keypointsSize),CV_32FC1 ); //importante poner un tipo de dato a doc

    //cout<<keypointsSize;
    float desc;
    float _size,_angle,_response;
    int _octave,_class_id;
    for(int i=0; i<keypointsSize; i++){
        iss >> x;
        iss >> y;
        iss>>_angle;
        iss>>_class_id;
        iss>>_octave;
        iss>>_response;
        iss>>_size;
        KeyPoint kp(x,y,_size,_angle,_response,_octave,_class_id);
        //cout<<x<<" "<<y<<endl;
        keypoints.insert(keypoints.end(),kp);
        /*descriptorSize*/
        for(int j=0;j<descriptorSize;j++){
            iss >> desc;
            //cout<<(uchar)desc<<" "<<endl;

            descriptor.at<float>(i,j) = (float)desc;

        }
    }
}
int main(int argc, char** argv){


    if( argc < 3 ){
        cout<<"Error -2"<<endl;
        return -1;
    }

    Mat descriptors1, descriptors2;
    vector<KeyPoint> keypoints1,keypoints2;
    read(keypoints1, descriptors1, argv[1]);
    read(keypoints2, descriptors2, argv[2]);



    double max_dist = 0; double min_dist = atof(argv[3]);

    /*hacer el matching*/

      /*FlannBasedMatcher matcher;
      std::vector< DMatch > matches;
      funciona bien con sift
      */




    /*descriptors1.convertTo(descriptors1, CV_32FC1);
    descriptors2.convertTo(descriptors2, CV_32FC1);
      if ( descriptors1.empty() )
        cout<<"EMPTY"<<endl;
      if ( descriptors2.empty() )
        cout<<"EMPTY"<<endl; */

    FlannBasedMatcher matcher;//(new flann::LshIndexParams(30,15,2));

    //  BFMatcher matcher(cv::NORM_HAMMING);
    std::vector< DMatch > matches;

    matcher.match( descriptors2, descriptors1, matches );


      max_dist = 0; min_dist = 100;


      std::vector< DMatch > good_matches,good_matches2;

      printf("-- Max dist : %f \n", max_dist );
      printf("-- Min dist : %f \n", min_dist );


      vector<DMatch> matches2;
      matcher.match( descriptors1, descriptors2, matches2 );



      /*comparar que matches si son iguales*/

    //  cout<<"M1: "<<matches.size()<<" M2: "<<matches2.size()<<endl;
      int ok=0;
      for( int i = 0; i < matches2.size(); i++ ){

         // KeyPoint kp2 = keypoints2[matches[i].trainIdx

          ok=0;
          for(int j=0;j<matches.size();j++){
            if( matches2[i].trainIdx == matches[j].queryIdx){
                /*ver que los queryIndex sean los mismos*/
                if(matches2[i].queryIdx == matches[j].trainIdx){
                    ok=1;
                    break;
                }
            }
          }
          //if( matches[i].distance < 250 && rad <10 && rad2 <10 )
          // m.distance < 0.8*n.distance
          if(ok==1&&sqrt(matches2[i].distance)<20)
            good_matches.push_back(matches2[i]);
      }


       /*Separar los buenos puntos usando RANSSAC*/
            vector<Point2f> points1Raw; //Raw points from Keypoints
            vector<Point2f> points1; //Undistorted points
            vector<Point2f> points2Raw;
            vector<Point2f> points2;

            for(int k=0; k<good_matches.size(); k++) {
                points1Raw.push_back(keypoints2[good_matches[k].queryIdx].pt);
                points2Raw.push_back(keypoints1[good_matches[k].trainIdx].pt);
            };

            vector<uchar> states;

            Mat f = findFundamentalMat(points1Raw, points2Raw, FM_RANSAC, 3, 0.99, states);
            //cout<<"XXXXXX"<<endl;
            for(int k=0; k<good_matches.size(); k++){
                //siftTotalM++;
                if((bool)states[k]){
                    //siftOk++;
                    good_matches2.push_back(matches[k]);
                }
            }

      cout<<good_matches2.size()<<endl;



    return 0;
}
