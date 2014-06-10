
#include <stdlib.h>     /* atoi */
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
/*
    Este codigo se mandara llamar desde PHP y debe imprimir los keypoints de una imagen, un punto por
    renglo, luego PHP se encarga de tomar esa salida y guardar los datos en una base en forma
    de cadena de texto.

    Parametros del programa:
    argc = 3
    argv[1] debe contener el nombre del archivo de imagen del cual se extraeran los keypoints
    argv[2] debe contener un entero que representa el numero de keypoints que se extraeran de la imagen
*/
int main( int argc, char** argv )
{
  //No se especifico el nombre de la imagen ni el numero de keypoints correctamente
  if( argc != 3 ){
    cout<<"Error -2"<<endl;
    return -1;
  }

  int numOfKeypoints=200;
  /*Leer la imagen*/
  Mat img_object = imread( argv[1], IMREAD_GRAYSCALE );
  numOfKeypoints = atoi( argv[2] );

  /* Leer la imagen a color y calcular el promedio de cada canal*/
  Mat colorImg = imread(argv[1], CV_LOAD_IMAGE_COLOR);
  Scalar mn = mean(colorImg);

  printf("%f\n%f\n%f\n",mn[0],mn[1],mn[2]); //BGR
  /*Error al leer la imagen*/
  if( !img_object.data ){
        cout<<"Error -3"<<endl;
        //std::cout<< " --(!) Error reading images " << std::endl;
        return -1;
  }

   // OrbDescriptorExtractor detector(numOfKeypoints);
    /*SiftFeatureDetector detector(   numOfKeypoints, // nFeatures
                                     4, // nOctaveLayers
                                     0.03, // contrastThreshold
                                     5, //edgeThreshold
                                     1.6 //sigma
                                     ); */

    FeatureDetector* detector;
    detector = new SiftFeatureDetector();
    DescriptorExtractor* extractor;
    extractor = new SiftDescriptorExtractor();


    std::vector<KeyPoint> keypoints_object;

    detector->detect( img_object, keypoints_object );

    Mat descriptors1;
    extractor->compute(img_object, keypoints_object, descriptors1);


    //printf("original image:%d keypoints are found.\n", (int)keypoints_object.size());
    int i;
    cout<<keypoints_object.size()<<" "<<endl;
    for (i=0; i<keypoints_object.size()  ; i++){
      KeyPoint kp = keypoints_object[i];
      //pintar un circulo en los kepoints detectados
      //circle(img_object, kp.pt, cvRound(1), Scalar(255,255,0), 1, 8, 0);//
      //Imprimir los keypoints generados por ORB
      //cout<<kp.size<<" ";*/
      printf("%f %f %f %d %d %f %f ",kp.pt.x,kp.pt.y,kp.angle,kp.class_id,kp.octave,kp.response,kp.size);
      //cout<<descriptors1.cols<<endl;

      for(int j=0; j<descriptors1.cols;j++){

            printf("%f ",(float)descriptors1.at<float>(i,j));   /*DESCOMENTAR*/
      }


    }

   /* FileStorage fs("Keypoints.yml", FileStorage::WRITE);
    write(fs, "keypoints_1", keypoints_object);
    write(fs, "descriptors_1", descriptors1);*/
    //cout<<i<<endl;

  return 0;
}
