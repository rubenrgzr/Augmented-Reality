
#include <stdlib.h>     /* atoi */
#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include<fstream>

using namespace std;
using namespace cv;




/*calcular el kernel para la derivada x*/
void dxKernel(float **&kernel,float sigma){
    /*tamaño del kernel*/
    int wIndex = 4*sigma;
    float sigma2 = sigma*sigma,k=1.0;//(1.0/sigma*sqrt(2.0*M_PI));
    //kernel = matrix(-wIndex,wIndex,-wIndex,wIndex);
    float gx=0;
    //float kc= 1.0/(sigma*sqrt(2*M_PI));
    float kc=1.0;
    for(int k=-wIndex;k<=wIndex;k++){
        gx = kc*exp(-(k*k)/(2.0*sigma2));
        for(int p=-wIndex;p <=wIndex;p++){
            kernel[k][p] = (p/sigma2)*(kc*exp(-(p*p)/(2.0*sigma2)))*gx;
        }
    }
}

/*calcular el kernel para la derivada y*/
void dyKernel(float **&kernel,float sigma){
    /*tamaño del kernel*/
    int wIndex = 4*sigma;
    float sigma2 = sigma*sigma;
    //kernel = matrix(-wIndex,wIndex,-wIndex,wIndex);
    float gx=0;
    //float kc= 1.0/(sigma*sqrt(2*M_PI));
    float kc=1.0;
    for(int k=-wIndex;k<=wIndex;k++){
        gx = exp(-(k*k)/(2.0*sigma2));
        for(int p=-wIndex;p <=wIndex;p++){
                kernel[k][p] = (k/sigma2)*(exp(-(p*p)/(2.0*sigma2)))*gx;
        }
    }

}


/*convolucion 2D*/
double convolucion(Mat &mono,int i, int j, float **kernel,double sigma){
	int wIndex = 4*sigma;
	int x1,x2;
	float sum,sumn;
	int hm = mono.rows;
	int wm = mono.cols;
	/*calcular la convolucion en el punto i,j con el kernel*/
			sum=0;
			/*para las condiciones de frontera adaptable*/
			sumn=0;
			/*se debe recorrer toda la ventana*/
			for(int k=-wIndex;k<=wIndex;k++){
				for(int p=-wIndex;p <=wIndex;p++){
					x1 = i+k; //renglon
					x2 = j+p; //columna
					/*condiciones frontera simetricas*/
					if(x1<0){
                        x1 = - x1;
                    }else if(x1>hm-1){
                        x1 = hm - (x1 - (hm-1));
                    }
                    if(x2<0){
                        x2 = -x2;
                    }else if(x2>wm-1){
                        x2 = wm - (x2 - (wm-1));
                    }
					sum += mono.at<uchar>(x1,x2)*kernel[k][p];
					//sumn += kernel[k][p];
				}
			}

        return sum;
}
/********************************************//****************************************************************/
float **matrix(int nrl,int nrh,int ncl,int nch){
    int i;
    float **m;
    m=(float **) malloc((unsigned) (nrh-nrl+1)*sizeof(float*));
    //if (!m) nrerror((char *)"fallo la asignaci¢n de memoria en el paso 1 par la matriz()");
    m -= nrl;
    for (i=nrl;i<=nrh;i++)
    {  m[i]=(float *) malloc((unsigned) (nch-ncl+1)*sizeof(float));
      //          if (!m[i]) nrerror((char *)"fallo la asignaci¢n de memoria en el paso 2 par la matriz()");
                m[i] -= ncl;
    }
    return m;
}

void free_matrix(float **m,int nrl,int nrh,int ncl,int nch)
{
    int i;
    for(i=nrh;i>=nrl;i--)
        free((char*) (m[i]+ncl));
    free((char*) (m+nrl));
}



float magnitudGradiente(Mat &mono,int i,int j){
   float **kernel;
   float sigma=2;

   int wIndex = 4*sigma;
   kernel = matrix(-wIndex,wIndex,-wIndex,wIndex);

   dxKernel(kernel,sigma);
   float dx1 = convolucion(mono,i,j, kernel,sigma);

   dyKernel(kernel,sigma);
   float dx2 = convolucion(mono,i,j, kernel,sigma);

   free_matrix(kernel,-wIndex,wIndex,-wIndex,wIndex);

   return sqrt(dx1*dx1+dx2*dx2);

}

/**
 * Rotate an image, from https://opencv-code.com/quick-tips/how-to-rotate-image-in-opencv/
 */
void rotate(cv::Mat& src, double angle, double scale,  cv::Mat& dst, cv::Mat &transMatrix){
    int len = std::max(src.cols, src.rows);
    cv::Point2f pt(len/2., len/2.);
    transMatrix = cv::getRotationMatrix2D(pt, angle, scale);
    cv::warpAffine(src, dst, transMatrix, cv::Size(len, len));
}
/**
    Regresa el punto transformado por la matriz M
*/
Point2f getTransform(Mat transMatrix, double x, double y){
        Mat src(3/*rows*/,1 /* cols */,CV_64F);

        src.at<double>(0,0)=x;
        src.at<double>(1,0)=y;
        src.at<double>(2,0)=1.0;
     //   cout<<"Mult matrix vector"<<endl;
     //   cout<<"col "<<transMatrix.cols<<" row "<<transMatrix.rows<<endl;
        Mat dst = transMatrix*src; //USE MATRIX ALGEBRA
        return Point2f(dst.at<double>(0,0),dst.at<double>(1,0));
}

double distance2(KeyPoint s,KeyPoint p){
    return sqrt(((s.pt.x-p.pt.x)*(s.pt.x-p.pt.x))+((s.pt.y-p.pt.y)*(s.pt.y-p.pt.y)));
}

/*calcular la media en un vecindario de nxn*/
double mean(KeyPoint k, Mat img,int wSize){
    double mean=0;
    int cc=0;
    int indexJ,indexI;
    for(int i=-wSize;i<=wSize;i++){
        for(int j=-wSize;j<=wSize;j++){
            indexI = i+k.pt.y;
            indexJ = j+k.pt.x;
            if(!(indexI >=0 && indexI < img.rows &&indexJ>=0 &&indexJ<img.cols)){
                continue;
            }
            cc++;
            mean += img.at<uchar>(indexI,indexJ);
        }
    }

    return mean/cc;
}


/*como se trabaja en escala de grises no es necesario convertir a otro espacio de color*/
//alpha - contraste
//beta - brillo
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

int main( int argc, char** argv ){

  int eq=0;
  /*nombre de la imagen de prueba*/
  char *fileName = "cimat.jpg";
  char *dataOut = "./debug/cimat.txt";
  int numOfKeypoints=500;
  /*redireccionar toda la salida a un archivo .txt*/
  ofstream out(dataOut);
  cout.rdbuf(out.rdbuf());// redireccionar cout a out


  /*Leer la imagen*/
  Mat img_object = imread( fileName , IMREAD_GRAYSCALE );

  /*Error al leer la imagen*/
  if( !img_object.data ){
        cout<<"Error -3"<<endl;
        //std::cout<< " --(!) Error reading images " << std::endl;
        return -1;
  }

  Mat img_object2 = img_object.clone();

  if(eq)
    equalizeHist( img_object, img_object );


   // OrbDescriptorExtractor detector(numOfKeypoints);
    SiftFeatureDetector detector(   numOfKeypoints, // nFeatures
                                     4, // nOctaveLayers
                                     0.03, // contrastThreshold
                                     5, //edgeThreshold
                                     1.6 //sigma
                                     );



    /*obtener los puntos caracteristicos*/
    std::vector<KeyPoint> keypoints_object;
    detector.detect( img_object, keypoints_object );

    /*Cambiar el descriptor para usar otro método*/
    DescriptorExtractor* siftExtractor = new SiftDescriptorExtractor();

    Mat orbDescriptor,siftDescriptor,surfDescriptor;

    siftExtractor->compute(img_object, keypoints_object, siftDescriptor);
    //cout<<"termino siftExtractor"<<endl;

    //cout<<"Se detectaron "<<keypoints_object.size()<<" keypoints."<<endl;


    /*Transformar la imagen y obtener los puntos caracteristicos*/

    FlannBasedMatcher siftMatcher,surfMatcher;
    //BFMatcher siftMatcher(cv::NORM_HAMMING);


    std::vector<KeyPoint> keypoints_objectT; /*para obtener los puntos caracteristicos de la imagent transformada*/

    /*Aplicar una transformacion a la imagen original y hace el match*/

    double angle=0;
    double brigth=-30;
    double scale=0.5;
    Mat img_transform;
    Mat transMatrix;
    double increment=60;
    double bincrement=5;
    double orbGoodM=0,siftGoodM=0,surfGoodM=0;
    double orbTotalM=0,siftTotalM=0,surfTotalM=0;
    int total=0;
    double maxRadius=10;/*radio en el que se considera que un pixel se emparejo correctamente*/
    double globalMinDist=10000000,globalMaxDist=-10000000;
/*ciclo para la escuala*/
while(scale<=2){
    brigth=-30;
    /*ciclo para cambiar brillo*/
    while(brigth<=30){
    /*ciclo para rotar*/
        angle=0;
        while(angle <=360){

            std::vector< DMatch > siftMatches,siftMatches2;

            /*no tomar en cuenta la misma imagen*/
            if(scale==1&&brigth==0&&angle==0){
                angle+=increment;
                continue;
            }

            total++;
            /*rotar*/
           /* cout<<"Rotando "<<angle<<" grados "<<endl;
            cout<<"Escalando "<<scale<<" "<<endl;*/
            rotate(img_object2,angle,scale,img_transform,transMatrix);
            /*cambiar la intencidad*/
            //Mat img_yuv  = Mat::zeros( img_object.size(), img_object.type() );
            changeIntensity(img_transform,1,brigth);
            //cout<<"Cambiando brillo "<<brigth<<" "<<endl;
           /* namedWindow("mywindow");
            imshow("mywindow", img_transform);
            waitKey(1);
            system("PAUSE");
            */

            if(eq)
                equalizeHist( img_transform, img_transform );

            detector.detect( img_transform, keypoints_objectT );

            /*obtener los puntos caracteristicos de esta imagen*/
            Mat orbDescriptorT,siftDescriptorT,surfDescriptorT;

            siftExtractor->compute(img_transform, keypoints_objectT, siftDescriptorT);
           // cout<<"termino siftExtractor T"<<endl;

           /* siftExtractor->compute(img_transform, keypoints_objectT, orbDescriptorT);
            cout<<"termino orbExtractor T"<<endl;

            surfExtractor->compute(img_transform, keypoints_objectT, surfDescriptorT);
            cout<<"termino sufrExtractor T"<<endl;
            */

           // cout<<"Se detectaron "<<keypoints_objectT.size()<<" keypoints T."<<endl;

            /*Hacer el matching con cada uno de los descriptores obtenidos*/

             siftMatcher.match( siftDescriptorT,siftDescriptor, siftMatches );
             siftMatcher.match( siftDescriptor,siftDescriptorT, siftMatches2 );
             //cout<<"SIFT - Se emparejaron "<<siftMatches.size()<<endl;
            /*
             surfMatcher.match(  surfDescriptor,surfDescriptorT, surfMatches );
             cout<<"SURF - Se emparejaron "<<surfMatches.size()<<endl;

             siftMatcher.match(  orbDescriptor,orbDescriptorT, orbMatches );
             cout<<"ORB - Se emparejaron "<<orbMatches.size()<<endl;
             */

             std::vector< DMatch > siftGoodMatches,siftGoodMatches2;
             /*std::vector< DMatch > orbGoodMatches;
             std::vector< DMatch > surfGoodMatches;*/

            /*contar el numero correcto de parejas para cada descriptor*/
            int orbOk=0,siftOk=0,surfOk=0;


            /*double x,y;
            Point2f pt;
            pt = getTransform(transMatrix,0,0);
            cout<<"El punto equivalente a 0,0 es "<<pt.x<<" , "<<pt.y<<endl;*/
            int p,k;
            double dist;
            Point2f pt;
            KeyPoint kp;


            /*Separar los buenos puntos*/
            vector<Point2f> points1Raw; //Raw points from Keypoints
            vector<Point2f> points1; //Undistorted points
            vector<Point2f> points2Raw;
            vector<Point2f> points2;

            for(int k=0; k<siftMatches.size(); k++) {
                points1Raw.push_back(keypoints_objectT[siftMatches[k].queryIdx].pt);
                points2Raw.push_back(keypoints_object[siftMatches[k].trainIdx].pt);
            };

            vector<uchar> states;

            Mat f = findFundamentalMat(points1Raw, points2Raw, FM_RANSAC, 3, 0.99, states);
            //cout<<"XXXXXX"<<endl;
            for(int k=0; k<siftMatches.size(); k++){
                //siftTotalM++;
                if((bool)states[k]){
                    //siftOk++;
                    siftGoodMatches2.push_back(siftMatches[k]);
                }else{
                    //diss++;
                }
            }

            //siftGoodM +=siftOk;


            double max_dist=siftGoodMatches2[0].distance,min_dist=siftGoodMatches2[0].distance;
            int ok=0;
            int diss = 0;
            for(int i=0;i<siftGoodMatches2.size();i++){
                /*Poner una restrición para discriminar los "malos matcheos"*/
                if(sqrt(siftGoodMatches2[i].distance)>20){
                    diss++;
                    continue;
                }
                if(max_dist<siftGoodMatches2[i].distance)
                    max_dist=siftGoodMatches2[i].distance;

                if(min_dist>siftGoodMatches2[i].distance)
                    min_dist=siftGoodMatches2[i].distance;



                k = siftGoodMatches2[i].queryIdx;
                p = siftGoodMatches2[i].trainIdx;
                /*calcular la distancia real entre los puntos*/
                kp = keypoints_object[p];
                pt = getTransform(transMatrix,kp.pt.x,kp.pt.y);
                kp.pt=pt;
                dist = distance2(kp,keypoints_objectT[k]);



                /*validar que sea el vecino mas cercano en ambas direcciones*/
                    ok=0;
                      for(int j=0;j<siftMatches2.size();j++){
                        if( siftGoodMatches2[i].trainIdx == siftMatches2[j].queryIdx){
                            /*ver que los queryIndex sean los mismos*/
                            if(siftGoodMatches2[i].queryIdx == siftMatches2[j].trainIdx){
                                ok=1;
                                break;
                            }
                        }
                      }
                if(ok==0)
                    continue;

                siftTotalM++;
                if( dist <= maxRadius ){
                    siftOk++;
                    siftGoodMatches.push_back( siftGoodMatches2[i]);
                    if(globalMaxDist<siftGoodMatches2[i].distance)
                        globalMaxDist=siftGoodMatches2[i].distance;
                    if(globalMinDist>siftGoodMatches2[i].distance)
                        globalMinDist=siftGoodMatches2[i].distance;
                }
            }
            siftGoodM +=siftOk;






            Mat img_matches;
            drawMatches( img_transform, keypoints_objectT,img_object, keypoints_object,
                       siftGoodMatches, img_matches, Scalar::all(-1), Scalar::all(-1),
                       vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

             std::ostringstream ss;
             ss << ((float)siftGoodMatches2.size()/(float)siftMatches.size());
             std::string s(ss.str());
             pt.x=20;
             pt.y=20;
             putText(img_matches, s , pt,
                        FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(255,255,255), 1, CV_AA);

            imshow("mywindow", img_matches);
            cvNamedWindow( "mywindow", CV_WINDOW_NORMAL );
            int c = waitKey(2);
            //system("PAUSE");


            /*Imprimir los resultados*/
            cout<<keypoints_object.size()<<(char)(9);
            cout<<keypoints_objectT.size()<<(char)(9);
            cout<<scale<<(char)(9);
            cout<<angle<<(char)(9);
            cout<<brigth<<(char)(9);
            cout<<maxRadius<<(char)(9);
            cout<<siftGoodMatches2.size()<<(char)(9);
            cout<<siftMatches.size()<<(char)(9);
            cout<<((float)siftGoodMatches2.size()/(float)siftMatches.size())<<(char)(9);
            cout<<min_dist<<(char)(9);
            cout<<max_dist<<(char)(9);
            cout<<endl;

            angle+=increment;
        }//rotar

        brigth+=bincrement;

      /*  cout<<"Total de transformaciones: "<<total<<endl;
        cout<<"Porcentaje de matching para SIFT: "<<(siftGoodM/siftTotalM)*100<<endl;
        cout<<"Porcentaje de matching para SURF: "<<(surfGoodM/surfTotalM)*100<<endl;
        cout<<"Porcentaje de matching para ORB: "<<(orbGoodM/orbTotalM)*100<<endl;*/

    }//brigth
    scale+=0.5;
}//scale
    cout<<"Total de transformaciones: "<<total<<endl;
    cout<<"Porcentaje de matching para SIFT: "<<(siftGoodM/siftTotalM)*100<<endl;
    /*cout<<"Porcentaje de matching para SURF: "<<(surfGoodM/surfTotalM)*100<<endl;
    cout<<"Porcentaje de matching para ORB: "<<(orbGoodM/orbTotalM)*100<<endl;*/
    cout<<"SIFT descritor distance rage: ["<<globalMinDist<<", "<<globalMaxDist<<"]"<<endl;
    return 0;
}
