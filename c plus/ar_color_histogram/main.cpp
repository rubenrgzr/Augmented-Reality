#include <iostream>
#include <cstdio>
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>

using namespace std;
using namespace cv;

//regresa el indice correspondiente
int cat(unsigned char *&cube,int b, int g, int r){
    return (int)cube[256*256*b+256*g+r];
}

int main(int argc, char *argv[]){

    // El programa debe recibir como parametro la ruta de la imagen a analizar
    if(argc!=3){
        cout<<"Error, no se especificaron los parametros correctos"<<endl;
        return 0;
    }

    char *image = argv[1];
    char *cb = argv[2];
    //leer el cubo
    unsigned char *cube = new unsigned char[256*256*256];
    if (cube ){
        FILE *fp = fopen(cb,"rb");
        fread(cube,256*256*256,sizeof(char),fp);
        fclose(fp);
    }

    //para guardar el histograma
    int *hist = new int[11];
    for(int i=0;i<11;i++){
        hist[i]=0;
    }
    //leer la imagen
    Mat img = imread( image , CV_LOAD_IMAGE_ANYCOLOR|CV_LOAD_IMAGE_ANYDEPTH );
    int b,g,r;
    for(int i=0; i< img.rows ;i++){
        for(int j=0; j<img.cols ;j++){
            b = img.at<Vec3b>(i,j)[0];
            g = img.at<Vec3b>(i,j)[1];
            r = img.at<Vec3b>(i,j)[2];
            hist[ cat(cube,b,g,r) ]++;
        }
    }
    //imprimir el histograma
    for(int i=0;i<11;i++){
        cout<<hist[i]<<" ";
    }
    return 0;
}
