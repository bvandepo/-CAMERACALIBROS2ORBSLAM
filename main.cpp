//CAMERACALIBROS2ORBSLAM Monocular camera calibration file converter \n Bertrand VANDEPORTAELE 2016
//example:    ./CAMERACALIBROS2ORBSLAM -i /home/bvandepo/orbslam/CAMERACALIBROS2ORBSLAM/ost.txt -m /home/bvandepo/orbslam/ORB_SLAM2/Examples/Monocular/ueye1280-BVANDEPO.yaml -o out.txt
//        model and output file can be the same file, so this is also valid, the output file serves also as model in that case
//           ./CAMERACALIBROS2ORBSLAM -i /home/bvandepo/orbslam/CAMERACALIBROS2ORBSLAM/ost.txt -o out.txt


#include <iostream>
#include <cstdlib>
#include <fstream>
#include <ios>
#include <sstream>
#include <vector>

using namespace std;
////////////////////////////////////////////////////////////////////////////////////////////////////
void usage(){
    cout <<"To use a model file that won't be modified, and generate an output file:" << endl;
    cout <<"CAMERACALIBROS2ORBSLAM -i inputFileName -m modelFileName -o outputFileName" << endl;
    cout <<"To update the output file (it is also used as model):" << endl;
    cout <<"CAMERACALIBROS2ORBSLAM -i inputFileName -o modelandoutputFileName" << endl;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[])
{

    int state=0;
    string fx,fy,cx,cy,k1,k2,p1,p2,dumb; //use string instead of double to avoid precision losses

    //for debbuging
    //    string inputFileName="/home/bvandepo/ROS2ORBSLAM/ost.txt";
    //    string modelFileName="/home/bvandepo/orbslam/ORB_SLAM2/Examples/Monocular/ueye1280-BVANDEPO.yaml";
    //    string outputFileName="out.txt";

    string inputFileName="";
    string modelFileName="";
    string outputFileName="";

    cout <<"CAMERACALIBROS2ORBSLAM Monocular camera calibration file converter \n Bertrand VANDEPORTAELE 2016"<<endl;

    //cout << __DATE__ << endl << __TIME__ << endl;
    //cout<<"app launched via: ";
    //for (int j=0;j<argc;j++)
    //    cout<<argv[j]<<" ";
    //cout<<"\n";

    ////////////////////////////PARSING////////////////////////////////////////////
    // Read the "argv" function input arguments
    for(int i = 1; i < argc; i++ )
    {
        if( string("--help").compare( argv[i] )==0){
            usage();
            return EXIT_FAILURE;
        }
        if(  string("-i").compare( argv[i] )==0 ){
            char* name=argv[++i];
            if (name[0]==0){
                cout << "Invalid input file name" << endl;
                return EXIT_FAILURE;
            } else
                inputFileName=name;
        }else if(  string("-m").compare( argv[i] )==0 ){
            char* name=argv[++i];
            if (name[0]==0){
                cout << "Invalid model input file name" << endl;
                return EXIT_FAILURE;
            } else
                modelFileName=name;
        }else if(  string("-o").compare( argv[i] )==0 ){
            char* name=argv[++i];
            if (name[0]==0){
                cout << "Invalid output file name" << endl;
                return EXIT_FAILURE;
            } else
                outputFileName=name;
        }else{
            cout << "Unknown option" << endl;
            return EXIT_FAILURE;
        }
    }
    //in case the model file name is not specified
    if ( (modelFileName.size()==0)){
        cout << "Using the output file as model" << endl;
        modelFileName=outputFileName;
    }

    if ( (inputFileName.size()==0) ||  (outputFileName.size()==0) ||  (modelFileName.size()==0) ){
        usage();
        return EXIT_FAILURE;
    }

    cout << "inputFileName: "<< inputFileName <<endl;
    cout << "modelFileName: "<< modelFileName <<endl;
    cout << "outputFileName: "<< outputFileName <<endl;

    std::fstream fi(inputFileName.c_str(), std::ios::in );
    if(!fi)
    {
        std::cerr << "could not open file " <<  inputFileName <<endl;
        return EXIT_FAILURE;
    }
    std::fstream fm(modelFileName.c_str(), std::ios::in );
    if(!fm)
    {
        std::cerr << "could not open file " <<  modelFileName <<endl;
        return EXIT_FAILURE;
    }


#define MAX_CAR_PER_LINE 256
    char line[MAX_CAR_PER_LINE ];
    while (fi.getline(line,MAX_CAR_PER_LINE ) && (state!=5)) {
        std::string str = string(line);
        std::stringstream ss(str);
        //cout <<line << endl;
        switch (state){
        case 0:
            if (string("camera matrix").compare(line )==0){
                state=1;
            }
            break;
        case 1:
            ss  >> fx >>dumb >>cx;
            state=2;
            break;
        case 2:
            ss  >>dumb >> fy >>cy;
            state=3;
            break;
        case 3:
            if (string("distortion").compare(line )==0){
                state=4;
            }
            break;
        case 4:
            ss  >> k1 >> k2>> p1>>p2 >>dumb;
            state=5;
            break;

        default:
            break;
        }
    }
    fi.close();

    std::vector<std::string> strVec; //a vector of string to store the model file lines
    while (fm.getline(line,MAX_CAR_PER_LINE )) {
        strVec.push_back(string(line));
        //cout <<"ligne lue: "<< string(line) << endl;
    }
    fm.close();
    //The output file has to be opened after the model file was closed, because it can be the same file that ware opened in read-only and then in write-only
    std::fstream fo(outputFileName.c_str(), std::ios::out );
    if(!fo)
    {
        std::cerr << "could not open file " <<  outputFileName <<endl;
        return EXIT_FAILURE;
    }
    for (unsigned int i=0;i<strVec.size();i++){
        std::string str=strVec[i];
        //cout <<"ligne: "<< str << endl;
        std::stringstream ss(str);
        string var;
        ss  >> var;
        if (var.compare("Camera.fx:")==0){
            fo << var << " " << fx << endl;
        }else if (var.compare("Camera.fy:")==0){
            fo << var << " " << fy << endl;
        }else if (var.compare("Camera.cx:")==0){
            fo << var << " " << cx << endl;
        }else if (var.compare("Camera.cx:")==0){
            fo << var << " " << cx << endl;
        }else if (var.compare("Camera.cy:")==0){
            fo << var << " " << cy << endl;
        }else if (var.compare("Camera.k1:")==0){
            fo << var << " " << k1 << endl;
        }else if (var.compare("Camera.k2:")==0){
            fo << var << " " << k2 << endl;
        }else if (var.compare("Camera.p1:")==0){
            fo << var << " " << p1 << endl;
        }else if (var.compare("Camera.p2:")==0){
            fo << var << " " << p2 << endl;
        }else
            fo << str <<endl;
    }
    fo.close();
    cout <<"process ended correctly"<<endl;
    return 0;
}

