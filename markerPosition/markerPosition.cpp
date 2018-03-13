/**
 *markerPosition parses text files containing poses outputted from Elastic Fusion and AR Tag poses from inputImage. 
 *It the computes the pose of the markers relative to Elastic Fusions coordinate system and outputs a ply file to visualize the 4 vertexs of 
 *of each tag. 
 *Finally it outputs a .dat multimarker configuration file for use with ARToolkit multimarker augemented reality. 
 *
 *Author Oisin Feely 2018
 */


/**
 *Includes
*/
#include <iostream>
#include <string.h>
#include <fstream>
#include <stdio.h>
#include <sstream>
#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <cmath>
#include <iostream>
#include <fstream>
using namespace std; 

/**
 *Typedefs for convenience
 *XYZ to hold xyz positions of each marker for averaging
 */
typedef Eigen::Matrix<double, 4, 4> Pose4X4;
typedef Eigen::Matrix<double, 4, 1> XYZ;

/**
 *Constants for file paths, add option to input files in refactoring. 
 */
const char *pfAR = "../TestLogs/ARLogReaderFrames&Poses/75cm.txt";
const char *pfEF = "../TestLogs/EFFrames&Poses/75Test.txt";

struct arInfo{
  int frame;
  int pattID;
  Pose4X4 pose;
};

struct efInfo{
  int frame;
  Pose4X4 pose;
};

//Vector for ar poses, contains, frame, patt id, and ar poses
vector<arInfo> arPoses;

//Vector for ef poses, contains, frame and ef poses; 
vector<efInfo> efPoses;

//vector to hold tag points relative to ef
vector<Eigen::Matrix4d> tagPoints; 

//Quaterions to hold global coords
vector<Eigen::Quaternion<double> > globalCoordsQuat;

//final global coordinates, i.e coordinates of tags in ef coordinate system
vector<Pose4X4> globalMarkerCoords;

//number of tags
int numPatts; 

//Parse columns from text, flag is what column i.e 1,2,3
void parseCols(string row, Pose4X4& pose, int flag)
{
  istringstream r(row);
  string buff;
  vector<string>col;
  while(r>>buff){
    col.push_back(buff);
  }
  if(flag==1){
    pose(0,0) = atof(col[0].c_str());
    pose(0,1) = atof(col[1].c_str());
    pose(0,2) = atof(col[2].c_str());
    pose(0,3) = atof(col[3].c_str());
  }
  else if(flag==2){
    pose(1,0) = atof(col[0].c_str());
    pose(1,1) = atof(col[1].c_str());
    pose(1,2) = atof(col[2].c_str());
    pose(1,3) = atof(col[3].c_str());
  }
  else if(flag==3){
    pose(2,0) = atof(col[0].c_str());
    pose(2,1) = atof(col[1].c_str());
    pose(2,2) = atof(col[2].c_str());
    pose(2,3) = atof(col[3].c_str());
  }
  
}

//Parse text from text files, input stream
int parseText(stringstream& stream, int flag)
{
  //variables to hold parsed data temporarily
  int currentFrame, pattID;
  string r1,r2,r3,r4, line;
  int index = 0;
  arInfo dataAR;
  efInfo dataEF; 
  Pose4X4 temp;
  temp(3,0) = 0.0;
  temp(3,1) = 0.0;
  temp(3,2) = 0.0;
  temp(3,3) = 1.0;

  while(!stream.eof())
    {
      getline(stream, line); 

      if(line.find("NumPatterns")!=string::npos){
	istringstream(line.substr(12))>>numPatts;	
      }
      
      //Parsed one frame and 
      if(line.empty()){
	//Pass parsed values into either ar or ef struct depending on flag, add to vector
	if(flag>0){
	  dataAR.frame = currentFrame;
	  dataAR.pattID = pattID;
	  dataAR.pose = temp;
	  arPoses.push_back(dataAR);
	}
	else {
	  dataEF.frame = currentFrame;
	  dataEF.pose = temp;
	  efPoses.push_back(dataEF);
	}
      }
      else if(line.find("Frame")!=string::npos){
	istringstream (line.substr(5))>>currentFrame;
      }
      else if (line.find("Patt")!=string::npos ){
	istringstream(line.substr(4))>>pattID;
      }
       else if(line.find("r0")!=string::npos){
	r1 = line.substr(2);
	parseCols(r1, temp, 1);
      }
      else if(line.find("r1")!=string::npos){
	r2 = line.substr(2);
	parseCols(r2, temp, 2);
      }
      else if(line.find("r2")!=string::npos){
	r3 = line.substr(2);
	parseCols(r3, temp, 3);
      }
    }
  return true; 
}

Eigen::Quaternion<double> slerpQuaternion(){

  Eigen::Quaternion<double> temp; 
  Eigen::Quaternion<double> total = globalCoordsQuat[0];
  
  for (int i=1; i< globalCoordsQuat.size(); i++){
   
    temp = globalCoordsQuat[i];
    //unsure what 1 should be, defines the scalar (time?) difference between quaternions
    total = total.slerp(1,temp);
    
  }
  return total; 

}

void getMatrix(int currentPatt){

   cout<<currentPatt<<endl;
   int avg = 0;
   XYZ avgTemp;
   avgTemp<<0,0,0,0;
   Pose4X4 temp;
   temp<<
     0, 0, 0, 0,
     0, 0, 0, 0,
     0, 0, 0, 0,
     0, 0 ,0, 0;
    Pose4X4 finalGlobalCoord;
   finalGlobalCoord<<
     0, 0, 0, 0,
     0, 0, 0, 0,
     0, 0, 0, 0,
     0, 0 ,0, 0;

   Eigen::Matrix4d TagPoints;
   TagPoints <<
     0,10,10,0,
     0,0,10,10,
     0,0,0,0,
     1,1,1,1;

   Eigen::Matrix4d EFPoints;
   EFPoints<<
     0, 0, 0, 0,
     0, 0, 0, 0,
     0, 0, 0, 0,
     0, 0 ,0, 0;

     
   //possibly 1,0,0,0, unsure what to instansiate quaternions as ***new note, identity = 1,0,0,0
   Eigen::Quaternion<double> total(1,0,0,0);
   Eigen::Quaternion<double> local(1,0,0,0);
   
  for (int i = 0; i<arPoses.size();i++){
    if (arPoses[i].pattID == currentPatt){
      
      for(int j = 0; j<efPoses.size(); j++){
	if(arPoses[i].frame == efPoses[j].frame){

	  //Finding pose of arPose relative to efPose i.e. the pose of the marker in the global (ef) coordinate system, multiply arPose by efPose
	  temp = arPoses[i].pose*efPoses[j].pose;

	  //for visualization, get 4 points of edge of markers
	  //Eigen::Vector4d P00(0,0,0,1),P10(1,0,0,1);

	  //Eigen::Vector3d EF00 = temp * P00;
	  //Eigen::Vector3d EF10 = temp * P10;

	  //4 corners of the tag, get avg
	  //EFPoints = temp * TagPoints;
	  EFPoints = EFPoints+(temp*TagPoints);
	  
	  //split temp into quaternion+xyz
	  
	  //local rotation i.e rotation of the marker in that frame
	  //push local into vector, slerp vector together
	  local = temp.block<3,3>(0,0);
	  globalCoordsQuat.push_back(local);
	  
	  //Adding up all x,y,z to get average x,y,z of marker
	  avgTemp = avgTemp+temp.col(3); 
	  //Increment for each frame pose is in
	  avg++;
	}
      }
    }
  }

  //iterate through quaternion vector, slerp, then recompose into 
  total = slerpQuaternion();
  avgTemp = avgTemp/avg;
  cout<<EFPoints<<endl;
  EFPoints = EFPoints/avg;
  cout<<EFPoints<<endl;
  finalGlobalCoord.block<3,3>(0,0) = total.toRotationMatrix();
  finalGlobalCoord.col(3) = avgTemp;

  //cout<<finalGlobalCoord<<endl; 
  //cout<<EFPoints<<endl;
  
  tagPoints.push_back(EFPoints);
  //cout<<"average"<<avgTemp<<endl;
  //Average x,y,z coordinates
  globalMarkerCoords.push_back(finalGlobalCoord);
}

void getMultiMarkerConfig(Pose4X4 originP){
  
}

void outputPLY(){

  //output stream 
  ofstream fp;
  //DIDN'T MULTIPLY BY INVERSE
  //**********************************************************
  
  fp.open("/home/oisin/libs/markerPosition/visualize2.ply");
  fp<<"ply\nformat ascii 1.0\ncomment visualization of marker positions by Oisin Feely\nelement vertex "<<tagPoints.size()*4<<"\nproperty float32 x\nproperty float32 y\nproperty float32 z\nelement face "<<tagPoints.size()<<"\nproperty list uint8 int32 vertex_index\nend_header\n";

  for(int i=0;i<tagPoints.size(); i++){

    Eigen::Matrix<double, 3,4> tagPts = (tagPoints[i].block<3,4>(0,0))/1000;
    cout<<tagPts<<endl;

    for(int j=0; j<4;j++){
      //need to divide by 100 as currently in mms
      Eigen::RowVector3d flatten = tagPts.col(j);
      
      cout<<flatten<<endl;
      fp<<flatten<<endl;
    }
  }
  int count=0;
  for(int g=0; g<tagPoints.size(); g++){
    fp<<4<<" ";
    
    for(int k=0; k<4; k++){
      fp<<count<<" ";
      count++;
    }
    fp<<endl;
  }
  fp.close();
 
 
}

int main ()
{
  //Declare ifstreams to read in txt data, to be passed to parseText
  ifstream posesAR;
  ifstream posesEF;

  //Flags to identify text
  int arFlag = 1;
  int efFlag = -1;
  
  arPoses.begin();
  efPoses.begin();
  
  posesAR.open(pfAR, ios::in);
  posesEF.open(pfEF, ios::in);

   if(!posesAR.is_open()) {
      cout<<"cannot open file"<<pfAR;
      return false; 
    }
  if(!posesEF.is_open()){
    cout<<"cannot open file"<<pfEF;
    return false;
  }

  //Possible issues with memory allocation, for very large files
  string arString((istreambuf_iterator<char>(posesAR)),
			    istreambuf_iterator<char>());
  stringstream arPoseStream(arString);
  posesAR.close();
  
  string efString((istreambuf_iterator<char>(posesEF)),
			    istreambuf_iterator<char>());
  stringstream efPoseStream(efString);
  posesEF.close();

  parseText(arPoseStream,1);
  parseText(efPoseStream,-1);

  for (int i=0; i<numPatts; i++){
    getMatrix(i);
  }

  //getMultiMarkerConfig(globalMarkerCoords[0]);
  /*
  for(int i=0;i<globalMarkerCoords.size(); i++){
    visualizeMarkers(globalMarkerCoords[i]);
  }
  */
  outputPLY();
  /*
  double xs = globalMarkerCoords[0](0,3)-globalMarkerCoords[1](0,3);
  double ys= globalMarkerCoords[0](1,3)-globalMarkerCoords[1](1,3);
  double zs= globalMarkerCoords[0](2,3)-globalMarkerCoords[1](2,3);

  cout<<"distance mm:"<<sqrt((pow(xs,2))+(pow(ys,2))+(pow(zs, 2)))<<"\n";
  */
  //test result differences 
  //cout<<"distance mm:"<<sqrt((pow(-676.437,2))+(pow(140.152,2))+(pow(-198.351, 2)))<<"\n";

  
}




