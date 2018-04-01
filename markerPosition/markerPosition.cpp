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

/**Typdefs*/

/**4x4 matrix to hold homogenous poses*/
typedef Eigen::Matrix<double, 4, 4> Pose4X4;

/**4x1 matrix to hold x,y,z,w for position averaging*/
typedef Eigen::Matrix<double, 4, 1> XYZ;

/**
 *Constants for file paths, must be recompiled if file path changed
 */
const char *pfAR = "../TestLogs/ARLogReaderFrames&Poses/Demos/Log04.txt";
const char *pfEF = "../TestLogs/EFFrames&Poses/Demos/Log04.txt";

/**Struct to hold frame, patt id and pose from marker detection output*/
struct arInfo{
  int frame;
  int pattID;
  Pose4X4 pose;
};
/**Struct to hold frame frame and pose from elastic fusion output*/
struct efInfo{
  int frame;
  Pose4X4 pose;
};

/**Vector to hold parsed information from marker detection output*/
vector<arInfo> arPoses;

/**Vector to hold parsed information from EF output*/
vector<efInfo> efPoses;

/**Vector to hold calcuated marker vertices*/
vector<Eigen::Matrix4d> tagPoints; 

/**Vector to hold quaterions for SLERP*/
vector<Eigen::Quaternion<double> > globalCoordsQuat;

/**Vector to hold averaged marker poses*/
vector<Pose4X4> globalMarkerCoords;

/**number of markers*/
int numPatts;

/**Vector for pattern names to output in multimarker config file*/
vector<string>patternNames;

/**Method to parse homogeneous coordinate columns from text file input
 *@param row, line of text to be parsed
 *@param pose, reference to the current pose that parsed data will be held in
 *@param flag, current row being parsed
 */
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

/**Method to parse text from text file input for both marker and camera poses
 *@param stream, reference to text file as stringstream
 *@param flag, distinguish between ef and ar input text input as formatting different
 */
int parseText(stringstream& stream, int flag)
{
  //variables to hold parsed data temporarily
  int currentFrame, pattID;
  string r1,r2,r3,r4, line;
  arInfo dataAR;
  efInfo dataEF; 
  Pose4X4 temp;

  //Last row of homogeneous matrix, 0,0,0,1
  temp(3,0) = 0.0;
  temp(3,1) = 0.0;
  temp(3,2) = 0.0;
  temp(3,3) = 1.0;
  string tempName ="";  
  while(!stream.eof())
    {
      getline(stream, line); 
      
      if(line.find("NumPatterns")!=string::npos){
	istringstream(line.substr(12))>>numPatts;
      }
      
      if(line.find("PN")!=string::npos){
        tempName = istringstream(line.substr(3)).str();
	patternNames.push_back(tempName);	
      }
      //Line empty parsed data from one frame already
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
	  //need to scale pose as current x,y,z is in meters rather than mms as ar poses are.
	  temp.block<3,1>(0,3) = temp.block<3,1>(0,3)*1000;
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

/**Method to SLERP between quaternions to get total orientation of marker
 *@return total,  returns total orientation of marker
 */
Eigen::Quaternion<double> slerpQuaternion(){

  
  Eigen::Quaternion<double> temp;
  Eigen::Quaternion<double> total = globalCoordsQuat[0];
  
  for (int i=1; i< globalCoordsQuat.size(); i++){
   
    temp = globalCoordsQuat[i];
    //0.5 to interpolate halfway between quaternions
    total = total.slerp(0.5,temp);
    
  }
  return total; 
}
/**Method to get average marker pose
 *@param currentPatt, current marker average being computed
 */
void getMatrix(int currentPatt){

  cout<<"current pattern matrix being calcuated "<<currentPatt<<endl;

  int avg = 0;
  //Translation vector for average
  XYZ avgTemp;
  avgTemp<<0,0,0,0;

  //Temporary and final  homogeneous pose
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

  //Matrix to transfrom single pose point to 4 vertices of tag
  Eigen::Matrix4d TagPoints;
  TagPoints <<
    0,100,100,0,
    0,0,100,100,
    0,0,0,0,
    1,1,1,1;

  //Matrix to hold vetices
  Eigen::Matrix4d EFPoints;
  EFPoints<<
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0 ,0, 0;

  Eigen::Quaternion<double> total(1,0,0,0);
  Eigen::Quaternion<double> local(1,0,0,0);

  //Iterate through marker poses
  for (int i = 0; i<arPoses.size(); i++){

    //If marker pose id equals current pattern argument
    if (arPoses[i].pattID == currentPatt){

      //Iterate through camera poses
      for(int j = 0; j<efPoses.size(); j++){

	//If marker pose frame equals camera pose frame, then poses were captured at the same frame and are related
       	if(arPoses[i].frame == efPoses[j].frame){
	  
	  //Finding pose of marker relative to camera  i.e. the pose of the marker in the global (ef) coordinate system, multiply arPose by efPose
	  temp =efPoses[j].pose* arPoses[i].pose;
	  
	  //split temp into quaternion+xyz
	  //local rotation i.e rotation of the marker in that frame
	  //push local into vector
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
  
  //iterate through quaternion vector, slerp, then recompose into total
  total = slerpQuaternion();

  cout<<"avg frames current pattern spotted in  "<<avg<<endl;

  //Average x,y,z
  avgTemp = avgTemp/avg;

  
  finalGlobalCoord.block<3,3>(0,0) = total.toRotationMatrix();
  finalGlobalCoord.col(3) = avgTemp;
  
  EFPoints = finalGlobalCoord*TagPoints;
  
  tagPoints.push_back(EFPoints);
  
  globalMarkerCoords.push_back(finalGlobalCoord);

  //Clear quaternion vector for reuse. 
  globalCoordsQuat.clear(); 
}

/**
 *Method to ouput multimarker configuration file
 */
void getMultiMarkerConfig(){
  
  ofstream fp;

  //Identity matrix i.e first marker
  Pose4X4 temp;
  temp<<
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0 ,0, 1;

  //File name
  fp.open("/home/oisin/libs/TestLogs/MultimarkerConfigs/Log04.dat");

  //Number of patterns
  fp<<numPatts<<endl<<endl;

  //First pattern file path
  fp<<patternNames[0]<<endl;
  //Marker size
  fp<<200.00<<endl;
  fp<<temp.block<3,4>(0,0)<<endl<<endl;

  //Iterate through marker poses and multiply by inverse to get pose of markers relative to the identity matrix
  for(int i = 1; i<globalMarkerCoords.size(); i++){

    temp =  globalMarkerCoords[0].inverse()*globalMarkerCoords[i];
    fp<<patternNames[i]<<endl;
    fp<<200.00<<endl;
    fp<<temp.block<3,4>(0,0)<<endl<<endl;
  }

  fp.close();
}

/**
 *Method to ouput .ply file for visualization
 */
void outputPLY(){

  //output stream 
  ofstream fp;

  //File name
  fp.open("/home/oisin/libs/TestLogs/Testlogs/Demos/Log04.ply");

  //Preamble
  fp<<"ply\nformat ascii 1.0\ncomment visualization of marker positions by Oisin Feely\nelement vertex "<<tagPoints.size()*4<<"\nproperty float32 x\nproperty float32 y\nproperty float32 z\nelement face "<<tagPoints.size()<<"\nproperty list uint8 int32 vertex_index\nend_header\n";

  for(int i=0;i<tagPoints.size(); i++){

    //divide by 1000 as current pose in mms and ef .ply is in meters, 
    Eigen::Matrix<double, 3,4> tagPts = (tagPoints[i].block<3,4>(0,0))/1000;

    //Iterate through vertices
    for(int j=0; j<4;j++){
      Eigen::RowVector3d flatten = tagPts.col(j);
      fp<<flatten<<endl;
    }
  }
  int count=0;
  //Specifying vertices for faces
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

/**
 *Main method
 */
int main ()
{
  //Declare ifstreams to read in txt data, to be passed to parseText
  ifstream posesAR;
  ifstream posesEF;

  
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

  string arString((istreambuf_iterator<char>(posesAR)),
			    istreambuf_iterator<char>());
  stringstream arPoseStream(arString);
  posesAR.close();
  
  string efString((istreambuf_iterator<char>(posesEF)),
			    istreambuf_iterator<char>());
  stringstream efPoseStream(efString);
  posesEF.close();

  parseText(arPoseStream,2);
  parseText(efPoseStream,-1);
  
  for (int i=0; i<numPatts; i++){
    getMatrix(i);
  }

  outputPLY();
  getMultiMarkerConfig();

}




