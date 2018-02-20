/**This program is to parse text files containing 
 *frames and poses. 
 */

//Includes
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
using namespace std; 

typedef Eigen::Matrix<double, 4, 4> Pose4X4;
typedef Eigen::Matrix<double, 4, 1> XYZ; 
//Variables
//typedef Eigen::Quaternion<double> quatern; 
//Ensure path names correct as no validitation as of yet
//AR poses and frames (pf)
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

//hold avg x,y,z of each patt
vector<XYZ> globalXYZ;

vector<Eigen::Quaternion<double> > globalCoordsQuat;
vector<Pose4X4> globalMarkerCoords;
//Quaterions to hold global coords

//intial quaternion
//Eigen::Quaterniond finalCoord;


int numPatts; 

void parseCols(string row, Pose4X4& pose, int flag)
{
  double z = 0.0;
  double one = 1.0;
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
     
   //possibly 1,0,0,0, unsure what to instansiate quaternions as ***new note, identity = 1,0,0,0
   Eigen::Quaternion<double> total(1,0,0,0);
   Eigen::Quaternion<double> local(1,0,0,0);
   
  for (int i = 0; i<arPoses.size();i++){
    if (arPoses[i].pattID == currentPatt){
      
      for(int j = 0; j<efPoses.size(); j++){
	if(arPoses[i].frame == efPoses[j].frame){
	  //cout<<"ar pose frame"<<arPoses[i].frame<<endl;
	  //cout<<"ef pose frame"<<efPoses[j].frame<<endl;

	  //Finding pose of arPose relative to efPose i.e. the pose of the marker in the global (ef) coordinate system, multiply arPose by efPose
	  temp = arPoses[i].pose*efPoses[j].pose;

	  //split temp into quaternion+xyz
	  
	  //local rotation i.e rotation of the marker in that frame
	  //push local into vector, slerp vector together
	  local = temp.block<3,3>(0,0);
	  globalCoordsQuat.push_back(local);
	  //Adding up all x,y,z to get average x,y,z of marker
	  avgTemp = avgTemp+temp.col(3); 
	  // cout<<avgTemp<<endl;

	  //Increment for each frame pose is in
	  avg++;
	}
      }
    }
  }

  //iterate through quaternion vector, slerp, then recompose into 
  total = slerpQuaternion();
  avgTemp = avgTemp/avg;


  finalGlobalCoord.block<3,3>(0,0) = total.toRotationMatrix();
  finalGlobalCoord.col(3) = avgTemp;

  cout<<finalGlobalCoord<<endl; 
  
  //cout<<"average"<<avgTemp<<endl;
  //Average x,y,z coordinates
  globalMarkerCoords.push_back(finalGlobalCoord);
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

  //testing
  //cout<<"pattern 1 x,y,z,w \n"<<finalXYZ[0]<<endl;
  //cout<<"pattern 2 x,y,z,w \n"<<finalXYZ[1]<<endl;

  //quick euclidean avg
  // double xs=
  /*
  double  xs = finalXYZ[0](0,0)-finalXYZ[1](0,0);
  double ys= finalXYZ[0](1,0)-finalXYZ[1](1,0);
  double zs= finalXYZ[0](2,0)-finalXYZ[1](2,0);

  cout<<"distance mm:"<<sqrt((pow(xs,2))+(pow(ys,2))+(pow(zs, 2)))<<"\n";

  //test result differences 
  //cout<<"distance mm:"<<sqrt((pow(-676.437,2))+(pow(140.152,2))+(pow(-198.351, 2)))<<"\n";
  //cout<<"distance mm:"<<sqrt((pow(-1428.25,2))+(pow(354.93,2))+(pow(1009.2, 2)))<<"\n";

  
  
  multiMarkerConfig(); 
  // cout<<"vector "<<finalXYZ[0]<<endl;
  //cout<<"tranpose"<<finalXYZ[0].inverse()<<endl;
  
  //testing loop
  for (int i = 0; i<arPoses.size(); i++)
    {
      cout<<arPoses[i].frame<<"\n";
      cout<<arPoses[i].pattID<<"\n";
      cout<<arPoses[i].pose(0,0)<<endl;
      cout<<arPoses[i].pose(2,3)<<endl;
      }
  cout<<"AR Pose"<<endl;
  cout <<arPoses[20].frame<<"\n";
  cout<<arPoses[20].pattID<<"\n";
  for (int i = 0; i<4; i++){
    for (int j = 0; j<4; j++){
      cout<<arPoses[20].pose(i,j)<<endl;
    }
    }  
      cout<<"EF Pose"<<endl;
      cout <<efPoses[20].frame<<"\n";
      for (int i = 0; i<4; i++){
      for (int j = 0; j<4; j++){
      cout<<efPoses[20].pose(i,j)<<endl;
    }
    }
  */ 
}




