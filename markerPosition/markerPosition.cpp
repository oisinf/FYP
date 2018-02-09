/**This program is to parse text files containing 
 *frames and poses. 
 */

//Includes
#include <iostream>
#include <string.h>
#include <fstream>
#include <stdio.h>
#include <sstream>
using namespace std; 


//Variables

//Ensure path names correct as no validitation as of yet
//AR poses and frames (pf)
const char *pfAR = "../TestLogs/ARLogReaderFrames&Poses/test.txt";
const char *pfEF = "../TestLogs/EFFrames&Poses/Test1.txt";

//ints to hold frame and patt nums
int currentFrame, currentPatt;

//strings to temporarily store values before conversion to ints/doubles
string  frame, patt, row1, row2, row3, row4;


//(stringstream& posesStream)
int parseText(string line)
{
  /*
  switch(line){
  case line.empty() :
    return true;
  case line.find("Frame")!=string::npos :
    frame = line.substr(5);
    istringstream(frame)>>currentFrame;
  case line.find("Patt")!=string::npos :
    patt = line.substr(4); 
    istringstream(patt)>>currentPatt;
  case line.find("r0")!=string::npos :
    row1 = line.substr(2);
  case line.find("r1")!=string::npos :
    row2 = line.substr(2);
  case line.find("r2")!=string::npos :
    row3 = line.substr(2);
  case line.find("r3")!=string::npos :
    row4 = line.substr(2);
  default : return false; 
  */
  
    //Matrix filled
    if (line.empty()){
      //cout<<lineIncr<<"\n";
       return true;  
    }
    else if (line.find("Frame")!=string::npos){
    frame = line.substr(5);
    istringstream ("frame")>> currentFrame;
    //current frame, need to find corresponding frame in EF poses
    //cout<<currentFrame<<"\n";
    }
    else if (line.find("Patt")!=string::npos){
      patt = line.substr(4);
      istringstream(patt)>>currentPatt;
      //cout<<currentPatt<<"\n";
    }
    else if(line.find("r0")!=string::npos){
      row1 = line.substr(2);
      //cout<<row1<<"\n";
    }
    else if(line.find("r1")!=string::npos){
      row2 = line.substr(2);
      //cout<<row2<<"\n";
    }
    else if(line.find("r2")!=string::npos){
      row3 = line.substr(2);
      // cout<<row3<<"\n";
    }
    else if(line.find("r3")!=string::npos){
      row4 = line.substr(2);
    }
  
    return false; 
}


int main ()
{
  //return eigen matrix of two poses at that time, then transform
  ifstream posesAR;
  ifstream posesEF;

  string line;
  
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
  
  int arFrame, arPatt, efFrame;
  string arR1, arR2, arR3, efR1, efR2, efR3, efR4;

  while(!arPoseStream.eof()){

    getline(arPoseStream, line); 

    if(parseText(line)){
      //cout<<currentFrame<<"\n"<<currentPatt<<"\n"<<row1<<"\n"<<row2<<"\n"<<row3<<"\n \n";
      
      arFrame=currentFrame;
      arPatt=currentPatt;
      arR1= row1;
      arR2= row2;
      arR3= row3;
      cout<<"AR "<<arFrame<<"\n"<<arPatt<<"\n"<<arR1<<"\n"<<arR2<<"\n"<<arR3<<"\n";
      
      //get corresponding EF frame
      while(!efPoseStream.eof()){
	
	getline(efPoseStream, line);
	
	if(parseText(line)){
	  efFrame = currentFrame;
	  efR1 = row1;
	  efR2 = row2;
	  efR3 = row3;
	  efR4 = row4;
	  
	  //createMatrix(arPatt);
	  
	}
      }
    }
  }
  cout<<"EF "<<efFrame<<"\n"<<efR1<<"\n"<<efR2<<"\n"<<efR3<<"\n"<<efR4<<"\n";

}



