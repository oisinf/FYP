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

//AR poses and frames (pf)
const char *pfAR = "../TestLogs/ARLogReaderFrames&Poses/test.txt";
const char *pfEF = "../TestLogs/EFFrames&Poses/test1.txt";

int parseARText()
{
   ifstream posesAR;
  //ifstream EFPoses(EFpf);
  posesAR.open(pfAR, ios::in);
  string line, frame, patt;
  size_t f1, f2; 
  const string frameLine = "Frame";
  const string pattLine = "Patt"; 
  
  int currentFrame, currentPatt;
  int lineIncr=0;
  string row1, row2, row3; 
  
  if(!posesAR.is_open()) {
      cout<<"cannot open file"<<pfAR;
      return false; 
    }
  
  string posesString((istreambuf_iterator<char>(posesAR)),
			    istreambuf_iterator<char>());
  istringstream posesStream(posesString);

  posesAR.close();

  cout<<posesStream;
  
  //read multiple lines into single m
  while (!posesAR.eof()){
    //while (getline (posesAR, line)){
    getline(posesAR, line);

    f1 = line.find(frameLine);
    f2 = line.find(pattLine);
    
    if (f1!=string::npos){
    frame = line.substr(5);
    istringstream (frame)>> currentFrame;
    //current frame, need to find corresponding frame in EF poses
    //cout<<currentFrame<<"\n";
    }
    if (f2!=string::npos){
      patt = line.substr(4);
      istringstream(patt)>>currentPatt;
      
      //cout<<currentPatt<<"\n";
    }

    if(line.find("r0")!=string::npos){
      row1 = line.substr(2);
      // cout<<row1<<"\n";
    }
    else if(line.find("r1")!=string::npos){
      row2 = line.substr(2);
      //cout<<row2<<"\n";
    }
    else if(line.find("r2")!=string::npos){
      row3 = line.substr(2);
      //cout<<row3<<"\n";
    }
    
    lineIncr++;
    // cout<<currentFrame<<"  "<<currentPatt<<"  "<<"\n";
 
  }
  
}


int main ()
{
  //return eigen matrix of two poses at that time, then transform
  parseARText();
  
 
 
}


