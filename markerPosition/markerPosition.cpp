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


int main ()
{
  //Declare ifstreams to read in txt data, to be passed to parseText
  ifstream posesAR;
  ifstream posesEF;

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

  //Declare 
  /*
  //Declare strings to be parsed
  string  line;
  string lineEF     ; 

  //open ifstreams
  posesAR.open(pfAR, ios::in);
  posesEF.open(pfEF, ios::in);

  
 

  

  //Initialize int and string to hold info from parsing
  int arFrame, arPatt, efFrame;     
  string arR1, arR2, arR3, efR1, efR2, efR3, efR4;

  while(!arPoseStream.eof()){
    getline(arPoseStream, line); 

    //returns true if empty line
    if(parseText(line, true, -1)){
      //cout<<currentFrame<<"\n"<<currentPatt<<"\n"<<row1<<"\n"<<row2<<"\n"<<row3<<"\n \n";
      
      arFrame=currentFrame;
      arPatt=currentPatt;
      arR1= row1;
      arR2= row2;
      arR3= row3;


      //cout<<"AR "<<arFrame<<"\n"<<arPatt<<"\n"<<arR1<<"\n"<<arR2<<"\n"<<arR3<<"\n";

      //get corresponding EF frame
      while(!efPoseStream.eof()){
	getline(efPoseStream, lineEF);
       
	if(parseText(lineEF, false, arFrame)){
	  efFrame = arFrame;
	  //cout<<currentFrame;
	  efR1=row1;
	  efR2=row2;
	  efR3=row3;
	  efR4=row4;
	  // cout<<"EF"<<efFrame<<"\n"<<efR1<<"\n"<<efR2<<"\n"<<efR3<<"\n"<<efR4<<"\n";
	  // cout<<"AR "<<arFrame<<"\n"<<arPatt<<"\n"<<arR1<<"\n"<<arR2<<"\n"<<arR3<<"\n";

	  break;
	  
	}
      }

      //***Idea, parse artext into a matrix, patt array of matrix and frame id's. Next iterate through array, extract frame. parse ef poses to identify pose at frame. 

      //cout<<"AR "<<arFrame<<"\n"<<arPatt<<"\n"<<arR1<<"\n"<<arR2<<"\n"<<arR3<<"\n";
    }
  }
  */
}




