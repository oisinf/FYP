#include <stdio.h>
#include <string.h>
#ifdef _WIN32
#  define snprintf _snprintf
#  define _USE_MATH_DEFINES
#endif
#include <stdlib.h>					// malloc(), free()
#include <math.h>
#ifdef __APPLE__
#  include <GLUT/glut.h>
#else
#  include <GL/glut.h>
#endif
#include <AR/config.h>
#include <AR/video.h>
#include <AR/param.h>			// arParamDisp()
#include <AR/ar.h>
#include <AR/gsub_lite.h>
#include <ARUtil/time.h>
#include <iostream>
#include <fstream>
#include <../include/reader/RawLogReader.h>

#include <opencv/cv.h>
//using namespace std; 

// Marker detection.
static ARHandle		*gARHandle = new ARHandle();
static ARPattHandle	*gARPattHandle = NULL;
static long		 gCallCountMarkerDetect = 0;

ARParam*		 cparam = new ARParam();


// Transformation matrix retrieval.
static AR3DHandle	*gAR3DHandle = NULL;
static ARdouble		gPatt_width     = 100.0;	// Per-marker, but we are using only 1 marker.
static ARdouble		gPatt_trans[3][4];		// Per-marker, but we are using only 1 marker.
static int	       	gPatt_found = FALSE;	// Per-marker, but we are using only 1 marker.
static int 	        gPatt_id;				// Per-marker, but we are using only 1 marker.

// Drawing.

static ARParamLT *gCparamLT = NULL;
static ARGL_CONTEXT_SETTINGS_REF gArglSettings = NULL;

FILE * fp; 

static int setupParams()
{
  //Pixel size
  int		  xsize=640, ysize=480;
  //Pixel format
  AR_PIXEL_FORMAT pixFormat = AR_PIXEL_FORMAT_RGB;
  //File path for camera parameters
  const char* camPar = "/home/oisin/libs/artoolkit/artoolkit5/bin/Data/5X7calib1.dat";

  //Clear AR param so can load new values into it
  if ((arParamClear(cparam, xsize, ysize, AR_DIST_FUNCTION_VERSION_DEFAULT))<0){
    ARLOGi("new param not happening");
    return(false);
  }

  //Load camera params into ar param struct
  if(arParamLoad(camPar, 1, cparam)<0){
     ARLOGw("param load not happenening");
     arParamClearWithFOVy(cparam, xsize, ysize, M_PI_4);
     ARLOGw("using default camera parameters");
    return(false);
  }
  
  if (cparam->xsize != xsize || cparam->ysize != ysize) {
    ARLOGw("*** Camera Parameter resized from %d, %d. ***\n", cparam->xsize, cparam->ysize);
    arParamChangeSize(cparam, xsize, ysize, cparam);
  }

  //Intialize param lookup table from camera parameters in ar param struct, 
  if ((gCparamLT = arParamLTCreate(cparam, AR_PARAM_LT_DEFAULT_OFFSET)) == NULL) {
    ARLOGe("setupCamera(): Error: arParamLTCreate.\n");
    return (FALSE);
  }
  
  if( (gARHandle=arCreateHandle(gCparamLT)) == NULL ) {
    ARLOGe("Error: arCreateHandle.\n");
    exit(0);
  }
  if( arSetPixelFormat(gARHandle, pixFormat) < 0 ) {
    ARLOGe("Error: arSetPixelFormat.\n");
    exit(0);
  }
  if( (gAR3DHandle=ar3DCreateHandle(cparam)) == NULL ) {
    ARLOGe("Error: ar3DCreateHandle.\n");
    exit(0);
  }
  
  arParamDisp(cparam);
  return (TRUE);
}

//set up maker r func
static int setupMarker(const char **patterns, int *pattIDs , ARHandle *arhandle, ARPattHandle **pattHandle_p, const int numMarkers )
{	
  const int size = 16;
  
  if ((*pattHandle_p = arPattCreateHandle2(size, numMarkers)) == NULL) {
    ARLOGe("setupMarker(): Error: arPattCreateHandle.\n");
    return (FALSE);
  }

  for (int i = 0;  i < numMarkers ; i++)
    {
      std::cout<<"Pattern file path "<<i<<"  "<<patterns[i]<<"\n";

      if ((pattIDs[i]= arPattLoad(*pattHandle_p, patterns[i])) < 0) {
	
	ARLOGe("setupMarker(): Error loading pattern file %s.\n");
	return (FALSE);
      }
      std::cout<<"pattern id: "<<pattIDs[i]<<"\n";
  //std vector of arhandles, attach patt handle
      arPattAttach(arhandle, *pattHandle_p);
      ARLOGi("setupMarker \n");

 
    }

  
  return (TRUE);
}
//Clean up
static void cleanup(void)
{   
  ARLOGi("cleanup");
  arglCleanup(gArglSettings);
  gArglSettings = NULL;
  arPattDetach(gARHandle);
  arPattDeleteHandle(gARPattHandle);
  arVideoCapStop();
  ar3DDeleteHandle(&gAR3DHandle);
  arDeleteHandle(gARHandle);
  arParamLTFree(&gCparamLT);
  arVideoClose();

}

int detectImage(AR2VideoBufferT *image, int currentFrame, int *pattIDs)
{
  ARdouble err;
  ARMarkerInfo   *markerInfo;
  int j, k, i;
  
  gCallCountMarkerDetect++;
  
  if (arDetectMarker(gARHandle, image) < 0) {
    ARLOGi("No marker");
    exit(-1);
  }
  markerInfo = arGetMarker(gARHandle);
  
  
  //Need another for loop to loop through patt ids 

  for (i = 0; i<gARHandle->marker_num; i++)
    {
      k = -1;
      //Increment through marker num on handle, multiple patterns attached with createPattHandle2
      for (j = 0; j < gARHandle->marker_num; j++) {
	/*
	  std::cout<<"\n"<<gPatt_id<<" patt id \n";
	  std::cout<<gARHandle->marker_num<<"\n";
	  std::cout<<gARHandle->markerInfo[j].id<<" info id \n";
	  std::cout<<gARHandle->markerInfo[j].cf<<" info cf \n";
	*/	
	if (markerInfo[j].id == pattIDs[i]) {
	  if (k == -1) {
	    if (markerInfo[j].cf >=0.7){
	      k = j;
	      std::cout<<"pattern id detected "<<markerInfo[j].id<<" in frame "<<currentFrame<<"\n";

	    }/*First marker detected. */
	    //ARLOGi("marker detected");
	  }
	  //might not need this if statment
	  //else if (markerInfo[j].cf > markerInfo[k].cf) k = j; // Higher confidence marker detected.
	}
	//need to add another else if statment, i.e if marker j and k both detected. Use a flag? 
      }
      if (k != -1) {
	// Get the transformation between the marker and the real camera into gPatt_trans.
	err = arGetTransMatSquare(gAR3DHandle, &(markerInfo[k]), gPatt_width, gPatt_trans);
	//ARLOGi("marker detected");
	
	//printf("%s %f", " ", gARHandle->markerInfo[k].cf);
	//unsure what err is. 
	//printf ("%s %f"," ",  err);
	
	fprintf(fp, "%s %i %s", "Frame ",currentFrame, "\n");
	fprintf(fp, "%s %i %s", "Patt" , pattIDs[i], "\n");
	int i ,j;  
	for (i=0; i<3; i++){
	  for (j=0; j<4; j++){
	    fprintf(fp, "%f %s", gPatt_trans[i][j]," ");
	  }
	  fprintf(fp, "\n");
	}
	fprintf(fp, "\n");
	// fprintf(fp, "%i %i %i %i %s", 0, 0, 0, 1, "\n"); //to make them homogneous, can do this elsewhere
	
	
	
	gPatt_found = TRUE;
      }
      else {
	gPatt_found = FALSE;
      }
    }
}

int main(void)
{
  //char    patt_name[]  = "Data/hiro.patt";
  //char    patt_name[]  = "Data/kanji.patt";
  
  const char* patterns [] = { "Data/kanji.patt", "Data/hiro.patt"};
  const int numMarkers= (sizeof(patterns)/sizeof(patterns[0]));
  int pattIDs[numMarkers]; 
  std::cout<<numMarkers;
  AR2VideoBufferT *image = new AR2VideoBufferT();
  ARUint8* im;
  int currentFrame; 
  //Can put Data folder in bin or share, arUtilChange changes it to share. as everything in bin no need
  //arUtilChangeToResourcesDirectory(AR_UTIL_RESOURCES_DIRECTORY_BEHAVIOR_BEST, NULL);

  setupParams();
  ///Load two markers 
  // Load marker(s).
  if (!setupMarker(patterns, pattIDs, gARHandle, &gARPattHandle, numMarkers)) {
    ARLOGe("main(): Unable to set up AR marker.\n");
    cleanup();
    exit(-1);
  }

  /*
  ARMarkerInfo *testMarkerInfo;
  testMarkerInfo = arGetMarker(gARHandle);
  std::cout<<gARHandle->marker_num;
  std::cout<<gARHandle->markerInfo[0].id;
  // std::cout<<testMarkerInfo[1].id;

	

  /*
  for (int j=0; j< gARHandle->marker_num; j++)
    {
      ARLOGi("this is working?");
      std::cout<<gARHandle->marker_num;
      std::cout<<testMarkerInfo[j].id;
    }
  /*
  std::cout<<"start \n";
  std::cout<< gARHandle->arDebug<<"\n";
  std::cout<< gARHandle->arPixelFormat<<"\n";
  std::cout<< gARHandle->arPixelSize<<"\n";
  std::cout<< gARHandle->arLabelingMode<<"\n";
  std::cout<< gARHandle->arLabelingThresh<<"\n";
  std::cout<< gARHandle->arImageProcMode<<"\n";
  std::cout<< gARHandle->arPatternDetectionMode<<"\n";
  std::cout<< gARHandle->arMarkerExtractionMode<<"\n";
  std::cout<< gARHandle->marker_num<<"\n";
  std::cout<< gARHandle->markerInfo<<"\n";
  std::cout<< gARHandle->pattHandle<<"\n";
  */
  std::string logfile = "/home/oisin/libs/TestLogs/Testlogs/kanji&hiro.klg";
  RawLogReader * logreader; 
  Resolution::get(640, 480);
  logreader = new RawLogReader(logfile);
  image->buff = new ARUint8[640*480];
  
  fp=fopen("/home/oisin/libs/TestLogs/ARLogReaderFrames&Poses/test1.txt", "w");

  
  while (logreader->grabNext())
    {
      im = logreader->decompressedImage;
            
      cv::Mat cvimage(480, 640, CV_8UC3, im);
      cv::Mat gimage;
      cv::cvtColor(cvimage, gimage, CV_BGR2GRAY);
      /*
      cv::imshow("Test", gimage);
      cv::waitKey(0);
      */
      image->buff = im ;
      image->bufPlanes = NULL;
      image->bufPlaneCount= 0; 
      image->buffLuma = gimage.data;
      image->fillFlag = 1; 
      currentFrame = logreader->currentFrame;
      detectImage(image, currentFrame, pattIDs);
    }

  
  fclose(fp);
  ARLOGi("main end");
  return (0);
}



