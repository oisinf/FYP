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
//using namespace std; 


//test git upload

// ============================================================================
//	Global variables
// ============================================================================

// Marker detection.
static ARHandle		*gARHandle = new ARHandle();
static ARPattHandle	*gARPattHandle = NULL;
static long		 gCallCountMarkerDetect = 0;

ARParam*		 cparam = new ARParam();


// Transformation matrix retrieval.
static AR3DHandle	*gAR3DHandle = NULL;
static ARdouble		gPatt_width     = 80.0;	// Per-marker, but we are using only 1 marker.
static ARdouble		gPatt_trans[3][4];		// Per-marker, but we are using only 1 marker.
static int	       	gPatt_found = FALSE;	// Per-marker, but we are using only 1 marker.
static int     		gPatt_id;				// Per-marker, but we are using only 1 marker.

// Drawing.
static int gWindowW;
static int gWindowH;
static ARParamLT *gCparamLT = NULL;
static ARGL_CONTEXT_SETTINGS_REF gArglSettings = NULL;
static int gShowHelp = 1;
static int gShowMode = 1;
static int gDrawRotate = FALSE;
static float gDrawRotateAngle = 0;			// For use in drawing.

static int setupParams()
{
  int		  xsize=640, ysize=480;
  AR_PIXEL_FORMAT pixFormat = AR_PIXEL_FORMAT_RGB;
  const char* camPar = "/home/oisin/libs/artoolkit/artoolkit5/bin/Data/calib2.dat";

  if ((arParamClear(cparam, xsize, ysize, AR_DIST_FUNCTION_VERSION_DEFAULT))<0){
    ARLOGi("new param not happening");
    return(false);
  }
  
  if(arParamLoad(camPar, 1, cparam)==0){
     ARLOGw("param load not happenening");
    return(false);
  }
  else
    {
      arParamClearWithFOVy(cparam, xsize, ysize, M_PI_4);
      ARLOGw("using default camera parameters");
    }

  /*
  if (arParamLoad(camPar, 1, cparam) < 0) {
    ARLOGe("setupCamera(): Error loading parameter file %s for camera.\n", cparam);
    return (false);
  }
   else
      {
	arParamClearWithFOVy(cparam, xsize, ysize, M_PI_4); // M_PI_4 radians = 45 degrees.
	ARLOGw("Using default camera parameters for %dx%d image size, 45 degrees vertical field-of-view.\n", xsize, ysize);
      }
  */
if (cparam->xsize != xsize || cparam->ysize != ysize) {
  ARLOGw("*** Camera Parameter resized from %d, %d. ***\n", cparam->xsize, cparam->ysize);
  arParamChangeSize(cparam, xsize, ysize, cparam);
 }

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
static int setupMarker(const char *patt_name, int *patt_id, ARHandle *arhandle, ARPattHandle **pattHandle_p)
{	
  
  if ((*pattHandle_p = arPattCreateHandle()) == NULL) {
    ARLOGe("setupMarker(): Error: arPattCreateHandle.\n");
    return (FALSE);
  }
  
  // Loading only 1 pattern in this example.
  if ((*patt_id = arPattLoad(*pattHandle_p, patt_name)) < 0) {
    ARLOGe("setupMarker(): Error loading pattern file %s.\n", patt_name);
    arPattDeleteHandle(*pattHandle_p);
    return (FALSE);
  }
  //cout<patt_id;
  arPattAttach(arhandle, *pattHandle_p);
  ARLOGi("setupMarker \n");
  
  
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

int detectImage(AR2VideoBufferT *image)
{
  ARdouble err;
  ARMarkerInfo   *markerInfo;
  int j, k;
  
  gCallCountMarkerDetect++;
  if (arDetectMarker(gARHandle, image) < 0) {
    ARLOGi("No marker");
    exit(0);
  }
  markerInfo = arGetMarker(gARHandle);
  k = -1;
  for (j = 0; j < gARHandle->marker_num; j++) {
    std::cout<<"\n"<<gPatt_id<<" patt id \n";
    std::cout<<gARHandle->markerInfo[j].id<<" info id \n";
    std::cout<<gARHandle->markerInfo[j].cf<<" info cf \n";
    
    if (markerInfo[j].id == gPatt_id) {
      if (k == -1) {k = j; /*First marker detected. */
	ARLOGi("marker detected");
      }
      else if (markerInfo[j].cf > markerInfo[k].cf) k = j; // Higher confidence marker detected.
    }
  }
  
  if (k != -1) {
    // Get the transformation between the marker and the real camera into gPatt_trans.
    err = arGetTransMatSquare(gAR3DHandle, &(gARHandle->markerInfo[k]), gPatt_width, gPatt_trans);
        ARLOGi("marker detected");

    //printf("%s %f", " ", gARHandle->markerInfo[k].cf);
    //unsure what err is. 
    printf ("%s %f"," ",  err);
    int i ,j; 
    
    FILE * fp; 
    
    fp=fopen("/home/oisin/libs/TestLogs/gPatt_trans.txt", "w");
    for (i=0; i<3; i++){
      for (j=0; j<4; j++){
	fprintf(fp, "%f %s", gPatt_trans[i][j]," ");
      }
      fprintf(fp, "\n");}
    fclose(fp);
    
    gPatt_found = TRUE;
  } else {
    gPatt_found = FALSE;
  }  
}

int main(void)
{
  ARLOGi("main");
  char    patt_name[]  = "Data/hiro.patt";
  //char    patt_name[]  = "Data/kanji3.patt";
  AR2VideoBufferT *image = new AR2VideoBufferT();
  ARUint8* im;

  //Can put Data folder in bin or share, arUtilChange changes it to share. as everything in bin no need
  //arUtilChangeToResourcesDirectory(AR_UTIL_RESOURCES_DIRECTORY_BEHAVIOR_BEST, NULL);

  setupParams();
  
 
  ///Load two markers 
  // Load marker(s).
  if (!setupMarker(patt_name, &gPatt_id, gARHandle, &gARPattHandle)) {
    ARLOGe("main(): Unable to set up AR marker.\n");
    cleanup();
    exit(-1);
  }
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

  std::string logfile = "/home/oisin/libs/TestLogs/Testlogs/hiroTest.klg";
  RawLogReader * logreader; 
  Resolution::get(640, 480);
  logreader = new RawLogReader(logfile);
  
  while (logreader->grabNext())
    {
      
      //printf("%d %s", logreader->currentFrame, " ");
      
      im = logreader->decompressedImage;
      image->buff = im ;
      image->bufPlanes = NULL;
      image->bufPlaneCount= 0; 
      image->buffLuma = image->buff; 
      image->fillFlag = 1; 
    
      detectImage(image);

      /*
      ofstream myfile; 
      const char *path = "/home/oisin/libs/TestLogs/Poses/Frames.txt";
      myfile.open (path);
      myfile <<logreader->currentFrame<< "\n";
      myfile.close();
      */
    }
  
  ARLOGi("main end");
  return (0);
}



