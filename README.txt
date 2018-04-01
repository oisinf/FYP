####Elastic Fusion Installation

EF can be cloned from https://github.com/mp3guy/ElasticFusion and built following the instructions given. 

For use in this project replace the MainController.cpp and CMakeLists.txt with MainController.cpp and CMakeLists.txt included in this project. 


####ARToolKit File Structure

Bin-Contains inputImage runnable, camera parameter file (.dat) and marker pattern files (.dat)
	-Also contains runnable multiTest, to use multiTest use this commmand to change the video configuration environment settings variable. 
	-export ARTOOLKIT5_VCONF="v4l2src device=/dev/video1 ! video/x-raw-yuv,width=480,height=320 ! videoscale ! video/x-raw-yuv,width=480,height=320 ! ffmpegcolorspace ! capsfilter caps=video/x-raw-rgb,bpp=24 ! identity name=artoolkit ! fakesink";

Examples-Contains inputImage and multiTest, both programs can be compiled using make. For both programs 
	file paths are hardcoded so if you wish to change the log file read by inputImage as an input, the text file output name of 		inputImage, or the multimarker configuration file read by multiTest the programs must be recompiled. 
	
	
Include/Lib-Libraries required by make file. reader in include was provided by Louis Gallagher. Lib can be recompiled with makefile in SRC, 		ensure library dependencies are there before doing so. Have included libraries in uploaded folder to moodle.  List of dependencies can 		be found here  https://github.com/artoolkit/artoolkit5

Share-Contains files for use with multiTest, this includes generated multimarker configuration files (Log01/02/04.dat), pattern files in Data 
	as well as default example (marker.dat) and associated pattern files. 

#####TestLogs Folder

ARLogReaderFrames&Poses-Output from inputImage, i.e. marker poses, frames identified in and marker id's. 

CalibrationFrames-Once of JPEG ouput of frames from calibration logs of checkboard at various angles. 

EFFRames&Poses-Output from EF

MultimarkerConfigs-Generated multimarker configuration file
Testlogs-Recorded logs 

VCONFCalibSettings-Python script to set video configuration settings environment variable to calibration frame JPEGs. Text files contain 
	this terminal command. 

######MarkerPosition Folder

Contains markerPosition runnable for generated multimarker configuration files and .ply output of marker positions for vizualisation. 
	Input and outputs are hardcoded but can be recompiled with g++ markerPosition.cpp -o markerPosition. 

######Using programs

Run EF with chosen log. 
Run inputImage with same log. 
Run markerPosition with text outputs from EF and inputImage 
View visualization (.ply) with meshlab and use (.dat) multimarker configuration file with multiTest. 


#######Folder upload
Uploaded folder on Moodle contains 

Testlogs-Demos-This folder containe 4 logs to run with EF and inputImage and ouputted .ply files at each stage. There is also the combined .ply files (.mlp) which can be viewed with meshLab. 

ARToolkitLibraries-Compiled library dependencies. Can add them to the lib folder if you do not wish to recompile the lib folder with the makefile in src. 




