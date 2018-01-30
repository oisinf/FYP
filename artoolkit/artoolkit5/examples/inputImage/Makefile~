
INC_DIR= ../../include
LIB_DIR= ../../lib
BIN_DIR= ../../bin

CC=g++ -ggdb
LDFLAG=-L/usr/lib/x86_64-linux-gnu -L/usr/lib64 -L$(LIB_DIR)/linux-x86_64 -L$(LIB_DIR) -L$(INC_DIR)/reader	
LIBS= -O2 -lARgsub_lite -lARvideo -lAR -lARICP -lAR -lARUtil -lglut -lGLU -lGL -lX11 -lm -lpthread -ljpeg -lz -pthread -lgstreamer-0.10 -lgobject-2.0 -lgmodule-2.0 -lgthread-2.0 -lxml2 -lglib-2.0 -lRawLog -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_objdetect
CFLAG= -O3 -fPIC -march=core2 -DHAVE_NFT=1 -pthread -I/usr/include/gstreamer-0.10 -I/usr/include/glib-2.0 -I/usr/lib/x86_64-linux-gnu/glib-2.0/include -I/usr/include/libxml2 -I$(INC_DIR)

OBJS = 
HEADERS =

all: $(BIN_DIR)/inputImage

$(BIN_DIR)/inputImage: inputImage.o $(OBJS) 
	$(CC) -o $(BIN_DIR)/inputImage inputImage.o $(OBJS) $(LDFLAG) $(LIBS)

inputImage.o:  inputImage.c $(HEADERS) 
	 $(CC)  -c $(CFLAG) inputImage.c  	
clean:
	rm -f *.o
	rm -f $(BIN_DIR)/inputImage

allclean:
	rm -f *.o
	rm -f $(BIN_DIR)/inputImage
	rm -f Makefile

distclean:
	rm -f *.o
	rm -f Makefile
