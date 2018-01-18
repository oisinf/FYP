f = open("pythonCalib.txt", "w")
f.write('export ARTOOLKIT5_VCONF="-module=Image  -loop ') 
for i in range (1,1070):
	f.write("-image=/home/oisin/libs/TestLogs/CalibrationFrames/"+str(i)+".jpg ")

f.close()
	
