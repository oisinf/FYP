f = open("calib7X92.txt", "w")
f.write('export ARTOOLKIT5_VCONF="-module=Image  -loop ') 
for i in range (1,1488):
	f.write("-image=/home/oisin/libs/TestLogs/CalibrationFrames/7X9calib2/"+str(i)+".jpg ")

f.close()
	
