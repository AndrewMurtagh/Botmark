forkin:
	gcc forkin.c -o ForwardKinematics -lm -lrt

invkin:
	gcc invkin.c -o InverseKinematics -lm -lrt

ogm: 
	g++ ogm.cpp -o OccupancyGridMapping -lm -lrt

prm:
	g++ prm.cpp -o ProbabilisticRoadmap -lm -lrt

haar:
	g++ haar.cpp -o Haar `pkg-config --cflags --libs opencv` -lrt

corrnav:
	g++ corrnav.cpp -o CorriderNavigation `pkg-config --cflags --libs opencv` -lrt
	
tracking:
	g++ tracking.cpp -o ObjectTracking `pkg-config --cflags --libs opencv` -lrt
