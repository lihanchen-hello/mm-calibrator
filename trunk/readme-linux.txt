mm-calibrator [[ LINUX ]] README

TO INSTALL:
===========
In bash, navigate to main directory (containing this file)

Prepare a binary directory
	mkdir bin

Run CMake in console
	cmake .

Run make in console
	make

TO TEST:
========
note: 	This command will not work unless the datasets are correctly placed. 
		Each camera (even if calibrating just one) should be stored under its 
		own numbered directory, e.g. mmcal_test/0/ mmcal_test/1/ etc...
	
	./bin/mm_calibrator -d /home/steve/calibration/data/mmcal_test -n 1 -i

