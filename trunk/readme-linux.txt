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
(note: this command will not work unless the datasets are correctly placed)
	./bin/mm_calibrator -d /home/steve/calibration/data/mmcal_test -n 1 -i

