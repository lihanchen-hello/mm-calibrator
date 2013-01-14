mm-calibrator [[ LINUX ]] README

TO INSTALL:
===========
In bash, navigate to main directory (containing this file), then:

	cd build
	cmake ..
	make

DEPENDENCIES:
=============
If cmake complains about boost dependencies, try going into synaptic and looking for:

	libboost-filesystem-dev
	libboost-system-dev

If it doesn't work when launched, it could be that OpenCV compiled without some dependencies required by mm-calibrator. You may need to use Synnaptic to install packages such as libgtk2.0-dev and pkg-config, and then re-configure cmake for OpenCV to include GTK, before re-making and re-installing it. Then you can try compiling and running mm-calibrator again afterwards.

TO TEST:
========
note: 	This command will not work unless the datasets are correctly placed. 
		Each camera (even if calibrating just one) should be stored under its 
		own numbered directory, e.g. mmcal_test/0/ mmcal_test/1/ etc...
	
	../bin/mmcalibrator -d /home/user/Downloads/miricle-307k-sample -n 1 -i -x 6 -y 4

