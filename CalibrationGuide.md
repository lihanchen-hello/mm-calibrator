# Introduction #

Although the package is capable of performing both intrinsic and extrinsic calibration using one set of input sequences, best results are generally achieved by doing intrinsic calibration first (separately), and then feeding the results into the extrinsic calibration algorithm.

# Intrinsic Calibration #

**Sample command:**
./mm-calibrator -d ~/intrinsics\_calib\_folder -i -t 0 -x 12 -y 8 -s -u

# Extrinsic Calibration #

**Sample command:**
./mm-calibrator -d ~/extrinsics\_calib\_folder -3 -n 2 -t 1 -x 12 -y 8 -s -u