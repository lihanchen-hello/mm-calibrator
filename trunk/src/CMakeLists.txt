cmake_minimum_required(VERSION 2.8)

PROJECT( ../bin/mm_calibrator )

FIND_PACKAGE( OpenCV REQUIRED )
#FIND_PACKAGE( Boost REQUIRED )
find_package(Boost COMPONENTS system filesystem REQUIRED)

ADD_EXECUTABLE( ${PROJECT_NAME} 
	mm_calibrator.cpp 	mm_calibrator.hpp
	cv_utils.cpp		cv_utils.hpp
	im_proc.cpp		im_proc.hpp
	calibration.cpp		calibration.hpp
	intrinsics.cpp		intrinsics.hpp
	extrinsics.cpp		extrinsics.hpp
)

TARGET_LINK_LIBRARIES( 
	${PROJECT_NAME} 
	${OpenCV_LIBS} 
	${Boost_FILESYSTEM_LIBRARY}
	${Boost_SYSTEM_LIBRARY}
)
