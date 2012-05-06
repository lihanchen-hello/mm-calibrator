cmake_minimum_required(VERSION 2.8)

PROJECT( ./bin/mm_calibrator )

FIND_PACKAGE( OpenCV REQUIRED )

IF(WIN32)
SET(_boost_LIBRARIES_SEARCH_DIRS
     C:/boost/lib
     C:/boost
	)

	FIND_PACKAGE( Boost REQUIRED )

ELSE()
	FIND_PACKAGE(Boost COMPONENTS system filesystem REQUIRED)
ENDIF()

ADD_EXECUTABLE( ${PROJECT_NAME} 
	./src/mm_calibrator.cpp 		./src/mm_calibrator.hpp
	#cv_utils.cpp		cv_utils.hpp
	./src/general_resources.h
	./src/improc.cpp		./src/improc.h
	./src/tools.cpp		./src/tools.h
	./src/calibration.cpp	./src/calibration.hpp
	./src/intrinsics.cpp		./src/intrinsics.hpp
	./src/extrinsics.cpp		./src/extrinsics.hpp

	./src/XGetopt.cpp		./src/XGetopt.h
)

TARGET_LINK_LIBRARIES( 
	${PROJECT_NAME} 
	${OpenCV_LIBS} 
	${Boost_FILESYSTEM_LIBRARY}
	${Boost_SYSTEM_LIBRARY}
)
