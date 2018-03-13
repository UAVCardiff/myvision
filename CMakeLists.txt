project(myvision)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED visp_core visp_detection visp_blob visp_io visp_gui visp_sensor)

set(myvision_cpp
  camera_calibration.cpp
  livecam-apriltag-detector-working.cpp
)


foreach(cpp ${myvision_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "myvision")
  endif()
endforeach()

# copy the data
list(APPEND data2copy "${CMAKE_CURRENT_SOURCE_DIR}/default-chessboard.cfg" )
list(APPEND data2copy "${CMAKE_CURRENT_SOURCE_DIR}/chessboard-01.png" )
list(APPEND data2copy "${CMAKE_CURRENT_SOURCE_DIR}/chessboard-02.png" )
list(APPEND data2copy "${CMAKE_CURRENT_SOURCE_DIR}/chessboard-03.png" )
list(APPEND data2copy "${CMAKE_CURRENT_SOURCE_DIR}/chessboard-04.png" )
list(APPEND data2copy "${CMAKE_CURRENT_SOURCE_DIR}/chessboard-05.png" )
list(APPEND data2copy "${CMAKE_CURRENT_SOURCE_DIR}/chessboard-06.png" )

# Copy the data files to the same location than the target
foreach(data ${data2copy})
  visp_copy_data(camera_calibration.cpp ${data})
endforeach()
