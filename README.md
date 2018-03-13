# myvision
AprilTag detection and pose estimation. Camera calibration.
MICHAEL WELCOMES YOU

░░░░░░░░░░░░░░░░░░░░░░█████████
░░███████░░░░░░░░░░███▒▒▒▒▒▒▒▒███
░░█▒▒▒▒▒▒█░░░░░░░███▒▒▒▒▒▒▒▒▒▒▒▒▒███
░░░█▒▒▒▒▒▒█░░░░██▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒██
░░░░█▒▒▒▒▒█░░░██▒▒▒▒▒██▒▒▒▒▒▒██▒▒▒▒▒███
░░░░░█▒▒▒█░░░█▒▒▒▒▒▒████▒▒▒▒████▒▒▒▒▒▒██
░░░█████████████▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒██
░░░█▒▒▒▒▒▒▒▒▒▒▒▒█▒▒▒▒▒▒▒▒▒█▒▒▒▒▒▒▒▒▒▒▒██
░██▒▒▒▒▒▒▒▒▒▒▒▒▒█▒▒▒██▒▒▒▒▒▒▒▒▒▒██▒▒▒▒██
██▒▒▒███████████▒▒▒▒▒██▒▒▒▒▒▒▒▒██▒▒▒▒▒██
█▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒█▒▒▒▒▒▒████████▒▒▒▒▒▒▒██
██▒▒▒▒▒▒▒▒▒▒▒▒▒▒█▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒██
░█▒▒▒███████████▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒██
░██▒▒▒▒▒▒▒▒▒▒████▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒█
░░████████████░░░█████████████████
**************************************************************
Run and skip to Install Visp
    $ ./buildVisp.sh
***************************Visp*******************************
Install prerequisities

Prior to build and install ViSP from source, you may install GNU g++ compiler and CMake.

    gcc 4.4.x or later. This can be installed with:
    $ sudo apt-get install build-essential
    CMake 2.8.12.2 or higher that could be installed with:
    $ sudo apt-get install cmake-curses-gui
    OpenCV
    $ sudo apt-get install libopencv-dev
    lapack and eigen to benefit from optimized mathematical capabilities
    $ sudo apt-get install liblapack-dev libeigen3-dev
    libv4l to grab images from usb or analogic cameras
    $ sudo apt-get install libv4l-dev
    libxml2 to be able to configure the model-based trackers from xml files
    $ sudo apt-get install libxml2-dev
**************************************************************
Create a workspace

    First create a workspace in $HOME/visp-ws that will contain ViSP source, build and dataset.
    $ export VISP_WS=$HOME/visp-ws
    $ mkdir -p $VISP_WS
**************************************************************
Getting ViSP source code

    Get the cutting-edge ViSP from GitHub repository using the following command
    $ cd $VISP_WS
    $ git clone https://github.com/lagadic/visp.git
**************************************************************
Configuring ViSP from source

    In the worspace, create first a directory named visp-build that will contain all the build material; generated Makefiles, object files, output libraries and binaries.
    $ mkdir $VISP_WS/visp-build
    Enter the visp-build folder and configure the build:
    $ cd $VISP_WS/visp-build
    $ cmake ../visp
**************************************************************
Building ViSP libraries

    To build ViSP libraries proceed with:
    $ cd $VISP_WS/visp-build
    $ make -j4
**************************************************************
Install Visp **start here after running $ ./buildVisp**
    $ sudo make install
**************************************************************
Configure the project

    $ cd ~/<working_directory>
    $ git clone https://github.com/UAVCardiff/myvision.git
    Get to the package
    $ cd $myvision
    Proceed as with any other project using CMake:
    $ cmake .
**************************************************************
Generate the executable

    Just run:
    $ make
**************************************************************
Exectute

    ./livecam-apriltag-detector-working.cpp
**************************************************************
