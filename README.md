# Gazebo support for the RoboCup 3D simulator league #

### Build And Install RoboCup 3D Gazebo simulation.

1. Clone the repository into a directory in your home folder:

        cd ~; hg clone https://bitbucket.org/osrf/robocup3ds

1. Change directory in the robocup3ds repository:

        cd ~/robocup3ds

1. Create a build directory and go there:

        mkdir build
        cd build

1. Configure robocup3ds (choose either method `a` or `b` below):

    > a. Release mode: This will generate optimized code, but will not have
    debug symbols. Use this mode if you don't need to use GDB.

    >        cmake ../


    >> Note: You can use a custom install path to make it easier to switch
    between source and debian installs:

    >>        cmake -DCMAKE_INSTALL_PREFIX=/home/$USER/local ../

    > b. Debug mode: This will generate code with debug symbols. RoboCup3dS will
    run slower, but you'll be able to use GDB.

    >        cmake -DCMAKE_BUILD_TYPE=Debug ../

1. The output from `cmake ../` may generate a number of errors and warnings
about missing packages. You must install the missing packages that have errors
and re-run `cmake ../`. Make sure all the build errors are resolved before
continuing (they should be there from the earlier step in which you installed
prerequisites). Warnings alert of optional packages that are missing.

1. Make note of your install path, which is output from `cmake` and should look something like:

          -- Install path: /home/$USER/local

1. Build robocup3ds:

        make -j4

1. Install robocup3ds:

        sudo make install
