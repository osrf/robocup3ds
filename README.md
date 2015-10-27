# Gazebo support for the RoboCup 3D simulator league #

## Install Gazebo

The RoboCup 3d plugin requires Gazebo 6 or higher. Follow the [Gazebo installation tutorial](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install) for installing Gazebo on your machine.

### Install RoboCup 3D Gazebo simulation

To install the robocup 3D gazebo package in Ubuntu please use:

```
#!sh
curl -ssL https://bitbucket.org/osrf/release-tools/raw/default/one-line-installations/robocup3ds.sh | sh
```
An alternative step by step installation method is detailed below.

### Alternative: build gazebo from source

If robocup 3D gazebo simulator was installed using the one-liner this instructions
are not required.

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

    >        cmake -DCMAKE_INSTALL_PREFIX=/usr ..


    > b. Debug mode: This will generate code with debug symbols. RoboCup3dS will
    run slower, but you'll be able to use GDB.

    >        cmake -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_BUILD_TYPE=Debug ..

1. The output from `cmake ..` may generate a number of errors and warnings
about missing packages. You must install the missing packages that have errors
and re-run `cmake`. Make sure all the build errors are resolved before
continuing (they should be there from the earlier step in which you installed
prerequisites). Warnings alert of optional packages that are missing.

1. Make note of your install path, which is output from `cmake` and should look something like:

          -- Install path: /usr

1. Build robocup3ds:

        make -j4

1. Install robocup3ds:

        sudo make install


#### Running tests

You can run the tests for the plugins from the same build directory by running:

        make test

#### Code checking

Code checking is done with cppchecker. Make sure you have it installed:

        sudo apt-get install cppcheck

Then to run the check:

1. First move to the robocup3ds directory:

        cd ~/robocup3ds

1. Then run the checker:

        . tools/code_check.sh

### Running the simulator

Run the simulator to check your installation. A world with a soccer field
should appear and there should be no errors printed in the terminal, so we know
the plugins are running.

```
gazebo-robocup3ds --verbose
```