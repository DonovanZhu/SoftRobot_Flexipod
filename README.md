# SoftRobot_Flexipod
SoftRobot research in Creative Machines Lab
## msgpack-c-cpp library install
When you use msgpack on C++, you can just add msgpack-c/include to your include path:

    g++ -I msgpack-c/include your_source_file.cpp

You will need:

 - `gcc >= 4.1.0`
 - `cmake >= 3.0.0`

C++03:

    $ git clone https://github.com/msgpack/msgpack-c.git
    $ cd msgpack-c
    $ git checkout cpp_master
    $ cmake .
    $ make
    $ sudo make install

If you want to setup C++11 or C++17 version of msgpack instead,
execute the following commands:

    $ git clone https://github.com/msgpack/msgpack-c.git
    $ cd msgpack-c
    $ git checkout cpp_master
    $ cmake -DMSGPACK_CXX[11|17]=ON .
    $ sudo make install

`MSGPACK_CXX[11|17]` flags are not affected to installing files. Just switching test cases. All files are installed in every settings.

### Known Issues when Cmake library
Could NOT find GTest:

    sudo apt-get install libgtest-dev
    
Could not find Doxygen (missing: DOXYGEN_EXECUTABLE):

    sudo apt-get install doxygen
    
CMake is not able to find BOOST libraries:

    sudo apt-get install cmake libblkid-dev e2fslibs-dev libboost-all-dev libaudit-dev
    
When using g++ -I msgpack-c/include your_source_file.cpp, fatal error: msgpack.hpp: No such file or directory:

    sudo apt install libmsgpack-dev

