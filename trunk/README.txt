****************************
INSTALL
****************************
[Unix]
 > cd build
 > cmake ..
 > make
 > sudo make install

[Win32]
 > cd build
 > cmake -G "MinGW Makefiles" ..
 > mingw32-make
 > mingw32-make install
 
Note : If you do not want to install and just copying the output library, the output destination is in build/bin and build/lib of this project.

****************************
Developer (Using Eclipse)
****************************
# requirements
    -See previous requirements...
    -CppUnit for tests
	-Eclipse with CDT (c++ plugin)

# compilation
	1- Eclipse -> Import existing project -> select this directory
	2- Eclipse -> Make targets -> Avpd -> execute CMake-Unix-Release (CMake-MinGW-Release on Windows)
	3- Eclipse -> Project -> Build All

-The AutoCompletion should already works... (Project->Properties->C/C++Build->Discovery options)

**************
Notes
**************
Memory leaks check (Linux):
 >valgrind --tool=memcheck --leak-check=yes ./testutilite
  There are some memory errors with vfprintf() or write(), but the more important is at the end of the report in the summary "LEAK SUMMARY".
