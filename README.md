# utilite

## News 
June 2012
  * Version 0.2.14
  * Qt widgets and audio library added, see below **New**.
February 2012
  * Version 0.2.13
  * Added [API documentation](http://utilite.googlecode.com/svn/trunk/doc/html/index.html).
# Overview #
UtilLite is a lite c++ library that includes cross-platform (Windows, Linux, Mac) useful utilities like :
  * threads and safe inter-thread communication (events-based),
  * logger,
  * timer,
  * **New** Qt widgets [UPlot](http://utilite.googlecode.com/svn/trunk/doc/html/class_u_plot.html) and [USpectrogram](http://utilite.googlecode.com/svn/trunk/doc/html/class_u_spectrogram.html)
  * <img src='http://utilite.googlecode.com/svn/trunk/doc/image/UPlot.gif' width='300'> <img src='http://utilite.googlecode.com/svn/trunk/doc/image/USpectrogram.png' width='300'>
<b>New</b> audio library to capture frames from mic or a file (<a href='http://utilite.googlecode.com/svn/trunk/doc/html/class_u_audio_recorder.html'>UAudioRecorder</a> and <a href='http://utilite.googlecode.com/svn/trunk/doc/html/class_u_audio_player.html'>UAudioPlayer</a>, wav and mp3 supported),for fast algorithm prototyping and monitoring.

## Installation
Build it from source : see <a href='http://code.google.com/p/utilite/source/checkout'>svn</a> and <a href='http://code.google.com/p/utilite/source/browse/trunk/utilite/README.txt'>README</a>.

### Linux
#### FULL install (Qt widgets and audio library). 
Install <a href='http://www.fmod.org/download'>Fmodex</a> libraries (on linux, includes should be installed in `/usr/local/include/fmodex` and libraries should be in `/usr/local/lib`)

```bash
 $ sudo apt-get install libqt4-dev libmp3lame-dev libfftw3-dev libopencv-dev
 $ svn checkout http://utilite.googlecode.com/svn/trunk/utilite utilite
 $ cd utilite/build
 $ cmake -DBUILD_AUDIO=ON -DBUILD_QT=ON -DBUILD_OPENCV=ON ..
 $ make
 $ make install
```

#### Minimum install
```bash
 $ svn checkout http://utilite.googlecode.com/svn/trunk/utilite utilite
 $ cd utilite/build
 $ cmake ..
 $ make
 $ make install
```

To build tests, add "-DBUILD_TESTS=ON". To not build examples, add "-DBUILD_EXAMPLES=OFF".

### Windows
#### Binaries **recommended**, see Downloads on the side of this page.

#### Source 
FULL install (Qt widgets and audio library)

 1. Install <a href='http://www.mingw.org/wiki/Getting_Started'>MinGW</a>
 2. Install <a href='http://qt.nokia.com/downloads/downloads#qt-lib'>Qt4 MinGW libraries</a>
 3. Install <a href='http://opencv.org'>OpenCV2.4+</a>
 4. Checkout the UtiLite svn trunk (<a href='http://utilite.googlecode.com/svn/trunk/'>http://utilite.googlecode.com/svn/trunk/</a>)
 5. Go to <a href='https://code.google.com/p/utilite/downloads/list'>download section</a>, and download all 3rdParty libraries. Extract them directly at the root of UtiLite directory.
 6. Then, in utilite/build directory:

 ```bash
$ cmake -G"MinGW Makefiles" -DBUILD_AUDIO=ON -DBUILD_QT=ON -DBUILD_OPENCV=ON ..
$ mingw32-make
$ mingw32-make install
```


### ROS
First, you need to install the UtiLite standalone libraries. Follow Linux instructions <a href='https://code.google.com/p/utilite/#Linux'>above</a>.
Now install the UtiLite ros-pkg in your src folder of your Catkin workspace.

```bash
$ cd ~/&lt;YOUR_CATKIN_WORKSPACE&gt;
$ svn checkout http://utilite.googlecode.com/svn/trunk/ros-pkg src/utilite
$ catkin_make
```

See <a href='UtiLite_ROS.md'>UtiLite_ROS</a> page for information about using launch files (input/output of the nodes).

## Projects that use UtiLite
<a href='http://rtabmap.googlecode.com'>RTAB-Map</a>

## Small example

Create a custom event, called SpecialEvent :

```cpp
class SpecialEvent : public UEvent {
public:
   SpecialEvent(int code) : UEvent(code) {}
   ~SpecialEvent() {}

   virtual std::string getClassName() const {
	   return "SpecialEvent";
   }
};
```

Create an events handler :

```cpp
class EventsPrinter : public UEventsHandler {
public:
   EventsPrinter() {}
   ~EventsPrinter() {}

protected:
   virtual void handleEvent(UEvent * e) {
      if(e-&gt;getClassName().compare("SpecialEvent") == 0) {
         UINFO("SpecialEvent \"%d\" received!", e-&gt;getCode());
      }
   }
};
```

The main :

```cpp
int main(int argc, char * argv[])
{
   ULogger::setType(ULogger::kTypeConsole);
   ULogger::setLevel(ULogger::kInfo);
   
   UDEBUG("This message won't be logged because the "
                 "severity level of the logger is set to kInfo.");
   UINFO("This message is logged.");

   EventsPrinter p;
   UEventsManager::addHandler(&amp;p);

   UEventsManager::post(new SpecialEvent(1));
   UEventsManager::post(new SpecialEvent(2));
   UEventsManager::post(new SpecialEvent(5));
   UEventsManager::post(new SpecialEvent(7));
   UEventsManager::post(new SpecialEvent(11));

   uSleep(10);
   UEventsManager::removeHandler(&amp;p);
   return 0;
}
```

Output :

```bash
[ INFO] (2010-09-25 18:08:20) eventsExample.cpp:53::main() This message is logged.
[ INFO] (2010-09-25 18:08:20) eventsExample.cpp:32::handleEvent() SpecialEvent "1" received!
[ INFO] (2010-09-25 18:08:20) eventsExample.cpp:32::handleEvent() SpecialEvent "2" received!
[ INFO] (2010-09-25 18:08:20) eventsExample.cpp:32::handleEvent() SpecialEvent "5" received!
[ INFO] (2010-09-25 18:08:20) eventsExample.cpp:32::handleEvent() SpecialEvent "7" received!
[ INFO] (2010-09-25 18:08:20) eventsExample.cpp:32::handleEvent() SpecialEvent "11" received!
```

More examples <a href='Examples.md'>here</a>.
