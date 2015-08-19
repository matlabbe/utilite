# News #
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
<ul><li><b>New</b> audio library to capture frames from mic or a file (<a href='http://utilite.googlecode.com/svn/trunk/doc/html/class_u_audio_recorder.html'>UAudioRecorder</a> and <a href='http://utilite.googlecode.com/svn/trunk/doc/html/class_u_audio_player.html'>UAudioPlayer</a>, wav and mp3 supported),<br>
for fast algorithm prototyping and monitoring.</li></ul>

<h1>Installation</h1>
<ul><li>Build it from source : see <a href='http://code.google.com/p/utilite/source/checkout'>svn</a> and <a href='http://code.google.com/p/utilite/source/browse/trunk/utilite/README.txt'>README</a>.</li></ul>

<h2>Linux</h2>
<ul><li>FULL install (Qt widgets and audio library)<br>
<ul><li>Install <a href='http://www.fmod.org/download'>Fmodex</a> libraries (on linux, includes should be installed in /usr/local/include/fmodex and libraries should be in /usr/local/lib)<br>
<pre><code> $ sudo apt-get install libqt4-dev libmp3lame-dev libfftw3-dev libopencv-dev<br>
 $ svn checkout http://utilite.googlecode.com/svn/trunk/utilite utilite<br>
 $ cd utilite/build<br>
 $ cmake -DBUILD_AUDIO=ON -DBUILD_QT=ON -DBUILD_OPENCV=ON ..<br>
 $ make<br>
 $ make install<br>
</code></pre></li></ul></li></ul>

<ul><li>Minimum install<br>
<pre><code> $ svn checkout http://utilite.googlecode.com/svn/trunk/utilite utilite<br>
 $ cd utilite/build<br>
 $ cmake ..<br>
 $ make<br>
 $ make install<br>
</code></pre></li></ul>

<ul><li>To build tests, add "-DBUILD_TESTS=ON". To not build examples, add "-DBUILD_EXAMPLES=OFF".<br>
<h2>Windows</h2>
</li><li>Binaries <b>recommended</b>, see Downloads on the side of this page.<br>
</li><li>Source: FULL install (Qt widgets and audio library)<br>
<ol><li>Install <a href='http://www.mingw.org/wiki/Getting_Started'>MinGW</a>
</li><li>Install <a href='http://qt.nokia.com/downloads/downloads#qt-lib'>Qt4 MinGW libraries</a>
</li><li>Install <a href='http://opencv.org'>OpenCV2.4+</a>
</li><li>Checkout the UtiLite svn trunk (<a href='http://utilite.googlecode.com/svn/trunk/'>http://utilite.googlecode.com/svn/trunk/</a>)<br>
</li><li>Go to <a href='https://code.google.com/p/utilite/downloads/list'>download section</a>, and download all 3rdParty libraries. Extract them directly at the root of UtiLite directory.<br>
</li><li>Then, in utilite/build directory:<br>
<pre><code>$ cmake -G"MinGW Makefiles" -DBUILD_AUDIO=ON -DBUILD_QT=ON -DBUILD_OPENCV=ON ..<br>
$ mingw32-make<br>
$ mingw32-make install<br>
</code></pre></li></ol></li></ul>

<h2>ROS</h2>
<ol><li>First, you need to install the UtiLite standalone libraries. Follow Linux instructions <a href='https://code.google.com/p/utilite/#Linux'>above</a>.<br>
</li><li>Now install the UtiLite ros-pkg in your src folder of your Catkin workspace.<br>
<pre><code>$ cd ~/&lt;YOUR_CATKIN_WORKSPACE&gt;<br>
$ svn checkout http://utilite.googlecode.com/svn/trunk/ros-pkg src/utilite<br>
$ catkin_make<br>
</code></pre></li></ol>

See <a href='UtiLite_ROS.md'>UtiLite_ROS</a> page for information about using launch files (input/output of the nodes).<br>
<br>
<h1>Projects that use UtiLite</h1>
<ul><li><a href='http://rtabmap.googlecode.com'>RTAB-Map</a></li></ul>

<h1>Small example</h1>

<b>Create a custom event, called SpecialEvent :</b>
<pre><code>class SpecialEvent : public UEvent {<br>
public:<br>
   SpecialEvent(int code) : UEvent(code) {}<br>
   ~SpecialEvent() {}<br>
<br>
   virtual std::string getClassName() const {<br>
	   return "SpecialEvent";<br>
   }<br>
};<br>
</code></pre>

<b>Create an events handler :</b>
<pre><code>class EventsPrinter : public UEventsHandler {<br>
public:<br>
   EventsPrinter() {}<br>
   ~EventsPrinter() {}<br>
<br>
protected:<br>
   virtual void handleEvent(UEvent * e) {<br>
      if(e-&gt;getClassName().compare("SpecialEvent") == 0) {<br>
         UINFO("SpecialEvent \"%d\" received!", e-&gt;getCode());<br>
      }<br>
   }<br>
};<br>
</code></pre>

<b>The main :</b>
<pre><code>int main(int argc, char * argv[])<br>
{<br>
   ULogger::setType(ULogger::kTypeConsole);<br>
   ULogger::setLevel(ULogger::kInfo);<br>
   <br>
   UDEBUG("This message won't be logged because the "<br>
                 "severity level of the logger is set to kInfo.");<br>
   UINFO("This message is logged.");<br>
<br>
   EventsPrinter p;<br>
   UEventsManager::addHandler(&amp;p);<br>
<br>
   UEventsManager::post(new SpecialEvent(1));<br>
   UEventsManager::post(new SpecialEvent(2));<br>
   UEventsManager::post(new SpecialEvent(5));<br>
   UEventsManager::post(new SpecialEvent(7));<br>
   UEventsManager::post(new SpecialEvent(11));<br>
<br>
   uSleep(10);<br>
   UEventsManager::removeHandler(&amp;p);<br>
   return 0;<br>
}<br>
</code></pre>

<b>Output :</b>
<pre><code>[ INFO] (2010-09-25 18:08:20) eventsExample.cpp:53::main() This message is logged.<br>
[ INFO] (2010-09-25 18:08:20) eventsExample.cpp:32::handleEvent() SpecialEvent "1" received!<br>
[ INFO] (2010-09-25 18:08:20) eventsExample.cpp:32::handleEvent() SpecialEvent "2" received!<br>
[ INFO] (2010-09-25 18:08:20) eventsExample.cpp:32::handleEvent() SpecialEvent "5" received!<br>
[ INFO] (2010-09-25 18:08:20) eventsExample.cpp:32::handleEvent() SpecialEvent "7" received!<br>
[ INFO] (2010-09-25 18:08:20) eventsExample.cpp:32::handleEvent() SpecialEvent "11" received!<br>
</code></pre>

More examples <a href='Examples.md'>here</a>.