/*
*  utilite is a cross-platform library with
*  useful utilities for fast and small developing.
*  Copyright (C) 2010  Mathieu Labbe
*
*  utilite is free library: you can redistribute it and/or modify
*  it under the terms of the GNU Lesser General Public License as published by
*  the Free Software Foundation, either version 3 of the License, or
*  (at your option) any later version.
*
*  utilite is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU Lesser General Public License for more details.
*
*  You should have received a copy of the GNU Lesser General Public License
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "utilite/ULogger.h"
#include "utilite/UConversion.h"
#include "utilite/UFile.h"
#include "utilite/UStl.h"
#include "utilite/UEventsManager.h"
#include <fstream>
#include <string>
#include <string.h>

#ifndef WIN32
#include <sys/time.h>
#endif

#ifdef WIN32
#include <Windows.h>
#define COLOR_NORMAL FOREGROUND_BLUE | FOREGROUND_GREEN | FOREGROUND_RED
#define COLOR_RED FOREGROUND_RED | FOREGROUND_INTENSITY
#define COLOR_GREEN FOREGROUND_GREEN
#define COLOR_YELLOW FOREGROUND_GREEN | FOREGROUND_RED
#else
#define COLOR_NORMAL "\033[0m"
#define COLOR_RED "\033[31m"
#define COLOR_GREEN "\033[32m"
#define COLOR_YELLOW "\033[33m"
#endif

bool ULogger::append_ = true;
bool ULogger::printTime_ = true;
bool ULogger::printLevel_ = true;
bool ULogger::printEndline_ = true;
bool ULogger::printWhere_ = true;
bool ULogger::printWhereFullPath_ = false;
bool ULogger::limitWhereLength_ = false;
bool ULogger::buffered_ = false;
bool ULogger::exitingState_ = false;
ULogger::Level ULogger::level_ = kInfo; // By default, we show all info msgs + upper level (Warning, Error)
ULogger::Level ULogger::exitLevel_ = kFatal;
ULogger::Level ULogger::eventLevel_ = kFatal;
const char * ULogger::levelName_[5] = {"DEBUG", " INFO", " WARN", "ERROR", "FATAL"};
ULogger* ULogger::instance_ = 0;
UDestroyer<ULogger> ULogger::destroyer_;
ULogger::Type ULogger::type_ = ULogger::kTypeNoLog; // Default nothing
UMutex ULogger::loggerMutex_;
const std::string ULogger::kDefaultLogFileName = "./ULog.txt";
std::string ULogger::logFileName_;
std::string ULogger::bufferedMsgs_;

/**
 * This class is used to write logs in the console. This class cannot
 * be directly used, use ULogger::setType() to console type to print in
 * console and use macro UDEBUG(), UINFO()... to print messages.
 * @see ULogger
 */
class UConsoleLogger : public ULogger
{
public :
    virtual ~UConsoleLogger() {this->_flush();}

protected:
    /**
     * Only the Logger can create inherited
     * loggers according to the Abstract factory patterns.
     */
    friend class ULogger;

    UConsoleLogger() {}

private:
    virtual void _write(const char* msg, va_list arg)
    {
    	if(arg != 0)
		{
			vprintf(msg, arg);
		}
		else
		{
			printf("%s", msg);
		}
    }
};

/**
 * This class is used to write logs in a file. This class cannot
 * be directly used, use ULogger::setType() to file type to print in
 * a file and use macro UDEBUG(), UINFO()... to print messages.
 * @see ULogger
 */
class UFileLogger : public ULogger
{
public:
    virtual ~UFileLogger()
    {
    	this->_flush();
    	if(fout_)
    	{
    		fclose(fout_);
    	}
    }

protected:
    /**
     * Only the Logger can create inherited
     * loggers according to the Abstract factory patterns.
     */
    friend class ULogger;

    /**
     * The UFileLogger constructor.
     * @param fileName the file name
     * @param append if true append logs in the file,
     *        ortherwise it overrides the file.
     *
     */
    UFileLogger(const std::string &fileName, bool append)
    {
        fileName_ = fileName;

        if(!append) {
			std::ofstream fileToClear(fileName_.c_str(), std::ios::out);
			fileToClear.clear();
			fileToClear.close();
		}

#ifdef _MSC_VER
        fopen_s(&fout_, fileName_.c_str(), "a");
#else
        fout_ = fopen(fileName_.c_str(), "a");
#endif

        if(!fout_) {
            printf("FileLogger : Cannot open file : %s\n", fileName_.c_str()); // TODO send Event instead, or return error code
            return;
        }
    }

private:
    virtual void _write(const char* msg, va_list arg)
    {
    	if(fout_)
    	{
			if(arg != 0)
			{
				vfprintf(fout_, msg, arg);
			}
			else
			{
				fprintf(fout_, "%s", msg);
			}
    	}
    }

private:
    std::string fileName_; ///< the file name
    FILE* fout_;
    std::string bufferedMsgs_;
};

void ULogger::setType(Type type, const std::string &fileName, bool append)
{
	ULogger::flush();
    loggerMutex_.lock();
    {
		// instance not yet created
		if(!instance_)
		{
			type_ = type;
			logFileName_ = fileName;
			append_ = append;
			instance_ = createInstance();
		}
		// type changed
		else if(type_ != type || (type_ == kTypeFile && logFileName_.compare(fileName)!=0))
		{
			destroyer_.setDoomed(0);
			delete instance_;
			instance_ = 0;
			type_ = type;
			logFileName_ = fileName;
			append_ = append;
			instance_ = createInstance();
		}
    }
    loggerMutex_.unlock();
}

void ULogger::reset()
{
	ULogger::setType(ULogger::kTypeNoLog);
	append_ = true;
	printTime_ = true;
	printLevel_ = true;
	printEndline_ = true;
	printWhere_ = true;
	printWhereFullPath_ = false;
	limitWhereLength_ = false;
	level_ = kInfo; // By default, we show all info msgs + upper level (Warning, Error)
	logFileName_ = ULogger::kDefaultLogFileName;
}

void ULogger::setBuffered(bool buffered)
{
	if(!buffered)
	{
		ULogger::flush();
	}
	buffered_ = buffered;
}


void ULogger::flush()
{
	loggerMutex_.lock();
	if(!instance_ || bufferedMsgs_.size()==0)
	{
		loggerMutex_.unlock();
		return;
	}

	instance_->_flush();
	loggerMutex_.unlock();
}

void ULogger::_flush()
{
	ULogger::getInstance()->_write(bufferedMsgs_.c_str(), 0);
	bufferedMsgs_.clear();
}

void ULogger::write(const char* msg, ...)
{
	loggerMutex_.lock();
	if(!instance_)
	{
		loggerMutex_.unlock();
		return;
	}

    std::string endline = "";
    if(printEndline_) {
        endline = "\r\n";
    }

    std::string time = "";
    if(printTime_)
    {
        getTime(time);
        time.append(" - ");
    }


    if(printTime_)
    {
    	if(buffered_)
    	{
    		bufferedMsgs_.append(time.c_str());
    	}
    	else
    	{
    		ULogger::getInstance()->_write(time.c_str(), 0);
    	}
    }

    va_list args;
    va_start(args, msg);
    if(buffered_)
    {
    	bufferedMsgs_.append(uFormatv(msg, args));
    }
	else
	{
		ULogger::getInstance()->_write(msg, args);
	}
    va_end(args);
    if(printEndline_)
    {
    	if(buffered_)
    	{
    		bufferedMsgs_.append(endline.c_str());
    	}
    	else
    	{
    		ULogger::getInstance()->_write(endline.c_str(), 0);
    	}
    }
    loggerMutex_.unlock();

} 

void ULogger::write(ULogger::Level level,
		const char * file,
		int line,
		const char * function,
		const char* msg,
		...)
{
	if(exitingState_)
	{
		// Ignore messages after a fatal exit...
		return;
	}
	loggerMutex_.lock();
	if(type_ == kTypeNoLog && level < kFatal)
	{
		loggerMutex_.unlock();
		return;
	}
	if(strlen(msg) == 0 && !printWhere_ && level < exitLevel_)
	{
		loggerMutex_.unlock();
		// No need to show an empty message if we don't print where.
		return;
	}

    if(level >= level_)
    {
#ifdef WIN32
    	int color = 0;
#else
    	const char* color = NULL;
#endif
    	switch(level)
    	{
    	case kDebug:
    		color = COLOR_GREEN;
    		break;
    	case kInfo:
    		color = COLOR_NORMAL;
    		break;
    	case kWarning:
    		color = COLOR_YELLOW;
    		break;
    	case kError:
    	case kFatal:
    		color = COLOR_RED;
    		break;
    	default:
    		break;
    	}

		std::string endline = "";
		if(printEndline_) {
			endline = "\r\n";
		}

		std::string time = "";
		if(printTime_)
		{
			time.append("(");
			getTime(time);
			time.append(") ");
		}

		std::string levelStr = "";
		if(printLevel_)
		{
			const int bufSize = 30;
			char buf[bufSize] = {0};

#ifdef _MSC_VER
			sprintf_s(buf, bufSize, "[%s]", levelName_[level]);
#else
			snprintf(buf, bufSize, "[%s]", levelName_[level]);
#endif
			levelStr = buf;
			levelStr.append(" ");
		}

		std::string whereStr = "";
		if(printWhere_)
		{
			whereStr.append("");
			//File
			if(printWhereFullPath_)
			{
				whereStr.append(file);
			}
			else
			{
				std::string fileName = UFile::getName(file);
				if(limitWhereLength_ && fileName.size() > 8)
				{
					fileName.erase(8);
					fileName.append("~");
				}
				whereStr.append(fileName);
			}

			//Line
			whereStr.append(":");
			std::string lineStr = uNumber2Str(line);
			whereStr.append(lineStr);

			//Function
			whereStr.append("::");
			std::string funcStr = function;
			if(!printWhereFullPath_ && limitWhereLength_ && funcStr.size() > 8)
			{
				funcStr.erase(8);
				funcStr.append("~");
			}
			funcStr.append("()");
			whereStr.append(funcStr);

			whereStr.append(" ");
		}

		va_list args;

		if(type_ != kTypeNoLog)
		{
			va_start(args, msg);
#ifdef WIN32
			HANDLE H = GetStdHandle(STD_OUTPUT_HANDLE);
#endif
			if(type_ == ULogger::kTypeConsole)
			{
#ifdef WIN32
				SetConsoleTextAttribute(H,color);
#else
				if(buffered_)
				{
					bufferedMsgs_.append(color);
				}
				else
				{
					ULogger::getInstance()->_write(color, 0);
				}
#endif
			}

			if(buffered_)
			{
				bufferedMsgs_.append(levelStr.c_str());
				bufferedMsgs_.append(time.c_str());
				bufferedMsgs_.append(whereStr.c_str());
				bufferedMsgs_.append(uFormatv(msg, args));
			}
			else
			{
				ULogger::getInstance()->_write(levelStr.c_str(), 0);
				ULogger::getInstance()->_write(time.c_str(), 0);
				ULogger::getInstance()->_write(whereStr.c_str(), 0);
				ULogger::getInstance()->_write(msg, args);
			}
			if(type_ == ULogger::kTypeConsole)
			{
#ifdef WIN32
				SetConsoleTextAttribute(H,COLOR_NORMAL);
#else
				if(buffered_)
				{
					bufferedMsgs_.append(COLOR_NORMAL);
				}
				else
				{
					ULogger::getInstance()->_write(COLOR_NORMAL, 0);
				}
#endif
			}
			if(buffered_)
			{
				bufferedMsgs_.append(endline.c_str());
			}
			else
			{
				ULogger::getInstance()->_write(endline.c_str(), 0);
			}
			va_end (args);
		}

		if(level >= eventLevel_)
		{
			std::string fullMsg = uFormat("%s%s%s", levelStr.c_str(), time.c_str(), whereStr.c_str());
			va_start(args, msg);
			if(args != 0)
			{
				fullMsg.append(uFormatv(msg, args));
			}
			else
			{
				fullMsg.append(msg);
			}
			va_end(args);
			if(level >= exitLevel_)
			{
				// Send it synchronously, then receivers
				// can do something before the code (exiting) below is executed.
				exitingState_ = true;
				UEventsManager::post(new ULogEvent(fullMsg, kFatal), false);
			}
			else
			{
				UEventsManager::post(new ULogEvent(fullMsg, level));
			}
		}

		if(level >= exitLevel_)
		{
			printf("\n*******\n%s message occurred!\n", levelName_[level]);
			printf("  %s%s%s", levelStr.c_str(), time.c_str(), whereStr.c_str());
			va_start(args, msg);
			if(args != 0)
			{
				vprintf(msg, args);
			}
			else
			{
				printf("%s", msg);
			}
			va_end(args);
			printf("\n*******\n");
			destroyer_.setDoomed(0);
			delete instance_; // If a FileLogger is used, this will close the file.
			instance_ = 0;
			//========================================================================
			//                          EXIT APPLICATION
			exit(1);
			//========================================================================
		}
    }
    loggerMutex_.unlock();
}

int ULogger::getTime(std::string &timeStr)
{
    if(!printTime_) {
        return 0;
    }
    struct tm timeinfo;
    const int bufSize = 30;
    char buf[bufSize] = {0};

#if _MSC_VER
    time_t rawtime;
    time(&rawtime);
    localtime_s (&timeinfo, &rawtime );
    int result = sprintf_s(buf, bufSize, "%d-%s%d-%s%d %s%d:%s%d:%s%d",
        timeinfo.tm_year+1900,
        (timeinfo.tm_mon+1) < 10 ? "0":"", timeinfo.tm_mon+1,
        (timeinfo.tm_mday) < 10 ? "0":"", timeinfo.tm_mday,
        (timeinfo.tm_hour) < 10 ? "0":"", timeinfo.tm_hour,
        (timeinfo.tm_min) < 10 ? "0":"", timeinfo.tm_min,
        (timeinfo.tm_sec) < 10 ? "0":"", timeinfo.tm_sec);
#elif WIN32
    time_t rawtime;
    time(&rawtime);
    timeinfo = *localtime (&rawtime);
    int result = snprintf(buf, bufSize, "%d-%s%d-%s%d %s%d:%s%d:%s%d",
		timeinfo.tm_year+1900,
		(timeinfo.tm_mon+1) < 10 ? "0":"", timeinfo.tm_mon+1,
		(timeinfo.tm_mday) < 10 ? "0":"", timeinfo.tm_mday,
		(timeinfo.tm_hour) < 10 ? "0":"", timeinfo.tm_hour,
		(timeinfo.tm_min) < 10 ? "0":"", timeinfo.tm_min,
		(timeinfo.tm_sec) < 10 ? "0":"", timeinfo.tm_sec);
 #else
    struct timeval rawtime;
    gettimeofday(&rawtime, NULL);
    localtime_r (&rawtime.tv_sec, &timeinfo);
	int result = snprintf(buf, bufSize, "%d-%s%d-%s%d %s%d:%s%d:%s%d.%s%d",
		timeinfo.tm_year+1900,
		(timeinfo.tm_mon+1) < 10 ? "0":"", timeinfo.tm_mon+1,
		(timeinfo.tm_mday) < 10 ? "0":"", timeinfo.tm_mday,
		(timeinfo.tm_hour) < 10 ? "0":"", timeinfo.tm_hour,
		(timeinfo.tm_min) < 10 ? "0":"", timeinfo.tm_min,
		(timeinfo.tm_sec) < 10 ? "0":"", timeinfo.tm_sec,
	    (rawtime.tv_usec/1000) < 10 ? "00":(rawtime.tv_usec/1000) < 100?"0":"", int(rawtime.tv_usec/1000));
#endif
    if(result)
    {
        timeStr.append(buf);
    }
    return result;
}

ULogger* ULogger::getInstance()
{
	if(!instance_)
	{
		instance_ = createInstance();
	}
    return instance_;
}

ULogger* ULogger::createInstance()
{
    ULogger* instance = 0;
    if(type_ == ULogger::kTypeConsole)
    {
        instance = new UConsoleLogger();
    }
    else if(type_ == ULogger::kTypeFile)
    {
        instance = new UFileLogger(logFileName_, append_);
    }
    destroyer_.setDoomed(instance);
    return instance;
}

ULogger::~ULogger() 
{
    instance_ = 0;
    //printf("Logger is destroyed...\n\r");
}