/**
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

#include "ULogger.h"
#include "UConversion.h"
#include "UFile.h"
#include "UStl.h"
#include <fstream>
#include <string>

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

bool ULogger::_append = true;
bool ULogger::printTime_ = true;
bool ULogger::printLevel_ = true;
bool ULogger::printEndline_ = true;
bool ULogger::printWhere_ = true;
bool ULogger::printWhereFullPath_ = false;
bool ULogger::limitWhereLength_ = false;
ULogger::Level ULogger::level_ = kInfo; // By default, we show all info msgs + upper level (Warning, Error)
const char * ULogger::levelName_[4] = {"DEBUG", " INFO", " WARN", "ERROR"};
ULogger* ULogger::instance_ = 0;
UDestroyer<ULogger> ULogger::destroyer_;
ULogger::Type ULogger::type_ = ULogger::kTypeNoLog; // Default nothing
UMutex ULogger::writeMutex_;
const std::string ULogger::kDefaultLogFileName = "./ULog.txt";
std::string ULogger::_logFileName;
UMutex ULogger::instanceMutex_;


/**
 * This class is used to write logs in the console.
 */
class UTILITE_EXP UConsoleLogger : public ULogger
{
public :
    virtual ~UConsoleLogger() {}

protected:
    /**
     * Only the Logger can create inherited
     * loggers according to the Abstract factory patterns.
     */
    friend class ULogger;

    UConsoleLogger() {}

private:
    /**
     * Overrided to print in the console.
     * @param msg the message to write.
     * @param arg the variable arguments
     * @see Logger::_write
     */
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
 * This class is used to write logs in a file.
 */
class UTILITE_EXP UFileLogger : public ULogger
{
public:
    virtual ~UFileLogger()
    {
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
     *
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
    /**
     * Overrided to print in the file.
     * @param msg the message to write.
     * @param arg the variable arguments
     * @see Logger::_write
     */
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
};

void ULogger::setType(Type type, const std::string &fileName, bool append)
{
    instanceMutex_.lock();
    {
		// instance not yet created
		if(!instance_)
		{
			type_ = type;
			_logFileName = fileName;
			_append = append;
			instance_ = createInstance();
		}
		// type changed
		else if(type_ != type || (type_ == kTypeFile && _logFileName.compare(fileName)!=0))
		{
			destroyer_.setDoomed(0);
			delete instance_;
			instance_ = 0;
			type_ = type;
			_logFileName = fileName;
			_append = append;
			instance_ = createInstance();
		}
    }
    instanceMutex_.unlock();
}

void ULogger::reset()
{
	ULogger::setType(ULogger::kTypeNoLog);
	_append = true;
	printTime_ = true;
	printLevel_ = true;
	printEndline_ = true;
	printWhere_ = true;
	printWhereFullPath_ = false;
	limitWhereLength_ = false;
	level_ = kInfo; // By default, we show all info msgs + upper level (Warning, Error)
	_logFileName = ULogger::kDefaultLogFileName;
}

void ULogger::write(const char* msg, ...)
{
	instanceMutex_.lock();
	if(!instance_)
	{
		instanceMutex_.unlock();
		return;
	}
	instanceMutex_.unlock();

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


    writeMutex_.lock();
    if(printTime_) ULogger::getInstance()->_write(time.c_str(), 0);
    va_list args;
    va_start(args, msg);
    ULogger::getInstance()->_write(msg, args);
    va_end(args);
    if(printEndline_) ULogger::getInstance()->_write(endline.c_str(), 0);
    writeMutex_.unlock();

} 

void ULogger::write(ULogger::Level level,
		const char * file,
		int line,
		const char * function,
		const char* msg,
		...)
{
	instanceMutex_.lock();
	if(!instance_ ||
	   (strlen(msg) == 0 && !printWhere_))
	{
		instanceMutex_.unlock();
		// No need to show an empty message if we don't print where.
		return;
	}
	instanceMutex_.unlock();

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
			std::string lineStr = uNumber2str(line);
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

		writeMutex_.lock();
		{
			va_list args;
			va_start(args, msg);
#ifdef WIN32
			HANDLE H = GetStdHandle(STD_OUTPUT_HANDLE);
#endif
			if(type_ == ULogger::kTypeConsole)
			{
#ifdef WIN32
				SetConsoleTextAttribute(H,color);
#else
				ULogger::getInstance()->_write(color, 0);
#endif
			}
			ULogger::getInstance()->_write(levelStr.c_str(), 0);
			ULogger::getInstance()->_write(time.c_str(), 0);
			ULogger::getInstance()->_write(whereStr.c_str(), 0);
			ULogger::getInstance()->_write(msg, args);
			if(type_ == ULogger::kTypeConsole)
			{
#ifdef WIN32
				SetConsoleTextAttribute(H,COLOR_NORMAL);
#else
				ULogger::getInstance()->_write(COLOR_NORMAL, 0);
#endif
			}
			ULogger::getInstance()->_write(endline.c_str(), 0);
			va_end (args);
		}
		writeMutex_.unlock();

    }
}

int ULogger::getTime(std::string &timeStr)
{
    if(!printTime_) {
        return 0;
    }
    time_t rawtime;
    struct tm timeinfo;
    const int bufSize = 30;
    char buf[bufSize] = {0};

    time ( &rawtime );
#ifdef _MSC_VER
    localtime_s (&timeinfo, &rawtime );
    int result = sprintf_s(buf, bufSize, "%d-%s%d-%s%d %s%d:%s%d:%s%d",
        timeinfo.tm_year+1900,
        (timeinfo.tm_mon+1) < 10 ? "0":"", timeinfo.tm_mon+1,
        (timeinfo.tm_mday) < 10 ? "0":"", timeinfo.tm_mday,
        (timeinfo.tm_hour) < 10 ? "0":"", timeinfo.tm_hour,
        (timeinfo.tm_min) < 10 ? "0":"", timeinfo.tm_min,
        (timeinfo.tm_sec) < 10 ? "0":"", timeinfo.tm_sec);
#else
 #if WIN32 // MinGW
    timeinfo = *localtime (&rawtime );
 #else
    localtime_r (&rawtime, &timeinfo);
 #endif
	int result = snprintf(buf, bufSize, "%d-%s%d-%s%d %s%d:%s%d:%s%d",
		timeinfo.tm_year+1900,
		(timeinfo.tm_mon+1) < 10 ? "0":"", timeinfo.tm_mon+1,
		(timeinfo.tm_mday) < 10 ? "0":"", timeinfo.tm_mday,
		(timeinfo.tm_hour) < 10 ? "0":"", timeinfo.tm_hour,
		(timeinfo.tm_min) < 10 ? "0":"", timeinfo.tm_min,
		(timeinfo.tm_sec) < 10 ? "0":"", timeinfo.tm_sec);
#endif
    if(result)
    {
        timeStr.append(buf);
    }
    return result;
}

ULogger* ULogger::getInstance()
{
    instanceMutex_.lock();
    {
        if(!instance_)
        {
            instance_ = createInstance();
        }
    }
    instanceMutex_.unlock();
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
        instance = new UFileLogger(_logFileName, _append);
    }
    destroyer_.setDoomed(instance);
    return instance;
}

ULogger::~ULogger() 
{
    instance_ = 0;
    //printf("Logger is destroyed...\n\r");
}
