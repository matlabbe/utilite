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
#include "UConsoleLogger.h"
#include "UFileLogger.h"
#include "UConversion.h"
#include "UFile.h"
#include "UStl.h"
#include <string.h>

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
bool ULogger::_printTime = true;
bool ULogger::_printLevel = true;
bool ULogger::_printEndline = true;
bool ULogger::_printWhere = true;
bool ULogger::_printWhereFullPath = false;
bool ULogger::_limitWhereLength = false;
ULogger::Level ULogger::_level = kInfo; // By default, we show all info msgs + upper level (Warning, Error)
const char * ULogger::_levelName[4] = {"DEBUG", " INFO", " WARN", "ERROR"};
ULogger* ULogger::_instance = 0;
UDestroyer<ULogger> ULogger::_destroyer;
ULogger::Type ULogger::_type = ULogger::kTypeNoLog; // Default nothing
UMutex ULogger::_writeMutex;
const std::string ULogger::kDefaultLogFileName = "./UtilLog.txt";
std::string ULogger::_logFileName;
UMutex ULogger::_instanceMutex;

void ULogger::setType(Type type, const std::string &fileName, bool append)
{
    _instanceMutex.Lock();
    {
		// instance not yet created
		if(!_instance)
		{
			_type = type;
			_logFileName = fileName;
			_append = append;
			_instance = createInstance();
		}
		// type changed
		else if(_type != type || (_type == kTypeFile && _logFileName.compare(fileName)!=0))
		{
			_destroyer.setDoomed(0);
			delete _instance;
			_instance = 0;
			_type = type;
			_logFileName = fileName;
			_append = append;
			_instance = createInstance();
		}
    }
    _instanceMutex.Unlock();
}

void ULogger::reset()
{
	ULogger::setType(ULogger::kTypeNoLog);
	_append = true;
	_printTime = true;
	_printLevel = true;
	_printEndline = true;
	_printWhere = true;
	_printWhereFullPath = false;
	_limitWhereLength = false;
	_level = kInfo; // By default, we show all info msgs + upper level (Warning, Error)
	_logFileName = ULogger::kDefaultLogFileName;
}

void ULogger::write(const char* msg, ...)
{
	_instanceMutex.Lock();
	if(!_instance)
	{
		_instanceMutex.Unlock();
		return;
	}
	_instanceMutex.Unlock();

    std::string endline = "";
    if(_printEndline) {
        endline = "\r\n";
    }

    std::string time = "";
    if(_printTime)
    {
        getTime(time);
        time.append(" - ");
    }


    _writeMutex.Lock();
    if(_printTime) ULogger::getInstance()->_write(time.c_str(), 0);
    va_list args;
    va_start(args, msg);
    ULogger::getInstance()->_write(msg, args);
    va_end(args);
    if(_printEndline) ULogger::getInstance()->_write(endline.c_str(), 0);
    _writeMutex.Unlock();

} 

void ULogger::write(ULogger::Level level,
		const char * file,
		int line,
		const char * function,
		const char* msg,
		...)
{
	_instanceMutex.Lock();
	if(!_instance ||
	   (strlen(msg) == 0 && !_printWhere))
	{
		_instanceMutex.Unlock();
		// No need to show an empty message if we don't print where.
		return;
	}
	_instanceMutex.Unlock();

    if(level >= _level)
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
		if(_printEndline) {
			endline = "\r\n";
		}

		std::string time = "";
		if(_printTime)
		{
			time.append("(");
			getTime(time);
			time.append(") ");
		}

		std::string levelStr = "";
		if(_printLevel)
		{
			const int bufSize = 30;
			char buf[bufSize] = {0};

#ifdef _MSC_VER
			sprintf_s(buf, bufSize, "[%s]", _levelName[level]);
#else
			snprintf(buf, bufSize, "[%s]", _levelName[level]);
#endif
			levelStr = buf;
			levelStr.append(" ");
		}

		std::string whereStr = "";
		if(_printWhere)
		{
			whereStr.append("");
			//File
			if(_printWhereFullPath)
			{
				whereStr.append(file);
			}
			else
			{
				std::string fileName = UFile::getName(file);
				if(_limitWhereLength && fileName.size() > 8)
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
			if(!_printWhereFullPath && _limitWhereLength && funcStr.size() > 8)
			{
				funcStr.erase(8);
				funcStr.append("~");
			}
			funcStr.append("()");
			whereStr.append(funcStr);

			whereStr.append(" ");
		}

		_writeMutex.Lock();
		{
			va_list args;
			va_start(args, msg);
#ifdef WIN32
			HANDLE H = GetStdHandle(STD_OUTPUT_HANDLE);
#endif
			if(_type == ULogger::kTypeConsole)
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
			if(_type == ULogger::kTypeConsole)
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
		_writeMutex.Unlock();

    }
}

int ULogger::getTime(std::string &timeStr)
{
    if(!_printTime) {
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
    _instanceMutex.Lock();
    {
        if(!_instance)
        {
            _instance = createInstance();
        }
    }
    _instanceMutex.Unlock();
    return _instance;
}

ULogger* ULogger::createInstance()
{
    ULogger* instance = 0;
    if(_type == ULogger::kTypeConsole)
    {
        instance = new UConsoleLogger();
    }
    else if(_type == ULogger::kTypeFile)
    {
        instance = new UFileLogger(_logFileName, _append);
    }
    _destroyer.setDoomed(instance);
    return instance;
}

ULogger::~ULogger() 
{
    _instance = 0;
    //printf("Logger is destroyed...\n\r");
}
