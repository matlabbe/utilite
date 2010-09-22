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

#ifndef FILELOGGER_H
#define FILELOGGER_H

#include "UtiLiteExp.h" // DLL export/import defines

#include "ULogger.h"

#include <fstream> 
#include <string>

/**
 * This class is used to write logs in a file.
 */
class UTILITE_EXP UFileLogger : public ULogger
{
public:
    virtual ~UFileLogger()
    {
    	if(_fout)
    	{
    		fclose(_fout);
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
        _fileName = fileName;

        if(!append) {
			std::ofstream fileToClear(_fileName.c_str(), std::ios::out);
			fileToClear.clear();
			fileToClear.close();
		}

#ifdef _MSC_VER
        fopen_s(&_fout, _fileName.c_str(), "a");
#else
        _fout = fopen(_fileName.c_str(), "a");
#endif

        if(!_fout) {
            printf("FileLogger : Cannot open file : %s\n", _fileName.c_str()); // TODO send Event instead, or return error code
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
    	if(_fout)
    	{
			if(arg != 0)
			{
				vfprintf(_fout, msg, arg);
			}
			else
			{
				fprintf(_fout, "%s", msg);
			}
    	}
    }

private:
    std::string _fileName; ///< the file name
    FILE* _fout;
};

#endif
