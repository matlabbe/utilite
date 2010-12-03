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

#ifndef ULOGGER_H
#define ULOGGER_H

#include "utilite/UtiLiteExp.h" // DLL export/import defines

#include "utilite/UMutex.h"
#include "utilite/UDestroyer.h"

#include <stdio.h>
#include <time.h>
#include <string>
#include <vector>

#include <stdarg.h>

/**
 * Convenient macros for logging...
 */
#define ULOGGER_LOG(level, ...) ULogger::write(level, __FILE__, __LINE__, __FUNCTION__, __VA_ARGS__)

#define ULOGGER_DEBUG(...)   ULOGGER_LOG(ULogger::kDebug,   __VA_ARGS__)
#define ULOGGER_INFO(...)    ULOGGER_LOG(ULogger::kInfo,    __VA_ARGS__)
#define ULOGGER_WARN(...) 	 ULOGGER_LOG(ULogger::kWarning, __VA_ARGS__)
#define ULOGGER_ERROR(...)   ULOGGER_LOG(ULogger::kError,   __VA_ARGS__)
#define ULOGGER_FATAL(...)   ULOGGER_LOG(ULogger::kFatal,   __VA_ARGS__)

#define UDEBUG(...)   ULOGGER_DEBUG(__VA_ARGS__)
#define UINFO(...)    ULOGGER_INFO(__VA_ARGS__)
#define UWARN(...) 	  ULOGGER_WARN(__VA_ARGS__)
#define UERROR(...)   ULOGGER_ERROR(__VA_ARGS__)
#define UFATAL(...)   ULOGGER_FATAL(__VA_ARGS__)

/**
 * This class is used to log messages with time on a console, in a file 
 * or with a event. At the start of the application, call 
 * Logger::setType() with the type of the logger you want. To use it, 
 * simply call Logger::write("A message"). It can be used like a printf.
 * Example of output: "[DEBUG] (2008-7-13 12:23:44) A message".
 * Type of the output can be changed at the run-time.
 * Logger Types: TypeInvalid, TypeConsole, TypeFile, TypeMsgEvent
 *
 * @see setType()
 * @see write()
 *
 * TODO : Make it ThreadSafe when the type is changed.
 */
class UTILITE_EXP ULogger
{

public:
    /**
     * The default log file name ("./UtilLog.txt").
     */
    static const std::string kDefaultLogFileName;

    /**
     * Loggers available.
     */
    enum Type{kTypeNoLog, kTypeConsole, kTypeFile};

    /**
     * Logger levels
     */
    enum Level{kDebug, kInfo, kWarning, kError, kFatal};

    /**
     * Set the type of the logger.
     * @param type the type of the logger.
     * @param fileName file name used with a file logger type.
     * @param append if true, the file isn't overwritten, default true.
     *
     * TODO : Can it be useful to have 2 or more types at the same time ? Print
     *         in console and file at the same time.
     */
    static void setType(Type type, const std::string &fileName = kDefaultLogFileName, bool append = true);

    // Setters
    static void setPrintTime(bool printTime) {printTime_ = printTime;}
    static void setPrintLevel(bool printLevel) {printLevel_ = printLevel;}
    static void setPrintEndline(bool printEndline) {printEndline_ = printEndline;}
    static void setPrintWhere(bool printWhere) {printWhere_ = printWhere;}
    static void setPrintWhereFullPath(bool printWhereFullPath) {printWhereFullPath_ = printWhereFullPath;}

    /**
     * Set logger level
     */
    static void setLevel(ULogger::Level level) {level_ = level;}

    /**
     * reset default parameters of the Logger
     */
    static void reset();

    /**
     * Write a message on the output with the format :
     * "2008-7-13 12:23:44 - A message". It automaticaly 
     * gets the static instance of the Logger and calls 
     * his private function _write(). It can be used like 
     * a printf.
     * @see _write()
     * @param msg the message to write.
     * @param ... the variable arguments
     * @deprecated use ULOGGER_DEBUG(), ULOGGER_INFO(), ULOGGER_WARNING() or UULOGGER_ERROR()
     */
    static void write(const char* msg, ...);

    /**
     * Write a message on the output with the format :
     * "[DEBUG] (2008-7-13 12:23:44) A message". It automaticaly
     * gets the static instance of the Logger and calls his private
     * function _write(). It can be used like 
     * a printf.
     * @see _write()
     * @see _levels
     * @param level the log level of this message
     * @param msg the message to write
     * @param ... the variable arguments
     */
    static void write(ULogger::Level level,
    		const char * file,
    		int line,
    		const char *function,
    		const char* msg,
    		...);

    /**
     * Get the time in the format "2008-7-13 12:23:44".
     * @param timeStr string were the time will be copied.
     * @return the number of characters written, or �1 if an error occurred.
     */
    static int getTime(std::string &timeStr);

protected:
    /**
     * This method is used to have a reference on the 
     * Logger. When no Logger exists, one is 
     * created. There is only one instance in the application.
     * See the Singleton pattern for further explanation.
     *
     * @return the reference on the Logger
     */
    static ULogger* getInstance();

    /**
     * Called only once in getInstance(). It can't be instanciated 
     * by the user.
     *
     * @see getInstance()
     */
    ULogger() {}

    /**
     * Only called by a Destroyer.
     * @see Destroyer
     */
    virtual ~ULogger();

    /**
     * A Destroyer is used to remove a dynamicaly created 
     * Singleton. It is friend here to have access to the 
     * destructor.
     *
     * @see Destroyer
     */
    friend class UDestroyer<ULogger>;
    
    /**
     * The log file name.
     */
    static std::string _logFileName;

    /**
     * Default true, it doesn't overwrite the file.
     */
    static bool _append;
    
private:
    /**
     * Create an instance according to type. See the Abstract factory 
     * pattern for further explanation.
     * @see type_
     * @return the reference on the new logger
     */
    static ULogger* createInstance();

    /**
     * Write a message on the output with the format :
     * "A message". Inherited class
     * must override this method to output the message. It 
     * does nothing by default.
     * @param msg the message to write.
     * @param arg the variable arguments
     */
    virtual void _write(const char* msg, va_list arg) {} // Do nothing by default

private:
    /**
     * The Logger instance pointer.
     */
    static ULogger* instance_;

    /**
     * The Logger's destroyer
     */
    static UDestroyer<ULogger> destroyer_;

    /**
     * If the logger prints the time for each message. 
     * Default is true.
     */
    static bool printTime_;

    /**
     * If the logger prints the level for each message. 
     * Default is true.
     */
    static bool printLevel_;

    /**
     * If the logger prints the end line for each message. 
     * Default is true.
     */
    static bool printEndline_;

    /**
	 * If the logger prints where the message is logged (fileName::function():line).
	 * Default is true.
	 */
    static bool printWhere_;

    /**
	 * If the logger prints the full path of the source file
	 * where the message is written. Only works when
	 * "printWhere_" is true.
	 * Default is false.
	 */
    static bool printWhereFullPath_;

    /**
	 * If the logger limit the size of the "where" path to
	 * characters. If the path is over 8 characters, a "~"
	 * is added. Only works when "printWhereFullPath_" is false.
	 * Default is false.
	 */
    static bool limitWhereLength_;

    /**
     * The type of the logger.
     */
    static Type type_;

    /**
	 * The severity of the log.
	 */
    static Level level_;

    static const char * levelName_[5];

    /**
     * Mutex used when writing.
     */
    static UMutex writeMutex_;

    /**
     * Mutex used when a logger is created.
     */
    static UMutex instanceMutex_;
};

#endif // ULOGGER_H
