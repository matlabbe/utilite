/*
Copyright (c) 2008-2014, Mathieu Labbe
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef UDIR_H
#define UDIR_H

#include "utilite/UtiLiteExp.h" // DLL export/import defines

#include <string>
#include <vector>
#include <list>

/**
 * Class UDir.
 *
 * This class can be used to get file names in a directory.
 */
class UTILITE_EXP UDir
{
public:
	/**
	 * Check if a directory exists.
	 * @param dirPath the directory path
	 * @return true if the directory exists
	 */
	static bool exists(const std::string & dirPath);

	/**
	 * Get the directory path of a file path.
	 * @param filePath the file path
	 * @return the directory path of the file
	 */
	static std::string getDir(const std::string & filePath);

	/**
	 * Get the current directory.
	 * @param trailingSeparator If true, a '/' is added to the path.
	 * @return the current directory
	 */
	static std::string currentDir(bool trailingSeparator = false);

	/**
	 * Make a directory.
	 * @param dirPath the directory path
	 * @return true on success, false otherwise.
	 */
	static bool makeDir(const std::string & dirPath);

	/**
	 * Remove a directory.
	 * @param dirPath the directory path
	 * @return true on success, false otherwise.
	 */
	static bool removeDir(const std::string & dirPath);

	/**
	 * Return the "home" directory.
	 * @return the directory path.
	 */
	static std::string homeDir();

	/**
	 * Return \ (Win32) or / (Unix) depending of the platform.
	 */
	static std::string separator();

public:
	/**
	 * Create a UDir object with path initialized to an existing "path" and with filter "extensions".
	 * @param path the path to an existing directory
	 * @param extensions filter to get only file names with the extensions specified, format is a
	 * list of extensions separated by a space: "jpg bmp" get only file names finishing by jpg or bmp.
	 */
	UDir(const std::string & path = "", const std::string & extensions = "");
	UDir(const UDir & dir);
	UDir & operator=(const UDir & dir);
	~UDir();

	/**
	 * Set path of the directory.
	 * @param path the new directory path.
	 */
	void setPath(const std::string & path, const std::string & extensions = "");

	/**
	 * Update indexed file names (if the directory changed).
	 */
	void update();

	/**
	 * Check is the directory exists.
	 * @return if directory exists.
	 */
	bool isValid();

	/**
	 * Get the next file name.
	 * @return the next file name
	 */
	std::string getNextFileName();

	/**
	 * Get all file names.
	 * @see UDir()
	 * @return all the file names in directory matching the set extensions.
	 */
	const std::list<std::string> & getFileNames() const {return fileNames_;}

	/**
	 * Return the pointer of file names to beginning.
	 */
	void rewind();

private:
	std::string path_;
	std::vector<std::string> extensions_;
	std::list<std::string> fileNames_;
	std::list<std::string>::iterator iFileName_;
};

#endif /* UDIR_H */
