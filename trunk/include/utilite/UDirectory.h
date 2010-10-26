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

#ifndef UDIRECTORY_H
#define UDIRECTORY_H

#include "utilite/UtiLiteExp.h" // DLL export/import defines

#include <string>
#include <vector>
#include <list>

class UTILITE_EXP UDirectory
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

public:
	UDirectory(const std::string & path, const std::string & extensions = "");
	~UDirectory();

	void update();
	bool isValid();
	std::string getNextFileName();
	void rewind();

private:
	std::string path_;
	std::vector<std::string> extensions_;
	std::list<std::string> fileNames_;
	std::list<std::string>::iterator iFileName_;
};

#endif /* UDIRECTORY_H */
