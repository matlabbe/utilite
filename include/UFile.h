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

#ifndef FILE_H
#define FILE_H

#include "UtiLiteExp.h" // DLL export/import defines

#include "UDirectory.h"
#include <string>

class UTILITE_EXP UFile
{
public:
	/**
	 * Check if a file exists.
	 * @param filePath the file path
	 * @return true if the file exists, otherwise false.
	 */
	static bool exists(const std::string &filePath);

	/**
	 * Get the file length
	 * @param filePath the file path
	 * @return long the length of the file. Return -1 if the file doesn't exist.
	 */
	static long length(const std::string &filePath);

	/**
	 * Erase a file.
	 * @param filePath the file path
	 * @return 0 if success.
	 */
	static int erase(const std::string &filePath);

	/**
	 * Rename a file.
	 * @param oldFilePath the old file path
	 * @param newFilePath the new file path
	 * @return 0 if success.
	 */
	static int rename(const std::string &oldFilePath,
						  const std::string &newFilePath);

	/**
	 * Get the file name from a file path.
	 * @param filePath the file path
	 * @return the file name.
	 */
	static std::string getName(const std::string & filePath);

	static std::string getExtension(const std::string &filePath);

public:
	UFile(const std::string & path) : _path(path) {}
	~UFile() {}

	bool isValid() {return exists(_path);}
	bool exists() {return exists(_path);}
	long length() {return length(_path);}
	int rename(const std::string &newName)
	{
		std::string ext = this->getExtension();
		std::string newPath = UDirectory::getDir(_path) + std::string("/") + newName;
		if(ext.size())
		{
			newPath += std::string(".") + getExtension(_path);
		}
		int result = rename(_path, newPath);
		if(result == 0)
		{
			_path = newPath;
		}
		return result;
	}
	std::string getName() {return getName(_path);}
	std::string getExtension() {return getExtension(_path);}

private:
	std::string _path;
};

#endif
