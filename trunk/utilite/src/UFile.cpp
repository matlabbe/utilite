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

#include "utilite/UFile.h"

#include <fstream>
#include "utilite/UStl.h"

bool UFile::exists(const std::string &filePath)
{
    bool fileExists = false;
    std::ifstream in(filePath.c_str(), std::ios::in);
    if (in.is_open())
    {
        fileExists = true;
        in.close();   
    }
    return fileExists;
}

long UFile::length(const std::string &filePath)
{
    long fileSize = 0;
    FILE* fp = 0;
#ifdef _MSC_VER
    fopen_s(&fp, filePath.c_str(), "rb");
#else
    fp = fopen(filePath.c_str(), "rb");
#endif
    if(fp == NULL)
    {
        return 0;
    }

    fseek(fp , 0 , SEEK_END);
    fileSize = ftell(fp);
    fclose(fp);

    return fileSize;
}

int UFile::erase(const std::string &filePath)
{
    return std::remove(filePath.c_str());
}

int UFile::rename(const std::string &oldFilePath,
                     const std::string &newFilePath)
{
    return std::rename(oldFilePath.c_str(), newFilePath.c_str());
}

std::string UFile::getName(const std::string & filePath, bool withExtension)
{
	std::string fullPath = filePath;
	std::string name;
	for(int i=fullPath.size()-1; i>=0; --i)
	{
		if(fullPath[i] == '/' || fullPath[i] == '\\')
		{
			break;
		}
		else
		{
			name.insert(name.begin(), fullPath[i]);
		}
	}
	if(!withExtension)
	{
		std::vector<std::string> parts = uListToVector(uSplit(name, '.')); // only remove the last extension if many '.'
		if(parts.size() > 1)
		{
			std::string tmp;
			for(unsigned int i=0; i<parts.size()-1; ++i)
			{
				if(i>0)
				{
					tmp += ".";
				}
				tmp += parts[i];
			}
			name = tmp;
		}
	}
	return name;
}

std::string UFile::getExtension(const std::string &filePath)
{
	std::list<std::string> list = uSplit(filePath, '.');
	if(list.size()>1)
	{
		return list.back();
	}
	return "";
}

void UFile::copy(const std::string & from, const std::string & to)
{
	std::ifstream src(from.c_str());
	std::ofstream dst(to.c_str());

	dst << src.rdbuf();
}
