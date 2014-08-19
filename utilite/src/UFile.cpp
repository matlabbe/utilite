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
