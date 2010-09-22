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

#include "UMathFunctions.h"
#include <cmath>

std::vector<float> uXCorr(const float * vA, const float * vB, unsigned int sizeA, unsigned int sizeB)
{
	if(!vA || !vB || sizeA == 0 || sizeB == 0)
	{
		LOGGER_ERROR("Util::xcorr() Parameters not valid.");
		return std::vector<float>();
	}

	std::vector<float> result(sizeA+sizeB-1);

	float meanA = uMean(vA, sizeA);
	float meanB = uMean(vB, sizeB);

	float stdDevA = uStdDev(vA, sizeA, meanA);
	float stdDevB = uStdDev(vB, sizeB, meanB);

	float resultA;
	float resultB;

	float den = stdDevA * stdDevB * result.size();

	int posA;
	int posB;
	unsigned int j;
	unsigned int endLoop = sizeA;
	if(sizeB<sizeA)
	{
		endLoop = sizeB;
	}

	for(unsigned int i=0; i<endLoop; i++)
	{
		posA = sizeA - i - 1;
		posB = sizeB - i - 1;
		resultA = 0;
		resultB = 0;
		for(j=0; (j + posB) < sizeB && (j + posA) < sizeA; ++j)
		{
			resultA += (vA[j] - meanA) * (vB[j + posB] - meanB);
			resultB += (vA[j + posA] - meanA) * (vB[j] - meanB);
		}
		result[i] = resultA / den;
		result[result.size()-1 -i] = resultB / den;
	}

	return result;
}

float uXCorr(const float * vA, const float * vB, unsigned int sizeA, unsigned int sizeB, unsigned int index)
{
	float meanA = uMean(vA, sizeA);
	float meanB = uMean(vB, sizeB);

	float stdDevA = uStdDev(vA, sizeA, meanA);
	float stdDevB = uStdDev(vB, sizeB, meanB);
	
	return uXCorr(vA, vB, sizeA, sizeB, index, meanA, meanB, stdDevA * stdDevB);
}

float uXCorr(const float * vA, const float * vB, unsigned int sizeA, unsigned int sizeB, unsigned int index, float meanA, float meanB, float stdDevAB)
{
	float result = 0;
	if(!vA || !vB || sizeA == 0 || sizeB == 0)
	{
		LOGGER_ERROR("Util::xcorr() Parameters not valid.");
		return result;
	}

	unsigned int size = sizeA + sizeB - 1;

	if(index < size)
	{
		int posB = sizeB - index - 1;
		unsigned int i;
		if(posB >= 0)
		{
			for(i=0; (i + posB) < sizeB && i < sizeA; ++i)
			{
				result += (vA[i] - meanA) * (vB[i + posB] - meanB);
			}
		}
		if(posB < 0)
		{
			int posA = posB*-1;
			for(i=0; (i+posA) < sizeA && i < sizeB; ++i)
			{
				result += (vA[i+posA] - meanA) * (vB[i] - meanB);
			}
		}
	}

	if(sizeA > sizeB)
	{
		result /= (stdDevAB * sizeA); 
	}
	else
	{
		result /= (stdDevAB * sizeB); 
	}

	return result; 
}
