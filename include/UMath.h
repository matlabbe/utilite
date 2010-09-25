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

#ifndef MATHFUNCTIONS_H_
#define MATHFUNCTIONS_H_

#include "UtiLiteExp.h" // DLL export/import defines

#include <math.h>
#include <list>
#include <vector>

/**
 * Get the maximum of a vector.
 * @param v the array
 * @param size the size of the array
 * @param index the index of the maximum value in the vector.
 * @return the maximum value of the array
 */
template<class T>
inline T uMax(const T * v, unsigned int size, unsigned int & index = 0)
{
	T max = 0;
	if(!v || size == 0)
	{
		return max;
	}
	index = 0;
	for(unsigned int i=0; i<size; i++)
	{
		if(max < v[i])
		{
			max = v[i];
			index = i;
		}
	}
	return max;
}

/**
 * Get the sign of value.
 * @param v the value
 * @return -1 if v<0, otherwise 1
 */
template<class T>
inline int uSign(const T & v)
{
	if(v < 0)
	{
		return -1;
	}
	else
	{
		return 1;
	}
}

/**
 * Get the sum of all values contained in a list. Provided for convenience.
 * @param list the list
 * @return the sum of values of the list
 */
template<class T>
inline T uSum(const std::list<T> & list)
{
	T sum = 0;
	for(typename std::list<T>::const_iterator i=list.begin(); i!=list.end(); ++i)
	{
		sum += *i;
	}
	return sum;
}

/**
 * Get the sum of all values contained in a vector. Provided for convenience.
 * @param v the vector
 * @return the sum of values of the vector
 */
template<class T>
inline T uSum(const std::vector<T> & v)
{
	T sum = 0;
	for(unsigned int i=0; i<v.size(); ++i)
	{
		sum += v[i];
	}
	return sum;
}

/**
 * Get the sum of all values contained in an array.
 * @param v the array
 * @param size the size of the array
 * @return the sum of values of the array
 */
template<class T>
inline T uSum(const T * v, unsigned int size)
{
	T sum = 0;
	if(v && size)
	{
		for(unsigned int i=0; i<size; ++i)
		{
			sum += v[i];
		}
	}
	return sum;
}

/**
 * Compute the mean of an array.
 * @param v the array
 * @param size the size of the array
 * @return the mean
 */
template<class T>
inline T uMean(const T * v, unsigned int size)
{
	T buf = 0;
	if(v && size)
	{
		for(unsigned int i=0; i<size; ++i)
		{
			buf += v[i];
		}
		buf /= size;
	}
	return buf;
}

/**
 * Get the mean of a list. Provided for convenience.
 * @param list the list
 * @return the mean
 */
template<class T>
inline T uMean(const std::list<T> & list)
{
	T m = 0;
	if(list.size())
	{
		for(typename std::list<T>::const_iterator i=list.begin(); i!=list.end(); ++i)
		{
			m += *i;
		}
		m /= list.size();
	}
	return m;
}

/**
 * Get the mean of a vector. Provided for convenience.
 * @param v the vector
 * @return the mean
 */
template<class T>
inline T uMean(const std::vector<T> & v)
{
	return uMean(v.data(), v.size());
}

/**
 * Compute the standard deviation of an array.
 * @param v the array
 * @param size the size of the array
 * @param meanV the mean of the array
 * @return the std dev
 * @see mean()
 */
template<class T>
inline T uStdDev(const T * v, unsigned int size, T meanV)
{
	T buf = 0;
	if(v && size>1)
	{
		float sum = 0;
		for(unsigned int i=0; i<size; ++i)
		{
			sum += (v[i]-meanV)*(v[i]-meanV);
		}
		buf = sqrt(sum/(size-1));
	}
	return buf;
}

/**
 * Get the standard deviation of a list. Provided for convenience.
 * @param list the list
 * @param m the mean of the list
 * @return the std dev
 * @see mean()
 */
template<class T>
inline T uStdDev(const std::list<T> & list, const T & m)
{
	T buf = 0;
	if(list.size()>1)
	{
		float sum = 0;
		for(typename std::list<T>::const_iterator i=list.begin(); i!=list.end(); ++i)
		{
			sum += (*i-m)*(*i-m);
		}
		buf = sqrt(sum/(list.size()-1));
	}
	return buf;
}

/**
 * Compute the standard deviation of an array.
 * @param v the array
 * @param size the size of the array
 * @return the std dev
 */
template<class T>
inline T uStdDev(const T * v, unsigned int size)
{
	T m = uMean(v, size);
	return uStdDev(v, size, m);
}

/**
 * Get the standard deviation of a vector. Provided for convenience.
 * @param v the vector
 * @param m the mean of the vector
 * @return the std dev
 * @see mean()
 */
template<class T>
inline T uStdDev(const std::vector<T> & v, const T & m)
{
	return uStdDev(v.data(), v.size(), m);
}

/**
 * Do a full cross correlation between 2 arrays.
 * @param vA the first array
 * @param vB the second array
 * @param sizeA the size of the first array
 * @param sizeB the size of the second array
 * @return the resulting correlation vector of size = (sizeA + sizeB)-1
 */
inline std::vector<float> uXCorr(const float * vA, const float * vB, unsigned int sizeA, unsigned int sizeB)
{
	if(!vA || !vB || sizeA == 0 || sizeB == 0)
	{
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

/**
 * Do a cross correlation between 2 arrays at a specified index.
 * @param vA the first array
 * @param vB the second array
 * @param sizeA the size of the first array
 * @param sizeB the size of the second array
 * @param index the index to correlate
 * @param meanA the mean of the array A
 * @param meanB the mean of the array B
 * @param stdDevAB the std dev of the 2 arrays: stdDevAB = stdDevA*stdDevB
 * @return the resulting correlation value
 */
inline float uXCorr(const float * vA, const float * vB, unsigned int sizeA, unsigned int sizeB, unsigned int index, float meanA, float meanB, float stdDevAB)
{
	float result = 0;
	if(!vA || !vB || sizeA == 0 || sizeB == 0)
	{
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

/**
 * Do a cross correlation between 2 arrays at a specified index.
 * The mean and the std dev are automatically computed for each array.
 * @param vA the first array
 * @param vB the second array
 * @param sizeA the size of the first array
 * @param sizeB the size of the second array
 * @param index the index to correlate
 * @return the resulting correlation value
 */
inline float uXCorr(const float * vA, const float * vB, unsigned int sizeA, unsigned int sizeB, unsigned int index)
{
	float meanA = uMean(vA, sizeA);
	float meanB = uMean(vB, sizeB);

	float stdDevA = uStdDev(vA, sizeA, meanA);
	float stdDevB = uStdDev(vB, sizeB, meanB);

	return uXCorr(vA, vB, sizeA, sizeB, index, meanA, meanB, stdDevA * stdDevB);
}

#endif // MATHFUNCTIONS_H_
