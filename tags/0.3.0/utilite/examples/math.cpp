/*
 * math.cpp
 *
 *  Created on: Mar 6, 2012
 *      Author: MatLab
 */

#include <utilite/UMath.h>
#include <stdio.h>

int main(int argc, char * argv[])
{
	float x1[] = {1, 2, 5};
	float x2[] = {2, 5, 1};

	printf("x1=[%f, %f, %f];\n", x1[0], x1[1], x1[2]);
	printf("x2=[%f, %f, %f];\n", x2[0], x2[1], x2[2]);

	std::vector<float> r;
	r = uXMatch(x1, x2, 3, 3, UXCorrRaw);
	printf("uXMatch(x1, x2, UXCorrRaw)=[%f, %f, %f, %f, %f];\n", r[0], r[1], r[2], r[3], r[4]);

	r = uXMatch(x1, x2, 3, 3, UXCorrBiased);
	printf("uXMatch(x1, x2, UXCorrBiased)=[%f, %f, %f, %f, %f];\n", r[0], r[1], r[2], r[3], r[4]);

	r = uXMatch(x1, x2, 3, 3, UXCorrUnbiased);
	printf("uXMatch(x1, x2, UXCorrUnbiased)=[%f, %f, %f, %f, %f];\n", r[0], r[1], r[2], r[3], r[4]);

	r = uXMatch(x1, x2, 3, 3, UXCorrCoeff);
	printf("uXMatch(x1, x2, UXCorrCoeff)=[%f, %f, %f, %f, %f];\n", r[0], r[1], r[2], r[3], r[4]);

	r = uXMatch(x1, x2, 3, 3, UXCovRaw);
	printf("uXMatch(x1, x2, UXCovRaw)=[%f, %f, %f, %f, %f];\n", r[0], r[1], r[2], r[3], r[4]);

	r = uXMatch(x1, x2, 3, 3, UXCovBiased);
	printf("uXMatch(x1, x2, UXCovBiased)=[%f, %f, %f, %f, %f];\n", r[0], r[1], r[2], r[3], r[4]);

	r = uXMatch(x1, x2, 3, 3, UXCovUnbiased);
	printf("uXMatch(x1, x2, UXCovUnbiased)=[%f, %f, %f, %f, %f];\n", r[0], r[1], r[2], r[3], r[4]);

	r = uXMatch(x1, x2, 3, 3, UXCovCoeff);
	printf("uXMatch(x1, x2, UXCovCoeff)=[%f, %f, %f, %f, %f];\n", r[0], r[1], r[2], r[3], r[4]);

	UXMatchMethod method = UXCovCoeff;
	r[0] = uXMatch(x1, x2, 3, 3, 0, method);
	r[1] = uXMatch(x1, x2, 3, 3, 1, method);
	r[2] = uXMatch(x1, x2, 3, 3, 2, method);
	r[3] = uXMatch(x1, x2, 3, 3, 3, method);
	r[4] = uXMatch(x1, x2, 3, 3, 4, method);
	printf("uXMatch(x1, x2, indexes, UXCovCoeff)=[%f, %f, %f, %f, %f];\n", r[0], r[1], r[2], r[3], r[4]);

	return 0;
}
