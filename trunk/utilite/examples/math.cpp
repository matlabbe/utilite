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
