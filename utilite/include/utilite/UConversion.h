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

#ifndef UCONVERSION_H
#define UCONVERSION_H

#include "utilite/UtiLiteExp.h" // DLL export/import defines

#include <string>
#include <vector>
#include <stdarg.h>

/**
 * \file UConversion.h
 * \brief Some conversion functions
 *
 * This contains functions to do some convenient conversion like
 * uNumber2str(), uBytes2Hex() or uHex2Bytes().
*/

/**
 * Replace old characters in a string to new ones.
 * Example :
 * @code
 * std::string str = "Hello";
 * uReplaceChar(str, 'l', 'p');
 * // The results is str = "Heppo";
 * @endcode
 *
 * @param str the string
 * @param before the character to be replaced by the new one
 * @param after the new character replacing the old one
 * @return the modified string
 */
std::string UTILITE_EXP uReplaceChar(const std::string & str, char before, char after);

/**
 * Replace old characters in a string with the specified string.
 * Example :
 * @code
 * std::string str = "Hello";
 * uReplaceChar(str, 'o', "oween");
 * // The results is str = "Helloween";
 * @endcode
 *
 * @param str the string
 * @param before the character to be replaced by the new one
 * @param after the new string replacing the old character
 * @return the modified string
 */
std::string UTILITE_EXP uReplaceChar(const std::string & str, char before, const std::string & after);

/**
 * Transform characters from a string to upper case.
 * Example :
 * @code
 * std::string str = "hello!";
 * str = uToUpperCase(str);
 * //str is now equal to "HELLO!"
 * @endcode
 * @param str the string
 * @return the modified string
 */
std::string UTILITE_EXP uToUpperCase(const std::string & str);

/**
 * Transform characters from a string to lower case.
 * Example :
 * @code
 * std::string str = "HELLO!";
 * str = uToLowerCase(str, false);
 * //str is now equal to "hello!"
 * @endcode
 * @param str the string
 * @return the modified string
 */
std::string UTILITE_EXP uToLowerCase(const std::string & str);

/**
 * Convert a number (char) to a string.
 * @param number the number to convert in a string
 * @return the string
 */
std::string UTILITE_EXP uNumber2Str(char number);

/**
 * Convert a number (unsigned char) to a string.
 * @param number the number to convert in a string
 * @return the string
 */
std::string UTILITE_EXP uNumber2Str(unsigned char number);

/**
 * Convert a number (short int) to a string.
 * @param number the number to convert in a string
 * @return the string
 */
std::string UTILITE_EXP uNumber2Str(short number);

/**
 * Convert a number (unsigned short int) to a string.
 * @param number the number to convert in a string
 * @return the string
 */
std::string UTILITE_EXP uNumber2Str(unsigned short number);

/**
 * Convert a number (unsigned int) to a string.
 * @param number the number to convert in a string
 * @return the string
 */
std::string UTILITE_EXP uNumber2Str(unsigned int number);
/**
 * Convert a number (int) to a string.
 * @param number the number to convert in a string
 * @return the string
 */
std::string UTILITE_EXP uNumber2Str(int number);
/**
 * Convert a number (float) to a string.
 * @param number the number to convert in a string
 * @return the string
 */
std::string UTILITE_EXP uNumber2Str(float number);
/**
 * Convert a number (double) to a string.
 * @param number the number to convert in a string
 * @return the string
 */
std::string UTILITE_EXP uNumber2Str(double number);

/**
 * Convert a string to a float independent of the locale (comma/dot).
 * @param the string
 * @return the number
 */
float UTILITE_EXP uStr2Float(const std::string & str);

/**
 * Convert a string to a double independent of the locale (comma/dot).
 * @param the string
 * @return the number
 */
double UTILITE_EXP uStr2Double(const std::string & str);

/**
 * Convert a bool to a string.
 * The format used is "true" and "false".
 * @param boolean the boolean to convert in a string
 * @return the string
 */
std::string UTILITE_EXP uBool2Str(bool boolean);
/**
 * Convert a string to a boolean.
 * The format used is :
 * "false", "FALSE" or "0" give false. All others give true.
 * @param str the string to convert in a boolean
 * @return the boolean
 */
bool UTILITE_EXP uStr2Bool(const char * str);

/**
 * Convert a bytes array to an hexadecimal string.
 * The resulting string is twice the size of the bytes array. The hexadecimal
 * Characters are in upper case.
 * Example :
 * @code
 * char bytes[] = {0x3F};
 * std::string hex = uBytes2Hex(bytes, 1);
 * // The string constains "3F".
 * @endcode
 *
 * @param bytes the bytes array
 * @param bytesLen the length of the bytes array
 * @return the hexadecimal string
 */
std::string UTILITE_EXP uBytes2Hex(const char * bytes, unsigned int bytesLen);
/**
 * Convert an hexadecimal string to a bytes array.
 * The string must be pair length. The hexadecimal
 * Characters can be in upper or lower case.
 * Example :
 * @code
 * std::string hex = "1f3B";
 * std::vector<char> bytes = uHex2Bytes(hex);
 * // The array contains {0x1F, 0x3B}.
 * @endcode
 *
 * @param hex the hexadecimal string
 * @return the bytes array
 */
std::vector<char> UTILITE_EXP uHex2Bytes(const std::string & hex);
/**
 * Convert an hexadecimal string to a bytes array.
 * The string must be pair length. The hexadecimal
 * Characters can be in upper or lower case.
 * Example :
 * @code
 * std::vector<char> bytes = uHex2Bytes("1f3B", 4);
 * // The array contains {0x1F, 0x3B}.
 * @endcode
 *
 * @param hex the hexadecimal string
 * @param bytesLen the hexadecimal string length
 * @return the bytes array
 */
std::vector<char> UTILITE_EXP uHex2Bytes(const char * hex, int hexLen);

/**
 * Convert an hexadecimal string to an ascii string. A convenient way
 * when using only strings.
 * The hexadecimal str MUST not contains any null values 0x00 ("00").
 * Think to use of hex2bytes() to handle 0x00 values.
 * Characters can be in upper or lower case.
 * Example :
 * @code
 * std::string str = uHex2Str("48656C6C4F21");
 * // The string contains "Hello!".
 * @endcode
 *
 * @see hex2bytes
 * @param hex the hexadecimal string
 * @return the ascii string
 */
std::string UTILITE_EXP uHex2Str(const std::string & hex);

/**
 * Convert hexadecimal (left or right part) value to an ascii character.
 * Example :
 * @code
 * unsigned char F = uHex2Ascii(0xFA, false);
 * unsigned char A = uHex2Ascii(0xFA, true);
 * @endcode
 * @see ascii2hex
 * @param c the hexadecimal value
 * @param rightPart If we want the character corresponding to the right of left part (4 bits) of the byte value.
 * @return the ascii character (in upper case)
 */
unsigned char UTILITE_EXP uHex2Ascii(const unsigned char & c, bool rightPart);

/**
 * Convert an ascii character to an hexadecimal value (right 4 bits).
 * Characters can be in upper or lower case.
 * Example :
 * @code
 * unsigned char hex = uAscii2Hex('F');
 * // The results is hex = 0x0F;
 * @endcode
 * @see hex2ascii
 * @param c the ascii character
 * @return the hexadecimal value
 */
unsigned char UTILITE_EXP uAscii2Hex(const unsigned char & c);

/**
 * Format a string like printf, and return it as a std::string
 */
std::string UTILITE_EXP uFormatv (const char *fmt, va_list ap);

/**
 * Format a string like printf, and return it as a std::string
 */
std::string UTILITE_EXP uFormat (const char *fmt, ...);

#ifdef WIN32
/**
 * Convert multi-byte string to unicode (wide-char) string.
 * Note that returned whar_t * must be deleted : delete [] wText;
 */
UTILITE_EXP wchar_t * createWCharFromChar(const char * text);

/**
 * Convert unicode (wide-char) string to multi-byte string.
 * Note that returned char * must be deleted : delete [] text;
 */
UTILITE_EXP char * createCharFromWChar(const wchar_t * wText);
#endif

#endif /* UCONVERSION_H */
