/*
* Copyright (c) 2016-2019 Irlan Robson 
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef B3_SETTINGS_H
#define B3_SETTINGS_H

#include <bounce/common/types.h>

#ifdef B3_USER_SETTINGS

// This is a user file that includes custom definitions of the macros, structs, and functions
#include <bounce/common/user_settings.h>

#else 

#include <stdarg.h>
#include <stdint.h>

// Default allocation functions.
void* b3Alloc_Default(uint32 size);
void b3Free_Default(void* block);

// You should implement this function to use your own memory allocator.
inline void* b3Alloc(uint32 size)
{
	return b3Alloc_Default(size);
}

// You must implement this function if you have implemented b3Alloc.
inline void b3Free(void* block)
{
	b3Free_Default(block);
}

// Default logging function.
void b3Log_Default(const char* string, va_list args);

// You should implement this function to visualize log messages coming 
// from this software.
inline void b3Log(const char* string, ...)
{
	va_list args;
	va_start(args, string);
	b3Log_Default(string, args);
	va_end(args);
}

#endif // B3_USER_SETTINGS

#include <bounce/common/common.h>

#endif
