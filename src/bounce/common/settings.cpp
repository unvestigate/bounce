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

#include <bounce/common/settings.h>
#include <bounce/common/common.h>
#include <bounce/common/math/math.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>

uint32 b3_allocCalls = 0;
uint32 b3_maxAllocCalls = 0;

b3Version b3_version = { 0, 0, 0 };

void* b3Alloc_Default(uint32 size) 
{
	++b3_allocCalls;
	b3_maxAllocCalls = b3Max(b3_maxAllocCalls, b3_allocCalls);
	return malloc(size);
}

void b3Free_Default(void* block) 
{
	return free(block);
}

void b3Log_Default(const char* text, va_list args)
{
	vprintf(text, args);
}
