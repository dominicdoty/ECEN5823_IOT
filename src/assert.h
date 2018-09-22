#ifndef ASSERT_H
#define ASSERT_H

#include "main.h"

// Note: not a direct copy, but heavily influenced by the following:
// https://nicisdigital.wordpress.com/2011/07/05/assertions-in-microcontrollers/

// For debugging- check if input is not true
// Put program in an infinite loop
// Statements are deleted if "ASSERT_DEBUG" = false

// Default to debugging on if not otherwise defined
#ifndef ASSERT_DEBUG
	#define ASSERT_DEBUG true
#endif //ASSERT_DEBUG

#if ASSERT_DEBUG		//if debugging on, replace assert calls with real function calls
	#define ASSERT(arg) assert_issue(arg, __FILE__, __BASE_FILE__, __LINE__)	//this allows us to record the file names and line # where called
	void assert_issue(bool arg, char* file, char* base_file, int line);			//declare real assert function
#else
	#define ASSERT(arg) //otherwise replace with nothing
#endif

#endif // ASSERT_H
