#ifndef ASSERT_H
#define ASSERT_H

#include <stdint.h>
#include <stdbool.h>

// Note: not a direct copy, but heavily influenced by the following:
// https://nicisdigital.wordpress.com/2011/07/05/assertions-in-microcontrollers/

// Function for debugging
// If "ASSERT_DEBUG == true", all ASSERT; calls are replaced with a real function call
// This function records the location from where it was called, and puts the program in an infinite loop if the argument is false
//
// If "ASSERT_DEBUG == false", all ASSERT(); calls are automatically deleted


// Default to debugging on if not otherwise defined
#ifndef ASSERT_DEBUG
	#define ASSERT_DEBUG true
#endif //ASSERT_DEBUG


#if ASSERT_DEBUG																//if debugging on, replace assert calls with real function calls
	#define ASSERT(arg) assert_issue(arg, __FILE__, __BASE_FILE__, __LINE__)	//this allows us to record the file names and line # where called
	void assert_issue(bool arg, char* file, char* base_file, int line);			//prototype real assert function
#else
	#define ASSERT(arg) //nothing												//otherwise replace with nothing
#endif

#endif // ASSERT_H
