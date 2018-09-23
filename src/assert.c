#include "assert.h"


// If debugging turned on, define the real assert function
#if ASSERT_DEBUG

// Put program in infinite loop if arg is false
void assert_issue(bool arg, char* file, char* base_file, int line)
{
	if(arg == false)
	{
		while(1);
	}
}

#endif //ASSERT_DEBUG
