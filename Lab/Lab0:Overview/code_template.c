#include <standard_headers.h>
#include "headers.h"

// Function Prototypes/Declarations
type function_1(type parameter_1, type parameter_2, ... );

// Global variables
// example:
int global_var_1;
float global_var_2;
// main function.

int main() { // return value of int is a convention
	// local variables
	// example:
	int local_var_1; // variable declaration. Value initially garbage
	local_var_1 = 10; // variable definition
	int local_var_2 = 0; // initialization combines declare/define
	global_var_1 = 1; // can access global variables anywhere in program
	
	// do something
	// ...
	function_1(local_var_1, local_var_2, ... ); // function call
	// ...
	
	// while(1) loops are common in embedded systems
	while( conditional) {
		// do while conditional is true
	}
	
	return; //
}

// function definitions

type function_1(type parameter_1, type parameter_2, ... ) {
	// can access parameter_1, parameter_2, etc. in function_1
	/* note that parameters are PASSED BY VALUE. That is, consider
	   parameter_1, parameter_2, etc. to be copies, not the same as
	   the original
	*/
	global_var_1 = 0;
	return return_val; //return_val has the same type as the declared function 
}
