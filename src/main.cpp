#include "xo/system/log.h"

int main( int argc, char* argv[] )
{
	xo::log::info( "Hello from xo!" );
	
	xo::wait_for_key();
	
	return 0;
}
