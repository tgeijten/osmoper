// OSMoPer.cpp : Defines the entry point for the console application.
//

#include "OpSiMoPer.h"

#include <iostream>
#include <conio.h>
#include "CoordinateFrame.h"
#include "xo/system/log.h"
#include "xo/filesystem/filesystem.h"
#include "xo/system/log_sink.h"

using std::cout;
using std::endl;
using namespace moper;
using namespace xo;

int main(int argc, char* argv[])
{
	log::console_sink sink( log::info_level );

	if ( argc != 4 )
	{
		std::cout << "Usage:" << std::endl;
		std::cout << "OSMoPer <Personalized Model File (*.mpf)> <OpenSim Model (*.osim)> <Config file (*.ini)> " << std::endl;
		return -1;
	}

	try
	{
		path mpffile( argv[ 1 ] );
		path osmodelfile( argv[ 2 ] );
		path configfile( argv[ 3 ] );
		path output_filename = path( mpffile ).replace_extension( "osim" );
		path report_prefix = mpffile.parent_path() / "reports" / mpffile.stem();
		create_directories( report_prefix.parent_path() );

		log::file_sink sink( log::trace_level, report_prefix + ".log" );

		OpenSim::Model model( osmodelfile.string() );
		moper::OpSiMoPer moper( model, configfile.string() );
		moper.ApplyPersonalization( mpffile.string() );
		model.print( output_filename.string() );
		log::info( "Model conversion successful, output written to: " + output_filename.string() );

		// write reports
		//moper.WriteModelInfo( report_prefix.string() + "_model_info.txt" );
		std::ofstream( report_prefix.string() + "_vol_report.txt" ) << moper.vol_report.str();
		std::ofstream( report_prefix.string() + "_vp_report.txt" ) << moper.vp_report.str();
		std::ofstream( report_prefix.string() + "_mus_report.txt" ) << moper.mus_data;
	}
	catch ( std::exception& e )
	{
		log::critical( "ERROR: ", e.what() );
#ifdef _DEBUG
		cout << "Press any key to continue..." << endl;
		getch();
#endif
		return -1;
	}

#ifdef _DEBUG
	cout << "Press any key to continue..." << endl;
	getch();
#endif
	return 0;
}
