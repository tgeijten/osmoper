// OSMoPer.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include "OpSiMoPer.h"

#include <iostream>
#include <conio.h>
#include "CoordinateFrame.h"
#include "xo/system/log.h"
#include "xo/filesystem/filesystem.h"

using std::cout;
using std::endl;
using namespace moper;
using namespace xo;

int main(int argc, char* argv[])
{
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
		OpenSim::Model model( osmodelfile.string() );
		moper::OpSiMoPer moper( model, configfile.string() );
		moper.WriteModelInfo( "debug_output/model_info.txt" );
		moper.ApplyPersonalization( mpffile.string() );
		path output_filename = mpffile;
		model.print( ( output_filename.replace_extension( "osim" ) ).string() );
		std::cout << "Model conversion successful, output written to: " + output_filename.string() << std::endl;

		// write reports
		output_filename = output_filename.parent_path() / path( "reports" ) / output_filename.filename();
		create_directories( output_filename.parent_path() );
		output_filename.replace_extension("");
		std::ofstream( output_filename.string() + "_vol_report.txt" ) << moper.vol_report.str();
		std::ofstream( output_filename.string() + "_vp_report.txt" ) << moper.vp_report.str();
		std::ofstream( output_filename.string() + "_mus_report.txt" ) << moper.mus_data;
	}
	catch ( std::exception& e )
	{
		cout << "ERROR: " << e.what() << endl;
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
