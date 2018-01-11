#include "stdafx.h"

#include "OpSiMoPer.h"

//#include <boost/property_tree/ini_parser.hpp>
//#include <boost/filesystem.hpp>
//#include <boost/format.hpp>
//#include <boost/algorithm/string.hpp>
#include "xo/string/string_tools.h"
#include "xo/stream/prop_node_tools.h"
#include "xo/filesystem/filesystem.h"

using std::cout;
using std::endl;
using namespace xo;

namespace moper
{
	OpSiMoPer::OpSiMoPer( OpenSim::Model& model, const string& lookup_table ) : model( model )
	{
		// read config file
		lookup = load_ini( lookup_table );
		//boost::property_tree::read_ini( lookup_table, lookup );

		// init model
		tk_state = &model.initSystem();

		// write report headers
		vol_report << "MuscleGroup\tOriginalVolume\tNewFolume\tFactor" << endl;
		vp_report << "Muscle\tnViaPointsDelp\tViaPointsMpf" << endl;
		
		// setup muscle table
		for ( int m = 0; m < model.updMuscles().getSize(); ++m )
			mus_data.add_row( model.updMuscles().get( m ).getName() );
		mus_data.add_column( "ForceScale", 1.0 );
	}

	bool OpSiMoPer::ApplyPersonalization( const std::string& moper_file )
	{
		// reset statistics
		avg_attach_disp.reset();

		try
		{
			// read moper file
			pers = load_ini( moper_file );
			//boost::property_tree::read_ini( moper_file, pers );

			for ( auto& p : pers )
				cout << p.first << endl;

			// apply coordinate frames
			if ( pers.has_key( "CoordinateFrames" ) )
			{
				log::info( "CoordinateFrames found, scaling model" );
				ScaleModel();
			}

			// apply muscle volumes
			auto musvols = pers.try_get_child( "MuscleVolumes" );
			if ( musvols )
			{
				// prepare table
				log::info( "MuscleVolumes found, scaling muscle forces" );
				for ( auto mpt : *musvols )
					ApplyMuscleVolume( mpt.first, mpt.second.get< double >() );
			}
			else log::info( "MuscleVolumes not found" );

			// apply muscle attachments
			if ( pers.has_key( "MuscleTendonLines_Delp") )
			{
				log::info( "MuscleAttachments found, updating muscle attachments" );
				for ( int idx = 0; idx < model.updMuscles().getSize(); ++idx )
					ApplyMuscleAttachments( model.updMuscles().get( idx ) );
				//log::info( "Average displacement: ", avg_attach_disp.get(), " Min: ", avg_attach_disp.min(), " Max: ", avg_attach_disp.max() );
				log::info( "Average displacement: ", avg_attach_disp.get() );
			}
			else log::info( "MuscleAttachments not found" );
		}
		catch ( std::exception& e )
		{
			cout << "ERROR: " << e.what() << endl;
			return false;
		}
		return true;
	}

	bool OpSiMoPer::WriteModelInfo( const string& output )
	{
		prop_node pt;
		//ptree pt;
		for ( int idx = 0; idx < model.updMuscles().getSize(); ++idx )
			pt.set( "MusclePathPoints." + model.updMuscles().get( idx ).getName(),
			model.getMuscles().get( idx ).getGeometryPath().getPathPointSet().getSize() );

		for ( int idx = 0; idx < model.updBodySet().getSize(); ++idx )
		{
			pt.set( "BodyNames." + model.updBodySet().get( idx ).getName(), "" );
			if ( model.updBodySet().get( idx ).hasJoint() )
				pt.set( "JointNames." + model.updBodySet().get( idx ).getJoint().getName(), "" );
		}

		xo::path filepath( output );
		xo::create_directories( filepath.parent_path() );
		save_ini( pt, filepath );
		//boost::property_tree::write_ini( filepath.string(), pt );

		return true;
	}

	SimTK::Vec3 OpSiMoPer::GetWorldPos( OpenSim::Body& body, const SimTK::Vec3& local_pos )
	{
		auto trans = model.getMatterSubsystem().getMobilizedBody( body.getIndex() ).getBodyTransform( *tk_state );
		auto world_pos = trans * local_pos;
		return world_pos;
	}

	bool OpSiMoPer::ApplyMuscleVolume( const string& name, double volume )
	{
		// get muscles
		// TODO: handle left / right cases properly
		string base_name = mid_str( name, in_str( name, "_" ) + 1 );
		std::vector< string > musnames = xo::split_str( lookup.get< string >( "MuscleVolumes." + base_name ), " \t" );
		if ( musnames.empty() )
			return Error( name + ": Not configured..." );

		string side_postfix = toupper( name[ 0 ] ) == 'L' ? "_l" : "_r";
	
		// get total current volume
		double org_vol = 0;
		for ( auto& m : musnames )
		{
			auto& mus = model.getMuscles().get( m + side_postfix );
			double sigma = .50e6;
			org_vol += ( mus.getMaxIsometricForce() / sigma ) * mus.getOptimalFiberLength();
		}

		double factor = volume / org_vol;

		// scale forces
		for ( auto& m : musnames )
		{
			auto& mus = model.updMuscles().get( m + side_postfix );
			auto org_fmax = mus.getMaxIsometricForce();
			mus.setMaxIsometricForce( factor * org_fmax );

			// report factor
			if ( mus_data( mus.getName(), "ForceScale" ) != 1.0 )
				log::error( mus.getName(), " has already been scaled!" );
			mus_data( mus.getName(), "ForceScale" ) = factor;
		}

		// report results
		vol_report << name << '\t' << org_vol * 1000 << '\t' << volume * 1000 << '\t' << ( volume / org_vol ) << endl;
		log::debug( name, ": volume set from ", org_vol * 1000, " to ", volume * 1000, " factor=", volume / org_vol );

		return true;
	}

	moper::CoordinateFrame OpSiMoPer::GetCoordinateFrame( const OpenSim::Body& body )
	{
		std::stringstream cfstream( GetLookup( "CoordinateFrames/" + body.getName() ) );
		string name;
		int ix = 0, iy = 1, iz = 2;
		cfstream >> name >> ix >> iy >> iz;
		CoordinateFrame cf;
		string basename = "CoordinateFrames/" + name;
		cf.o = vec3_from_string( GetMpf( basename + ".Origin" ) );
		cf.axes[ abs( ix ) - 1 ] = sign<real_t>( ix ) * normalized( vec3_from_string( GetMpf( basename + ".X" ) ) );
		cf.axes[ abs( iy ) - 1 ] = sign<real_t>( iy ) * normalized( vec3_from_string( GetMpf( basename + ".Y" ) ) );
		cf.axes[ abs( iz ) - 1 ] = sign<real_t>( iz ) * normalized( vec3_from_string( GetMpf( basename + ".Z" ) ) );
		cf.q = quat_from_axes( cf.axes[ 0 ], cf.axes[ 1 ], cf.axes[ 2 ] );
		return cf;
	}

	moper::CoordinateFrame OpSiMoPer::GetCoordinateFrame( const string& name )
	{
		CoordinateFrame cf;
		string basename = "CoordinateFrames/" + name;
		cf.o = vec3_from_string( GetMpf( basename + ".Origin" ) );
		cf.axes[ 0 ] = normalized( vec3_from_string( GetMpf( basename + ".X" ) ) );
		cf.axes[ 0 ] = normalized( vec3_from_string( GetMpf( basename + ".Y" ) ) );
		cf.axes[ 0 ] = normalized( vec3_from_string( GetMpf( basename + ".Z" ) ) );
		return cf;
	}

	real_t OpSiMoPer::GetScaleFactor( OpenSim::Body& bod )
	{
		if ( !bod.hasJoint() )
		{
			log::warning( bod.getName(), " does not have a joint" );
			return 0;
		}

		try
		{
			// find the segment length of the opensim body
			auto& mss = model.getMultibodySystem().getMatterSubsystem();
			auto& mb = mss.getMobilizedBody( bod.getIndex() );
	
			auto& bodyset = model.getBodySet();
			averaged scale_factor;
	
			// just get the value from the settings
			string settings_str = TryGetLookup( "BodyScaleSettings/" + bod.getName() );
			if ( !settings_str.empty() )
			{
				auto ss = split_str( settings_str, " \t" );
				auto pos0 = mss.getMobilizedBody( model.getBodySet().get( ss[ 0 ] ).getIndex() ).getBodyOriginLocation( *tk_state );
				auto pos1 = mss.getMobilizedBody( model.getBodySet().get( ss[ 1 ] ).getIndex() ).getBodyOriginLocation( *tk_state );
				auto cf0 = GetCoordinateFrame( ss[ 2 ] );
				auto cf1 = GetCoordinateFrame( ss[ 3 ] );
				auto mplen = length( cf0.o - cf1.o );
				auto oslen = ( pos0 - pos1 ).norm();
				double factor = mplen / oslen;
				log::debug( "Scaling ", bod.getName(), " from ", ss[0], "-", ss[1], "=", oslen, " to ", ss[2], "-", ss[3], "=", mplen, " factor=", factor );
				
				return factor;
			}
			else return 1.0;
		}
		catch ( std::exception& e )
		{
			log::warning( bod.getName(), " scaling error: ", e.what() );
			return 1.0;
		}
	}

	bool OpSiMoPer::ApplyMuscleAttachments( OpenSim::Muscle& mus )
	{
		try
		{
			// find moper muscle name
			auto os_base_name = left_str( mus.getName(), -2 );
			auto os_side_name = right_str( mus.getName(), 2 );
			auto mpf_base_name = GetLookup( "MuscleTendonLines_Delp/" + os_base_name );
			auto mpf_side_name = os_side_name == "_l" ? "Left_" : "Right_";
			auto mpf_name = mpf_side_name + mpf_base_name;
	
			// get attachment points
			auto& geom_path = mus.updGeometryPath();
			auto& pps = geom_path.updPathPointSet();
			auto pps_size = pps.getSize();

			// output number of via points
			int mpf_vp_count = 0;
			while ( !TryGetMpf( "MuscleTendonLines_Delp/" + mpf_name + '.' + to_str( mpf_vp_count ) ).empty() )
				++mpf_vp_count;
			vp_report << mpf_name << "\t" << pps.getSize() << "\t" << mpf_vp_count << endl;

			// Get the os path first (because we're going to change it!)
			auto os_path = GetOsPath( pps );
			double os_len = geom_path.getLength( *tk_state );

			for ( int idx = 0; idx < pps.getSize(); ++idx )
			{
				auto os_pp = pps.get( idx );
				vec3 mpf_pp_world = vec3::zero(); // the global pos of the path point

				if ( use_muscle_via_point_interpolation )
				{
					auto mpf_path = GetMpfPath( "MuscleTendonLines_Delp/" + mpf_name );
					real_t rel_pos = os_path[ idx ].second / os_path.back().second;
					mpf_pp_world = GetInterpolatedPathPoint( mpf_path, rel_pos );
				}
				else
				{
					// get point in moper
					auto point_name = mpf_name + '.' + to_str( idx );
					auto pstr = TryGetMpf( "MuscleTendonLines_Delp/" + point_name );
					if ( !pstr.empty() )
						mpf_pp_world = vec3_from_string( pstr );
				}

				if ( !mpf_pp_world.is_null() )
				{
					// transform to local coordinate
					auto cf = GetCoordinateFrame( os_pp.getBody() );
					vec3 mpf_pp_local = transform_to( cf, mpf_pp_world );
					auto os_point_vec3 = make_vec3( os_pp.getLocation() );
					auto dist = length( os_point_vec3 - mpf_pp_local );

					//log::trace( mus.getName(), std::setprecision( 3 ), '.', idx, ": from ", os_point_vec3, " to ", mpf_pp_local, " dist=", dist );

					os_pp.getLocation()[0] = mpf_pp_local.x;
					os_pp.getLocation()[1] = mpf_pp_local.y;
					os_pp.getLocation()[2] = mpf_pp_local.z;

					// update report and statistics
					avg_attach_disp.add( dist );
				}
				else log::warning( mus.getName(), ": Skipping via point ", idx );

			}
		}
		catch ( std::exception& e )
		{
			return Error( "Error converting " + mus.getName() + ": " + e.what() );
		}

		return true;
	}

	bool OpSiMoPer::ScaleModel()
	{
		int i;
		OpenSim::ScaleSet theScaleSet;
		SimTK::Vec3 unity( 1.0 );
		auto& bodySet = model.getBodySet();
		for (i = 0; i < bodySet.getSize(); i++)
		{
			auto factor = GetScaleFactor( bodySet.get( i ) );
			if ( factor > 0 )
			{
				OpenSim::Scale* bodyScale = new OpenSim::Scale();
				bodyScale->setSegmentName( bodySet.get( i ).getName() );
				bodyScale->setScaleFactors( SimTK::Vec3( factor ) );
				bodyScale->setApply( true );
				theScaleSet.adoptAndAppend( bodyScale );
			}
		}

		model.scale( *tk_state, theScaleSet, 50, true );

		return true;
	}

	std::string OpSiMoPer::GetLookup( const string& key )
	{
		return lookup.get_delimited< string >( key, '/' );
	}

	std::string OpSiMoPer::TryGetLookup( const string& key )
	{
		return lookup.get_delimited< string >( key, '/' );
	}

	std::string OpSiMoPer::GetMpf( const string& key )
	{
		return pers.get_delimited< string >( key, '/' );
	}

	std::string OpSiMoPer::TryGetMpf( const string& key )
	{
		return pers.get_delimited< string >( key, '/' );
	}

	OpSiMoPer::muscle_path OpSiMoPer::GetMpfPath( const string& muscle_name )
	{
		muscle_path path;
		for ( index_t idx = 0; idx < 1000; ++idx )
		{
			string str = TryGetMpf( muscle_name + stringf( ".%d", idx ) );
			if ( str.empty() )
				break; // no more points available
			vec3 pp = vec3_from_string( str );
			if ( idx > 0 )
			{
				auto& prev = path.back();
				auto len = prev.second + length( pp - prev.first );
				path.push_back( std::make_pair( pp, len ) );

			}
			else path.push_back( std::make_pair( pp, 0.0 ) );

		}
		return path;
	}

	OpSiMoPer::muscle_path OpSiMoPer::GetOsPath( const OpenSim::PathPointSet& pps )
	{
		muscle_path path;
		for ( int idx = 0; idx < pps.getSize(); ++idx )
		{
			auto pp_world = GetWorldPos( pps.get( idx ).getBody(), pps.get( idx ).getLocation() );
			if ( idx > 0 )
			{
				auto& prev = path.back();
				auto len = prev.second + length( make_vec3( pp_world ) - prev.first );
				path.push_back( std::make_pair( make_vec3( pp_world ), len ) );
			}
			else path.push_back( std::make_pair( make_vec3( pp_world ), 0 ) );
		}
		return path;
	}

	vec3 OpSiMoPer::vec3_from_string( std::string s )
	{
		vec3 v( 0, 0, 0 );
		std::stringstream str( s );
		str >> v;
		return v;
	}

	vec3 OpSiMoPer::make_vec3( SimTK::Vec3& osv )
	{
		return vec3( osv[0], osv[1], osv[2] );
	}

	vec3 OpSiMoPer::GetInterpolatedPathPoint( const muscle_path& path, double os_rel_len )
	{
		xo_assert( path.size() >= 2 );

		auto path_len = path.back().second;
		index_t idx = 0;
		while ( ( idx < path.size() - 2 ) && ( path[ idx + 1 ].second / path_len ) < os_rel_len )
			++idx;

		// now interpolate between idx and idx + 1
		double rel_len0 = path[ idx ].second / path_len;
		double rel_len1 = path[ idx + 1 ].second / path_len;
		double w = ( os_rel_len - rel_len0 ) / ( rel_len1 - rel_len0 );

		xo_assert( w >= 0 && w <= 1 );

		vec3 pos = ( 1 - w ) * path[ idx ].first + w * path[ idx + 1 ].first;
		return pos;
	}
}
