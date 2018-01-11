#pragma once

#include "common.h"

#include "xo/geometry/vec3.h"
#include "xo/system/log.h"

#include <string>
#include <OpenSim/OpenSim.h>
//#include <boost/property_tree/ptree.hpp>

#include "CoordinateFrame.h"
#include <sstream>
#include "xo/numerical/average.h"
#include "xo/container/table.h"

namespace moper
{
	using string = std::string;

	class OpSiMoPer 
	{
	public:
		OpSiMoPer( OpenSim::Model& model, const string& lookup_table );
		~OpSiMoPer() {}

		bool ApplyPersonalization( const string& moper_file );
		OpenSim::Model& GetModel() { return model; }

		bool WriteModelInfo( const string& output );

		std::stringstream vol_report;
		std::stringstream att_report;
		std::stringstream vp_report;
		std::stringstream cf_report;

		xo::table< double > mus_data;

	private:
		SimTK::Vec3 GetWorldPos( OpenSim::Body& body, const SimTK::Vec3& local_pos );
		bool ApplyMuscleVolume( const string& name, double volume );
		CoordinateFrame GetCoordinateFrame( const OpenSim::Body& body );
		CoordinateFrame GetCoordinateFrame( const string& name );
		double GetScaleFactor( OpenSim::Body& bod );
		bool ApplyMuscleAttachments( OpenSim::Muscle& mus );
		bool ScaleModel();

		string GetLookup( const string& key );
		string TryGetLookup( const string& key );
		string GetMpf( const string& key );
		string TryGetMpf( const string& key );

		typedef std::vector< std::pair< vec3, double > > muscle_path;

		muscle_path GetMpfPath( const string& muscle_name );
		muscle_path GetOsPath( const OpenSim::PathPointSet& pps );

		xo::average_<double> avg_attach_disp;

		bool Error( string str ) { xo::log::error( str ); return false; }
		bool Info( string str ) { xo::log::info( str ); return true; }
		vec3 vec3_from_string( std::string s );
		vec3 make_vec3( SimTK::Vec3& osv );
		vec3 GetInterpolatedPathPoint( const muscle_path& mpf_path, double os_len );

		OpenSim::Model& model;
		SimTK::State* tk_state;
		xo::prop_node lookup;
		xo::prop_node pers;
		bool use_muscle_via_point_interpolation = true;
	};
}
