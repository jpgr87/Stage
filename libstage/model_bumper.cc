///////////////////////////////////////////////////////////////////////////
//
// File: model_bumper.c
// Author: Richard Vaughan
// Date: 28 March 2006
//
// CVS info:
//  $Source: /home/tcollett/stagecvs/playerstage-cvs/code/stage/src/model_bumper.c,v $
//  $Author: rtv $
//  $Revision:$
//
///////////////////////////////////////////////////////////////////////////

/**
@ingroup model
@defgroup model_bumper Bumper/Whisker  model
The bumper model simulates an array of binary touch sensors.

<h2>Worldfile properties</h2>

@par Summary and default values

@verbatim
bumper
(
  # bumper properties
  bcount 1
  bpose[0] [ 0 0 0 0 ]
  blength 0.1
)
@endverbatim

@par Notes

The bumper model allows configuration of the pose and length parameters of each transducer seperately using bpose[index]
and blength[index]. For convenience, the length of all bumpers in the array can be set at once with the blength property.
If a blength with no index is specified, this global setting is applied first, then specific blengh[index] properties are
applied afterwards. Note that the order in the worldfile is ignored.

@par Details
- bcount int
  - the number of bumper transducers
- bpose[\<transducer index\>] [float float float]
  - [x y z theta]
  - pose of the center of the transducer relative to its parent.
- blength float
  - sets the length in meters of all transducers in the array
- blength[\<transducer index\>] float
  - length in meters of a specific transducer. This is applied after the global setting above.

*/

#include "stage.hh"
#include "worldfile.hh"
#include "option.hh"
using namespace Stg;

#include <math.h>

static const watts_t BUMPER_WATTS = 0.1; // bumper power consumption
static const char* BUMPER_HIT_COLOR = "red"; // red
static const char* BUMPER_NOHIT_COLOR = "green"; // green
static const meters_t BUMPER_HIT_THICKNESS = 0.02;
static const meters_t BUMPER_NOHIT_THICKNESS = 0.01;
static const Size DEFAULT_SIZE( 0.1, 0.1, 0.1 );


Option ModelBumper::showBumperData( "Show Bumper Data", "show_bumper", "", true, NULL );

ModelBumper::ModelBumper( World* world,
			  Model* parent,
		          const std::string& type  )
  : Model( world, parent, type  )
{
  PRINT_DEBUG2( "Constructing ModelBumper %d (%s)\n",
		id, type );

  // Set up sensible defaults
  SetGeom( Geom( Pose(0,0,0,0), DEFAULT_SIZE ) );
  SetColor( Color( BUMPER_NOHIT_COLOR ) );

  // assert that Update() is reentrant for this derived model
  thread_safe = true;

  bumpers = NULL;
  samples = NULL;
  bumper_count = 0;

  RegisterOption( &showBumperData );

}

ModelBumper::~ModelBumper()
{
  if( bumpers )
	 delete[] bumpers;
  if( samples )
	 delete[] samples;
}

void ModelBumper::Startup( void )
{
  Model::Startup();

  PRINT_DEBUG( "bumper startup" );

  this->SetWatts( BUMPER_WATTS );
}


void ModelBumper::Shutdown( void )
{
  PRINT_DEBUG( "bumper shutdown" );

  this->SetWatts( 0 );

  if( this->samples )
    {
      delete[] samples;
      samples = NULL;
    }

  Model::Shutdown();
}

void ModelBumper::Load( void )
{
  if( wf->PropertyExists( wf_entity, "bcount" ) )
    {
      PRINT_DEBUG( "Loading bumper array" );

      // Load the geometry of a bumper array
      bumper_count = wf->ReadInt( wf_entity, "bcount", 0);
      assert( bumper_count > 0 );

      char key[256];

      if( bumpers ) delete [] bumpers;
      bumpers = new bumper_config_t[bumper_count];

      meters_t common_length;
      common_length = wf->ReadLength( wf_entity, "blength", 0 );

      // set all transducers with the common settings
      for( unsigned int i = 0; i < bumper_count; i++)
	{
          bumpers[i].length = common_length;
	}

      // allow individual configuration of transducers
      for( unsigned int i = 0; i < bumper_count; i++)
	{
	  snprintf(key, sizeof(key), "bpose[%d]", i);
	  bumpers[i].pose.x = wf->ReadTupleLength( wf_entity, key, 0, 0 );
	  bumpers[i].pose.y = wf->ReadTupleLength( wf_entity, key, 1, 0 );
	  bumpers[i].pose.z = wf->ReadTupleLength( wf_entity, key, 2, 0 );
	  bumpers[i].pose.a = wf->ReadTupleAngle( wf_entity, key, 3, 0);

	  snprintf(key, sizeof(key), "blength[%d]", i);
	  bumpers[i].length = wf->ReadLength( wf_entity, key, bumpers[i].length );
	}

      PRINT_DEBUG1( "loaded %d bumpers configs", (int)bumper_count );
    }

  Model::Load();

}

static bool bumper_match( Model* candidate,
			  Model* finder, const void* dummy )
{
  // Ignore myself, my children, and my ancestors.
  return( candidate->vis.obstacle_return &&
	  !candidate->IsRelated( finder ) );
}

void ModelBumper::Update( void )
{
  Model::Update();

  if( (bumpers == NULL) || (bumper_count < 1 ))
    return;

  if( samples ==  NULL )
    samples = new bumper_sample_t[bumper_count];
  assert( samples );

  for( unsigned int t=0; t<bumper_count; t++ )
    {
      // change the pose of bumper to act as a sensor rotated of PI/2, positioned at
      // an extremity of the bumper, and with a range of "length"
      Pose bpose;
      bpose.a = bumpers[t].pose.a + M_PI/2.0;
      bpose.x = bumpers[t].pose.x - bumpers[t].length/2.0 * cos(bpose.a);
      bpose.y = bumpers[t].pose.y - bumpers[t].length/2.0 * sin(bpose.a);

      RaytraceResult ray = Raytrace( bpose,
					    bumpers[t].length,
					    bumper_match,
					    NULL );
      samples[t].hit = ray.mod;
      if (ray.mod) {
        samples[t].hit_point = point_t(ray.pose.x, ray.pose.y);
      }
    }
}

void ModelBumper::Print( char* prefix )
{
  Model::Print( prefix );

  printf( "\tBumpers[ " );

  for( unsigned int i=0; i<bumper_count; i++ )
    printf( "%d ", samples[i].hit?1:0 );
  puts( " ]" );
}

void ModelBumper::DataVisualize( Camera* cam )
{

  double thickness;
  if( ! (samples && bumpers && bumper_count) )
    return;

  if (!showBumperData )
    return;

  for( unsigned int t=0; t<bumper_count; t++ )
    {
      glPushMatrix();
      if (samples[t].hit) {
        thickness = 0.02;
        glColor3f(1.0f, 0.0f, 0.0f);
      } else {
        thickness = 0.01;
        glColor3f(0.0f, 1.0f, 0.0f);
      }
      glRotatef(bumpers[t].pose.a, 0, 0, 1);
      glTranslatef(bumpers[t].pose.x, bumpers[t].pose.y,0);
      glRectf(-bumpers[t].length/2.0, -thickness/2.0,
              bumpers[t].length/2.0, thickness/2.0);
      glPopMatrix();
    }

}
