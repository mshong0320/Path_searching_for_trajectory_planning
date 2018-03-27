#include "assignment3_context.h"
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit/planning_scene/planning_scene.h>
#include <queue>

using namespace std;
// utility function to interpolate between two configuration
CS436Context::vertex interpolate( const CS436Context::vertex& qA, 
 const CS436Context::vertex& qB, 
 double t ){  
  CS436Context::vertex qt( qA.size(), 0.0 );
  for( std::size_t i=0; i<qt.size(); i++ )
    { qt[i] = ( 1.0 - t )*qA[i] + t*qB[i]; }
  return qt;
}

CS436Context::CS436Context( const robot_model::RobotModelConstPtr& robotmodel,
   const std::string& name, 
   const std::string& group, 
   const ros::NodeHandle& nh ) :
  planning_interface::PlanningContext( name, group ),
  robotmodel( robotmodel ){}

CS436Context::~CS436Context(){}

bool CS436Context::state_collides( const vertex& q ) const {
  // create a robot state
  moveit::core::RobotState robotstate( robotmodel );
  robotstate.setJointGroupPositions( "manipulator", q );
  if( getPlanningScene()->isStateColliding( robotstate, "manipulator", false ) )
    { return true; }
  else
    { return false; } 
}

bool CS436Context::edge_collides( const vertex& qA, 
 const vertex& qB, 
 double step )const{
  // Simple trajectory in configuration space
  for( double t=0.0; t<1.0; t+=step ){
      if( state_collides( interpolate( qA, qB, t ) ) ) { return true; }
  }
 
  return false;
}

std::vector<CS436Context::vertex> CS436Context::make_vertices( int N )const{
  std::vector<CS436Context::vertex> V( N );
  // configurations
  std::cout<<"vertices now"<<std::endl;
  //find random points for the end effector by finding random joint angles
 
srand(time(NULL));
  for(int i=0;i<N;i++)
  {
    CS436Context::vertex q; 
    do
     {
      q = CS436Context::vertex();
      for(int j=0;j<6;j++)
       {
         double angle = (4*M_PI*((double) rand()/(double)(RAND_MAX)))-(2*M_PI);
      q.push_back(angle);        
         //cout<<"q="<<q[j];
       } 
     }while(state_collides(q));//should not collide
    
    V[i]=q; //build up vector of points
   }
  
  std::cout<<"no.of vertices:"<<V.size()<<std::endl;
  //for(int i=0;i<V.size();i++)
    //{std::cout<<"V in make_vertices="<<V[i][0]<<","<<V[i][1]<<","<<V[i][2]<<","<<V[i][3]<<","<<V[i][4]<<","<<V[i][5]<<std::endl;
      //}
  return V;
}

std::vector<CS436Context::edge> CS436Context::make_edges( const std::vector<CS436Context::vertex>& V )const{
  std::vector<CS436Context::edge> E;
  // TODO
  // Find and return collision-free edges to connect vertices in V
  std::cout<<"edges now"<<std::endl;
  //for all points find edges between each that do not collide in a combinational fashion (vs permutative)
  for(int i=0;i<V.size();i++)
{
    for(int j=i+1;j<V.size();j++)
    {
      if (!edge_collides(V[i],V[j])) 
      { 
         E.push_back(edge(i,j));
      }
    }
  }
      //std::cout<<"edge:"<<E.size<<std::endl;
      //for(int i=0;i<E.size();i++){
//std::cout<<"e:"<<E[i].first<<"   "<<E[i].second<<std::endl;}
  return E;
}

CS436Context::index
CS436Context::search_accessibility
( const std::vector<CS436Context::vertex>& V, 
  const CS436Context::vertex& q )const{
  // TODO
  // Find and return the index of the accessible vertex
  // return V.size() if no vertex is accessible
  std::cout<<"accessibility now"<<std::endl;
 
  //Store the very first accesible vertex you can find
  for(int i=0;i<V.size();i++)
   {
    if(!CS436Context::edge_collides(q,V[i]))
    {
     return i;
    }
  }

  return V.size();
}

CS436Context::index 
CS436Context::search_departability
( const std::vector<CS436Context::vertex>& V, 
  const CS436Context::vertex& q )const{
  // Find and return the index of the departable vertex
  // return V.size() if no vertex is departable
  std::cout<<"departability now"<<std::endl;
  //Store the very first departible vertex you can find
  for(int i=0;i<V.size();i++)
  {
    if(!CS436Context::edge_collides(q,V[i]))
    {
      return i;
    }
  }
  return V.size();
}


CS436Context::path 
CS436Context::search_path
( const std::vector<CS436Context::vertex>& V,
  const std::vector<CS436Context::edge>& E,
  CS436Context::index idx_start,
  CS436Context::index idx_final )const{
  std::vector<CS436Context::index> path;
 // Find and return a path between the vertices idx_start and idx_final
  
 std::cout<<"search path now"<<std::endl;
 std::vector<CS436Context::act_vert> a;
 for(int i=0; i<V.size(); i++)
    {
     CS436Context::act_vert b;
     b.idx=i;
     b.parent=V.size();
     a.push_back(b);
    } 

  std::queue<CS436Context::act_vert> c;

  std::cout<<"queuing now"<<std::endl; 

  c.push(a[idx_start]);
  while(!c.empty())
  {
    CS436Context::act_vert b=c.front();
    c.pop();
    for(int i=0;i<E.size();i++) 
    {
      if(E[i].first==b.idx && a[E[i].second].parent==V.size()) 
        {
          a[E[i].second].parent=b.idx;        
          c.push(a[E[i].second]);
        }

      if(E[i].second==b.idx && a[E[i].first].parent==V.size()) 
        {
          a[E[i].first].parent=b.idx;
          c.push(a[E[i].first]);
        }
    }
 }

 CS436Context::index active=idx_final;

  while(active!=idx_start && active!=V.size()) 
   { 
    path.push_back(active);
    active=a[active].parent;
   } 
  path.push_back(idx_start);
  
 if(active==V.size())
  {
    return CS436Context::path();
  }
 std::reverse(path.begin(),path.end());
 std::cout<<"found a path"<<std::endl; 

 return path;
}


// This is the method that is called each time a plan is requested
bool CS436Context::solve( planning_interface::MotionPlanResponse &res ){
  // Create a new empty trajectory
  res.trajectory_.reset(new robot_trajectory::RobotTrajectory(robotmodel, 
       getGroupName()));
  res.trajectory_->clear();

  // copy the initial/final joints configurations to vectors qfin and qini
  // This is mainly for convenience.
  std::vector<double> qstart, qfinal;
  for( size_t i=0; i<robotmodel->getVariableCount(); i++ ){
    qfinal.push_back(request_.goal_constraints[0].joint_constraints[i].position);
    qstart.push_back(request_.start_state.joint_state.position[i]);
  }

  // start the timer
  ros::Time begin = ros::Time::now();

  // Adjust N to your need for # of nodes
  int N = 100;

  // Create a vector of collision-free vertices
  std::vector<vertex> V = make_vertices( N );

  // Find the index of the accessible vertex
  index idx_start = search_accessibility( V, qstart );
  // Find the index of the departable vertex
  index idx_final = search_departability( V, qfinal );

  // Both index must be valid (accessible and departable vertices exist)
  if( V.size() <= idx_start || V.size() <= idx_final ) { return false; }

  // Create a vector edges
  std::vector<edge> E = make_edges( V );
  
  // Find a path between the start index and final index
  path P = search_path( V, E, idx_start, idx_final );

  // end the timer
  ros::Time end = ros::Time::now();

  // The rest is to fill in the animation. You can ignore this part.
  moveit::core::RobotState robotstate( robotmodel );
  robotstate.setJointGroupPositions( "manipulator", qstart );

  for( double t=0.0; t<=1.0; t+=0.01 ){
    vertex q = interpolate( qstart, V[P[0]], t );
    robotstate.setJointGroupPositions( "manipulator", q );
    res.trajectory_->addSuffixWayPoint( robotstate, 0.01 );
  }

  for( std::size_t i=0; i<P.size()-1; i++ ){
    for( double t=0.0; t<=1.0; t+=0.01 ){
      vertex q = interpolate( V[P[i]], V[P[i+1]], t );
      robotstate.setJointGroupPositions( "manipulator", q );
      res.trajectory_->addSuffixWayPoint( robotstate, 0.01 );
    }
  }

  for( double t=0.0; t<=1.0; t+=0.01 ){
    vertex q = interpolate( V[P[P.size()-1]], qfinal, t );
    robotstate.setJointGroupPositions( "manipulator", q );
    res.trajectory_->addSuffixWayPoint( robotstate, 0.01 );
  }

  // set the planning time
  ros::Duration duration = end-begin;
  res.planning_time_ = duration.toSec();
  res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

  return true;
}

bool CS436Context::solve( planning_interface::MotionPlanDetailedResponse &res )
{ return true; }

void CS436Context::clear(){}

bool CS436Context::terminate(){return true;}
