#include "aBehaviors.h"

#include <math.h>
#include "GL/glew.h"
#include "GL/glut.h"

// Base Behavior
///////////////////////////////////////////////////////////////////////////////
Behavior::Behavior()
{
}

Behavior::Behavior( char* name) 
{
	m_name = name;
	m_pTarget = NULL;
}

Behavior::Behavior( Behavior& orig) 
{
	m_name = orig.m_name;
	m_pTarget = NULL;
}

string& Behavior::GetName() 
{
    return m_name;
}

// Behaviors derived from Behavior
//----------------------------------------------------------------------------//
// Seek behavior
///////////////////////////////////////////////////////////////////////////////
// For the given the actor, return a desired velocity in world coordinates
// Seek returns a maximum velocity towards the target
// m_pTarget contains target world position
// actor.getPosition() returns Agent's world position


Seek::Seek( AJoint* target) 
{
	m_name = "seek";
	m_pTarget = target;

}

Seek::Seek( Seek& orig) 
{
	m_name = "seek";
	m_pTarget = orig.m_pTarget;
}


Seek::~Seek()
{
}

vec3 Seek::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 targetPos = m_pTarget->getLocalTranslation();
	vec3 actorPos = actor->getPosition();

	// TODO: add your code here to compute Vdesired
	Vdesired = ((targetPos - actorPos) / ((targetPos - actorPos).Length()))*actor->gMaxSpeed;


	return Vdesired;
}


// Flee behavior
///////////////////////////////////////////////////////////////////////////////
// For the given the actor, return a desired velocity in world coordinates
// Flee calculates a a maximum velocity away from the target
// m_pTarget contains target world position
// actor.getPosition() returns Agent's world position

Flee::Flee( AJoint* target) 
{
	m_name = "flee";
	m_pTarget = target;
}

Flee::Flee( Flee& orig) 
{
	m_name = "flee";
	m_pTarget = orig.m_pTarget;
}

Flee::~Flee()
{
}

vec3 Flee::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 targetPos = m_pTarget->getLocalTranslation();
	vec3 actorPos = actor->getPosition();

	// TODO: add your code here to compute Vdesired
	Vdesired = -1*((targetPos - actorPos) / ((targetPos - actorPos).Length()))*actor->gMaxSpeed;



	return Vdesired;

}

// Arrival behavior
///////////////////////////////////////////////////////////////////////////////
// Given the actor, return a desired velocity in world coordinates
// Arrival returns a desired velocity vector whose speed is proportional to
// the actors distance from the target
// m_pTarget contains target world position
// actor.getPosition() returns Agent's world position
//  Arrival strength is in BehavioralController::KArrival


Arrival::Arrival( AJoint* target) 
{
	m_name = "arrival";
	m_pTarget = target;
}

Arrival::Arrival( Arrival& orig) 
{
	m_name = "arrival";
	m_pTarget = orig.m_pTarget;
}

Arrival::~Arrival()
{
}

vec3 Arrival::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 targetPos = m_pTarget->getLocalTranslation();
	vec3 actorPos = actor->getPosition();

	// TODO: add your code here to compute Vdesired

	Vdesired = (targetPos - actorPos)*actor->KArrival;



	return Vdesired;
}


// Departure behavior
///////////////////////////////////////////////////////////////////////////////
// Given the actor, return a desired velocity in world coordinates
// Arrival returns a desired velocity vector whose speed is proportional to
// 1/(actor distance) from the target
// m_pTarget contains target world position
// actor.getPosition() returns Agent's world position
//  Departure strength is in BehavioralController::KDeparture

Departure::Departure(AJoint* target) 
{
	m_name = "departure";
	m_pTarget = target;
}

Departure::Departure( Departure& orig) 
{
	m_name = "departure";
	m_pTarget = orig.m_pTarget;
}

Departure::~Departure()
{
}

vec3 Departure::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 targetPos = m_pTarget->getLocalTranslation();
	vec3 actorPos = actor->getPosition();

	// TODO: add your code here to compute Vdesired


	Vdesired = (actorPos- targetPos)*actor->KDeparture/(pow((actorPos - targetPos).Length(),2.0));
	//Vdesired = -1 * (targetPos - actorPos)*actor->KArrival / ((targetPos - actorPos).Length());



	return Vdesired;
}


// Avoid behavior
///////////////////////////////////////////////////////////////////////////////
//  For the given the actor, return a desired velocity in world coordinates
//  If an actor is near an obstacle, avoid adds a normal response velocity to the 
//  the desired velocity vector computed using arrival
//  Agent bounding sphere radius is in BehavioralController::radius
//  Avoidance parameters are  BehavioralController::TAvoid and BehavioralController::KAvoid

Avoid::Avoid(AJoint* target, vector<Obstacle>* obstacles) 
{
	m_name = "avoid";
	m_pTarget = target;
	mObstacles = obstacles;
}

Avoid::Avoid( Avoid& orig) 
{
	m_name = "avoid";
	m_pTarget = orig.m_pTarget;
	mObstacles = orig.mObstacles;
}

Avoid::~Avoid()
{
}

vec3 Avoid::calcDesiredVel( BehaviorController* actor)
{

	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	m_actorPos = actor->getPosition();
	m_actorVel = actor->getVelocity();

	//TODO: add your code here
	vec3 Varrival(0, 0, 0);
	// Step 1. compute initial value for Vdesired = Varrival so agent moves toward target
	vec3 targetPos = m_pTarget->getLocalTranslation();
	vec3 actorPos = actor->getPosition();
	Varrival = (targetPos - actorPos)*actor->KArrival;



	vec3 Vavoid(0, 0, 0);
	//TODO: add your code here to compute Vavoid 



	// Step 2. compute Lb
	//TODO: add your code here
	float Lb = actor->TAvoid*Varrival.Length();


	// Step 3. find closest obstacle 
	//TODO: add your code here
	float mindist;
	int minindex;
	for (int i = 0; i < (*mObstacles).size(); i++) {
		float distance = ((*mObstacles)[i].m_Center.getLocalTranslation() - m_actorPos).Length();
		if (i < 1){
			mindist = distance;
			minindex = 0;
		}
		else {
			if (distance < mindist) {
				mindist = distance;
				minindex = i;
			}
		}
	}



	// Step 4. determine whether agent will collide with closest obstacle (only consider obstacles in front of agent)
	//TODO: add your code here
	if ((*mObstacles).size() > 0) {
		//mat3 tobody;
		//tobody = tobody.FromEulerAngles(mat3::ZYX, actor->getOrientation());
		//float xdist = (tobody*((*mObstacles)[minindex].m_Center.getLocalTranslation() - m_actorPos))[0];
		//float zdist = (tobody*((*mObstacles)[minindex].m_Center.getLocalTranslation() - m_actorPos))[2];
		//bool colliding = (xdist < (Lb + (*mObstacles)[minindex].m_Radius + 1.0)) && (zdist < (Lb + (*mObstacles)[minindex].m_Radius + 1.0));
		//mat3 inverseme = actor->getGuide().getLocalRotation();
		//inverseme = inverseme.Inverse();
		//float xdist = (inverseme*((*mObstacles)[minindex].m_Center.getLocalTranslation() - m_actorPos))[0];
		//float zdist = (inverseme*((*mObstacles)[minindex].m_Center.getLocalTranslation() - m_actorPos))[2];

		// Step 5.  if potential collision detected, compute Vavoid and set Vdesired = Varrival + Vavoid
		//TODO: add your code here
		//The direction of dx is the same as the direction of the velocity
		//Therefore, they are the same as unit vectors
		vec3 arr = Lb*(m_actorVel / (m_actorVel.Length())) - ((*mObstacles)[minindex].m_Center.getLocalTranslation() - m_actorPos);
		vec3 rhatavoid = arr/(arr.Length());
		float vmag = (actor->KAvoid*(((*mObstacles)[minindex].m_Radius + actor->gAgentRadius) - (arr.Length()))) / ((*mObstacles)[minindex].m_Radius + actor->gAgentRadius);

		if ((arr.Length())<=((*mObstacles)[minindex].m_Radius + actor->gAgentRadius)) {//Something's preventing this from activating, most likely an earlier error

		//if ((((*mObstacles)[minindex].m_Center.getLocalTranslation() - m_actorPos).Length()) <= (Lb+(*mObstacles)[minindex].m_Radius + actor->gAgentRadius)) {
			Vavoid = rhatavoid*vmag;
		}
	}


	Vdesired = Varrival + Vavoid;
	return Vdesired;
	
}

void Avoid::display( BehaviorController* actor)
{
	//  Draw Debug info
	vec3 angle = actor->getOrientation();
	vec3 vel = actor->getVelocity();
	vec3 dir = vec3(cos(angle[1]), 0, sin(angle[1]));
	vec3 probe = dir * (vel.Length()/BehaviorController::gMaxSpeed)*BehaviorController::TAvoid;
	
	glBegin(GL_LINES);
	glColor3f(0, 0, 1);
	glVertex3f(m_actorPos[0], m_actorPos[1], m_actorPos[2]);
	glVertex3f(m_obstaclePos[0], m_obstaclePos[1], m_obstaclePos[2]);
	glColor3f(0, 1, 1);
	glVertex3f(m_actorPos[0], m_actorPos[1], m_actorPos[2]);
	glVertex3f(m_actorPos[0] + probe[0], m_actorPos[1] + probe[1], m_actorPos[2] + probe[2]);
	glEnd();
}


// Wander Behavior
///////////////////////////////////////////////////////////////////////////////
// For the given the actor, return a desired velocity in world coordinates
// Wander returns a desired velocity vector whose direction changes at randomly from frame to frame
// Wander strength is in BehavioralController::KWander

Wander::Wander() 
{
	m_name = "wander";
	m_Wander = vec3(1.0, 0.0, 0.0);
}

Wander::Wander( Wander& orig) 
{
	m_name = "wander";
	m_Wander = orig.m_Wander;
}

Wander::~Wander()
{
}

vec3 Wander::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 actorPos = actor->getPosition();

	// compute Vdesired = Vwander

	// Step. 1 find a random direction
	//TODO: add your code here

	double randval = rand() % 361;//Random number between 0 and 360
	vec3 randdirec = vec3(cos(randval*M_PI / 180.0), 0, sin(randval*M_PI / 180.0));
	randdirec = randdirec.Normalize();

	// Step2. scale it with a noise factor
	//TODO: add your code here
	randdirec = randdirec*actor->KNoise;



	// Step3. change the current Vwander  to point to a random direction
	//TODO: add your code here
	m_Wander=actor->KWander*((m_Wander+randdirec).Normalize());



	// Step4. scale the new wander velocity vector and add it to the nominal velocity
	//TODO: add your code here
	Vdesired = m_Wander + vec3(sqrt(2), 0, sqrt(2));//Nominal velocity here just an arbitrary vector




	return Vdesired;
}


// Alignment behavior
///////////////////////////////////////////////////////////////////////////////
// For the given the actor, return a desired velocity vector in world coordinates
// Alignment returns the average velocity of all active agents in the neighborhood
// agents[i] gives the pointer to the ith agent in the environment
// Alignment parameters are in BehavioralController::RNeighborhood and BehavioralController::KAlign


Alignment::Alignment(AJoint* target, vector<AActor>* agents) 
{
	m_name = "alignment";
	m_pAgentList = agents;
	m_pTarget = target;
}



Alignment::Alignment( Alignment& orig) 
{
	m_name = orig.m_name;
	m_pAgentList = orig.m_pAgentList;
	m_pTarget = orig.m_pTarget;

}

Alignment::~Alignment()
{
}

vec3 Alignment::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 targetPos = m_pTarget->getLocalTranslation();
	vec3 actorPos = actor->getPosition();
	vector<AActor>& agentList = *m_pAgentList;
	

	// compute Vdesired 
	
	// Step 1. compute value of Vdesired for fist agent (i.e. m_AgentList[0]) using an arrival behavior so it moves towards the target
	 
	BehaviorController* leader = agentList[0].getBehaviorController(); // first agent is the leader
	//TODO: add your code here
	double numthings = 0.0;

	if (actor->gKNeighborhood >= (leader->getPosition() - actorPos).Length()) {
		Vdesired = Vdesired + (targetPos - leader->getPosition())*leader->KArrival;
		if (actor == leader){
			return Vdesired;
		}
		numthings = numthings + 1;
	}


	// Step 2. if not first agent compute Valign as usual
	//TODO: add your code here
	for (int i = 1; i < agentList.size(); i++) {
		BehaviorController* other = agentList[i].getBehaviorController();
		if (actor->gKNeighborhood >= (other->getPosition() - actorPos).Length()) {
			Vdesired = Vdesired + other->getVelocity();
			numthings = numthings + 1;
		}

		}
	
	if (agentList.size() > 0) {
		Vdesired = actor->KAlignment*Vdesired / numthings;
	}
	
	return Vdesired;
}

// Separation behavior
///////////////////////////////////////////////////////////////////////////////
// For the given te actor, return a desired velocity vector in world coordinates
// Separation tries to maintain a constant distance between all agents
// within the neighborhood
// agents[i] gives the pointer to the ith agent in the environment
// Separation settings are in BehavioralController::RNeighborhood and BehavioralController::KSeperate

 

Separation::Separation( AJoint* target,  vector<AActor>* agents) 
{
	m_name = "separation";
	m_AgentList = agents;
	m_pTarget = target;
}

Separation::~Separation()
{
}

Separation::Separation( Separation& orig) 
{
	m_name = "separation";
	m_AgentList = orig.m_AgentList;
	m_pTarget = orig.m_pTarget;
}

vec3 Separation::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 targetPos = m_pTarget->getLocalTranslation();
	vec3 actorPos = actor->getPosition();
	vector<AActor>& agentList = *m_AgentList;
	
	// compute Vdesired = Vseparate
	// TODO: add your code here to compute Vdesired 

	for (int i = 0; i < agentList.size(); i++) {
		BehaviorController* other = agentList[i].getBehaviorController();
		if (other != actor) {
			vec3 onedist = (actorPos-other->getPosition());
			if (actor->gKNeighborhood >= onedist.Length()) {
				Vdesired = Vdesired + actor->KSeparation*onedist / (onedist.SqrLength());
			}
		}

	}


	//The following code was already here:
	//if (Vdesired.Length() < 5.0){}
	//	Vdesired = 0.0;
	//I commented it out

	
	return Vdesired;
}


// Cohesion behavior
///////////////////////////////////////////////////////////////////////////////
// For the given actor, return a desired velocity vector in world coordinates
// Cohesion moves actors towards the center of the group of agents in the neighborhood
//  agents[i] gives the pointer to the ith agent in the environment
//  Cohesion parameters are in BehavioralController::RNeighborhood and BehavioralController::KCohesion


Cohesion::Cohesion( vector<AActor>* agents) 
{
	m_name = "cohesion";
	m_AgentList = agents;
}

Cohesion::Cohesion( Cohesion& orig) 
{
	m_name = "cohesion";
	m_AgentList = orig.m_AgentList;
}

Cohesion::~Cohesion()
{
}

vec3 Cohesion::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 actorPos = actor->getPosition();
	vector<AActor>& agentList = *m_AgentList;
	
	// compute Vdesired = Vcohesion
	// TODO: add your code here 

	vec3 centerofmass = vec3(0.0, 0.0, 0.0);
	int neighbors = 0;
	for (int i = 0; i < agentList.size(); i++) {
		BehaviorController* other = agentList[i].getBehaviorController();
		if (actor->gKNeighborhood >= (actorPos - other->getPosition()).Length()) {
			neighbors = neighbors + 1;
			centerofmass = centerofmass + other->getPosition();
		}
	}
	if (agentList.size() > 0) {
		centerofmass = centerofmass / neighbors;//Average postion=center of mass
		Vdesired = Vdesired + actor->KCohesion*(centerofmass- actorPos);
	}




	return Vdesired;
}

// Flocking behavior
///////////////////////////////////////////////////////////////////////////////
// For the given actor, return a desired velocity vector  in world coordinates
// Flocking combines separation, cohesion, and alignment behaviors
//  Utilize the Separation, Cohesion and Alignment behaviors to determine the desired velocity vector


Flocking::Flocking( AJoint* target,  vector<AActor>* agents) 
{
	m_name = "flocking";
	m_AgentList = agents;
	m_pTarget = target;
}

Flocking::Flocking( Flocking& orig) 
{
	m_name = "flocking";
	m_AgentList = orig.m_AgentList;
	m_pTarget = orig.m_pTarget;
}

Flocking::~Flocking()
{
}

vec3 Flocking::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 actorPos = actor->getPosition();
	vector<AActor>& agentList = *m_AgentList;

	// compute Vdesired = Vflocking
	// TODO: add your code here 
	//Behavior* cohesionbase=new Cohesion(m_AgentList);
	//vec3 Vcohesion = cohesionbase->calcDesiredVel(actor);

	//Behavior* arrivalbase = new Alignment(m_pBehaviorTarget, m_AgentList);
	//vec3 Vcohesion = arrivalbase->calcDesiredVel(actor);

	//Behavior* seperatebase = new Cohesion(m_AgentList);
	//vec3 Vcohesion = seperatebase->calcDesiredVel(actor);

	vec3 Vcohesion = vec3(0.0, 0.0, 0.0);
	vec3 centerofmass = vec3(0.0, 0.0, 0.0);
	int neighbors = 0;
	for (int i = 0; i < agentList.size(); i++) {
		BehaviorController* other = agentList[i].getBehaviorController();
		if (actor->gKNeighborhood >= (actorPos - other->getPosition()).Length()) {
			neighbors = neighbors + 1;
			centerofmass = centerofmass + other->getPosition();
		}
	}
	if (agentList.size() > 0) {
		centerofmass = centerofmass / neighbors;//Average postion=center of mass
		Vcohesion = Vcohesion + actor->KCohesion*(centerofmass - actorPos);
	}

	vec3 Vseperate = vec3(0.0, 0.0, 0.0);
	for (int i = 0; i < agentList.size(); i++) {
		BehaviorController* other = agentList[i].getBehaviorController();
		if (other != actor) {
			vec3 onedist = (actorPos - other->getPosition());
			if (actor->gKNeighborhood >= onedist.Length()) {
				Vseperate = Vseperate + actor->KSeparation*onedist / (onedist.SqrLength());
			}
		}
	}

	vec3 targetPos = m_pTarget->getLocalTranslation();
	vec3 Valignment = vec3(0.0, 0.0, 0.0);
	BehaviorController* leader = agentList[0].getBehaviorController();
	double numthings = 0.0;
	if (actor->gKNeighborhood >= (leader->getPosition() - actorPos).Length()) {
		//Valignment = Valignment + (targetPos - leader->getPosition())*leader->KArrival;
		//numthings = numthings + 1;
	}
	for (int i = 1; i < agentList.size(); i++) {
		BehaviorController* other = agentList[i].getBehaviorController();
		if (actor->gKNeighborhood >= (other->getPosition() - actorPos).Length()) {
			Valignment = Valignment + other->getVelocity();
			numthings = numthings + 1;
		}

	}
	if (agentList.size() > 0) {
		Valignment = actor->KAlignment*Valignment / numthings;
		if (actor == leader) {
			Valignment = (targetPos - leader->getPosition())*leader->KArrival;
		}
	}

	//Actual new bit
	Vdesired = 5.5*Vcohesion + 2.5*Vseperate + 1.0*Valignment;

	return Vdesired;
}

//	Leader behavior
///////////////////////////////////////////////////////////////////////////////
// For the given actor, return a desired velocity vector in world coordinates
// If the agent is the leader, move towards the target; otherwise, 
// follow the leader at a set distance behind the leader without getting to close together
//  Utilize Separation and Arrival behaviors to determine the desired velocity vector
//  You need to find the leader, who is always agents[0]

Leader::Leader( AJoint* target, vector<AActor>* agents) 
{
	m_name = "leader";
	m_AgentList = agents;
	m_pTarget = target;
}

Leader::Leader( Leader& orig) 
{
	m_name = "leader";
	m_AgentList = orig.m_AgentList;
	m_pTarget = orig.m_pTarget;
}

Leader::~Leader()
{
}

vec3 Leader::calcDesiredVel( BehaviorController* actor)
{
	
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 actorPos = actor->getPosition();
	vector<AActor>& agentList = *m_AgentList;

	// TODO: compute Vdesired  = Vleader
	// followers should stay directly behind leader at a distance of -200 along the local z-axis

	float CSeparation = 4.0;  float CArrival = 2.0;

	BehaviorController* leader = agentList[0].getBehaviorController(); // first agent is the leader
	mat3 Rmat = leader->getGuide().getLocalRotation();  // is rotattion matrix of lead agent

	vec3 targetPos = m_pTarget->getLocalTranslation();
	if (actor == leader) {
		return (targetPos - leader->getPosition())*leader->KArrival;
	}

	vec3 Followertarget = leader->getPosition() - Rmat*vec3(0.0, 0.0, 200);

	vec3 Vseperate = vec3(0.0, 0.0, 0.0);
	for (int i = 0; i < agentList.size(); i++) {
		BehaviorController* other = agentList[i].getBehaviorController();
		if (other != actor) {
			vec3 onedist = (actorPos - other->getPosition());
			if (actor->gKNeighborhood >= onedist.Length()) {
				if (((targetPos - leader->getPosition())*leader->KArrival).Length() >3 ){//Partially prevents weird freakouts when at rest
					Vdesired = Vdesired + CSeparation*actor->KSeparation*onedist / (onedist.SqrLength());
				}
			}
		}
	}
	if (((targetPos - leader->getPosition())*leader->KArrival).Length() > 3) {//Partially prevents weird freakouts when at rest
		Vdesired = Vdesired + CArrival*(Followertarget - actorPos)*leader->KArrival;
	}




	return Vdesired;
}

///////////////////////////////////////////////////////////////////////////////

