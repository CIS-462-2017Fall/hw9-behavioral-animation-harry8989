#include "aBehaviorController.h"

#include "aVector.h"
#include "aRotation.h"
#include <Windows.h>
#include <algorithm>

#include "GL/glew.h"
#include "GL/glut.h"



#define Truncate(a, b, c) (a = max<double>(min<double>(a,c),b))

double BehaviorController::gMaxSpeed = 1000.0; 
double BehaviorController::gMaxAngularSpeed = 200.0;  
double BehaviorController::gMaxForce = 2000.0;  
double BehaviorController::gMaxTorque = 2000.0;
double BehaviorController::gKNeighborhood = 500.0;   
//double BehaviorController::gOriKv = 1.0;    
//double BehaviorController::gOriKp = 1.0;  
//double BehaviorController::gVelKv = 1.0;
double BehaviorController::gOriKv = 32.0;
double BehaviorController::gOriKp = 256.0;
double BehaviorController::gVelKv = 10.0;
double BehaviorController::gAgentRadius = 80.0;  
double BehaviorController::gMass = 1;
double BehaviorController::gInertia = 1;
double BehaviorController::KArrival = 1.0; 
double BehaviorController::KDeparture = 12000.0;
double BehaviorController::KNoise = 15.0;
double BehaviorController::KWander = 80.0;   
double BehaviorController::KAvoid = 600.0;  
double BehaviorController::TAvoid = 1000.0;   
double BehaviorController::KSeparation = 12000.0; 
double BehaviorController::KAlignment = 1.0;  
double BehaviorController::KCohesion = 1.0;  

const double M2_PI = M_PI * 2.0;

BehaviorController::BehaviorController() 
{
	m_state.resize(m_stateDim);
	m_stateDot.resize(m_stateDim);
	m_controlInput.resize(m_controlDim);

	vec3 m_Pos0 = vec3(0, 0, 0);
	vec3 m_Vel0 = vec3(0, 0, 0);
	vec3 m_lastVel0 = vec3(0, 0, 0);
	vec3 m_Euler = vec3(0, 0, 0);
	vec3 m_VelB = vec3(0, 0, 0);
	vec3 m_AVelB = vec3(0, 0, 0);
	
	m_Vdesired = vec3(0, 0, 0);
	m_lastThetad = 0.0;

	m_Active = true; 
	mpActiveBehavior = NULL;
	mLeader = false;

	reset();
}

AActor* BehaviorController::getActor()
{
	return m_pActor;
}

void BehaviorController::setActor(AActor* actor)

{
	m_pActor = actor;
	m_pSkeleton = m_pActor->getSkeleton();

}


void BehaviorController::createBehaviors(vector<AActor>& agentList, vector<Obstacle>& obstacleList)
{
	
	m_AgentList = &agentList;
	m_ObstacleList = &obstacleList;

	m_BehaviorList.clear();
	m_BehaviorList[SEEK] = new Seek(m_pBehaviorTarget);
	m_BehaviorList[FLEE] = new Flee(m_pBehaviorTarget);
	m_BehaviorList[ARRIVAL] = new Arrival(m_pBehaviorTarget);
	m_BehaviorList[DEPARTURE] = new Departure(m_pBehaviorTarget);
	m_BehaviorList[WANDER] = new Wander();
	m_BehaviorList[COHESION] = new Cohesion(m_AgentList);
	m_BehaviorList[ALIGNMENT] = new Alignment(m_pBehaviorTarget, m_AgentList);
	m_BehaviorList[SEPARATION] = new Separation(m_pBehaviorTarget, m_AgentList);
	m_BehaviorList[LEADER] = new Leader(m_pBehaviorTarget, m_AgentList);
	m_BehaviorList[FLOCKING] = new Flocking(m_pBehaviorTarget, m_AgentList);
	m_BehaviorList[AVOID] = new Avoid(m_pBehaviorTarget, m_ObstacleList);
}

BehaviorController::~BehaviorController()
{
	mpActiveBehavior = NULL;
}

void BehaviorController::reset()
{
	vec3 startPos;
	startPos[0] = ((double)rand()) / RAND_MAX;
	startPos[1] =  ((double)rand()) / RAND_MAX, 
	startPos[2] = ((double)rand()) / RAND_MAX;
	startPos = startPos - vec3(0.5, 0.5, 0.5);

	startPos[1] = 0; // set equal to zero for 2D case (assume y is up)

	m_Guide.setLocalTranslation(startPos * 500.0);
	
	for (int i = 0; i < m_stateDim; i++)
	{
		m_state[i] = 0.0;
		m_stateDot[i] = 0.0;
	}

	m_force = 0.0;
	m_torque = 0.0;
	m_thetad = 0.0;
	m_vd = 0.0;
}

///////////////////////////////////////////////////

inline void ClampAngle(double& angle)
{
	while (angle > M_PI)
	{
		angle -= M2_PI;
	}
	while (angle < -M_PI)
	{
		angle += M2_PI;
	}
}

void BehaviorController::sense(double deltaT)
{
	if (mpActiveBehavior)
	{
		// find the agents in the neighborhood of the current character.
	}
	
}

void BehaviorController::control(double deltaT)
// Given the active behavior this function calculates a desired velocity vector (Vdesired).  
// The desired velocity vector is then used to compute the desired speed (vd) and direction (thetad) commands

{

	if (mpActiveBehavior)
	{ 
		m_Vdesired = mpActiveBehavior->calcDesiredVel(this);
		m_Vdesired[1] = 0;

		//  force and torque inputs are computed from vd and thetad as follows:
		//              Velocity P controller : force = mass * Kv * (vd - v)
		//              Heading PD controller : torque = Inertia * (-Kv * thetaDot + Kp * (thetad - theta))
		//  where the values of the gains Kv and Kp are different for each controller

		// TODO: insert your code here to compute m_force and m_torque

		//Calculate m_vd
		m_vd = m_Vdesired.Length();

		//The Kv , Kv, and Kp have already been changed in the place from which they are drawn
		//They were based on calculations done elsewhere

		////Only the Z section matters for force
		m_force = vec3(0.0, 0.0, 0.0);
		m_force[2] = gMass*gVelKv*(m_vd - m_state[VEL][2]);

		//Want angle between m_Vdesired and world x axis
		
		vec3 unitxAxis = vec3(1.0, 0.0, 0.0);
		vec3 unitVdesired = m_Vdesired / (m_Vdesired.Length());
		double thetad;

		thetad = atan2(m_Vdesired[2], m_Vdesired[0]);
		ClampAngle(thetad);
		m_thetad = thetad;
		double needsclamping = m_thetad - m_Euler[1];
		ClampAngle(needsclamping);

		//Only the Y section matters for torque
		m_torque = vec3(0.0, 0.0, 0.0);
		m_torque[1] = gInertia * (-gOriKv * m_AVelB[1] + gOriKp * (needsclamping));

		//Apply max limits
		float forcemag = sqrt(m_force[0] * m_force[0] + m_force[1] * m_force[1] + m_force[2] * m_force[2]);
		if (gMaxForce < forcemag) {
			m_force = (m_force / forcemag)*gMaxForce;//Convert to unit vector, apply new magnitude
		}
		float torquemag = sqrt(m_torque[0] * m_torque[0] + m_torque[1] * m_torque[1] + m_torque[2] * m_torque[2]);
		if (gMaxTorque < torquemag) {
			m_torque = (m_torque / torquemag)*gMaxTorque;//Convert to unit vector, apply new magnitude
		}

		









		// when agent desired agent velocity and actual velocity < 2.0 then stop moving
		if (m_vd < 2.0 &&  m_state[VEL][_Z] < 2.0)
		{
			m_force[2] = 0.0;
			m_torque[1] = 0.0;
		}
	}
	else
	{
		m_force[2] = 0.0;
		m_torque[1] = 0.0;
	}

	// set control inputs to current force and torque values
	m_controlInput[0] = m_force;
	m_controlInput[1] = m_torque;
}

void BehaviorController::act(double deltaT)
{
	computeDynamics(m_state, m_controlInput, m_stateDot, deltaT);
	
	int EULER = 0;
	int RK2 = 1;
	updateState(deltaT, EULER);
}


void BehaviorController::computeDynamics(vector<vec3>& state, vector<vec3>& controlInput, vector<vec3>& stateDot, double deltaT)
// Compute stateDot vector given the control input and state vectors
//  This function sets derive vector to appropriate values after being called
{
	vec3& force = controlInput[0];
	vec3& torque = controlInput[1];

	// Compute the stateDot vector given the values of the current state vector and control input vector
	// TODO: add your code here


	//Looking at the aBehaviorController.h gives me ideas for most if not all
	stateDot[0][0] = state[2][2] * cos(state[1][1]);//+ state[2][0] * cos(state[1][1]);
	stateDot[0][1] = 0.0;
	stateDot[0][2] = state[2][2] * sin(state[1][1]);//- state[2][0] * sin(state[1][1]);
	stateDot[1] = state[3];
	stateDot[2] = force / gMass;
	stateDot[3] = torque / gInertia;

	m_Vel0 = stateDot[0];






}

void BehaviorController::updateState(float deltaT, int integratorType)
{
	//  Update the state vector given the m_stateDot vector using Euler (integratorType = 0) or RK2 (integratorType = 1) integratio
	//  this should be similar to what you implemented in the particle system assignment

	// TODO: add your code here
	
	//Basically copied from my aParticle.cpp version
	//It's RK2
	computeDynamics(m_state, m_controlInput, m_stateDot, deltaT);

	//TODO:  Add your code here to update the state using EULER and Runge Kutta2  integration
	if (integratorType == 0) {
		for (int i = 0; i <= 3; i++)
		{
			m_state[i] = m_state[i] + deltaT*m_stateDot[i];
		}
	}
	else //RK2 default
	{

		vector<vec3> predictedstate;
		vector<vec3> predictedstateDot;
		for (int j = 0; j <= 3; j++)
		{
			predictedstate[j] = m_state[j] + deltaT*m_stateDot[j];
		}

		computeDynamics(predictedstate, m_controlInput, predictedstateDot, deltaT);//

		for (int k = 0; k <= 3; k++)
		{
			predictedstate[k] = m_state[k] + 0.5*deltaT*(m_stateDot[k] + predictedstateDot[k]);
		}

	}






	//  given the new values in m_state, these are the new component state values 
	m_Pos0 = m_state[POS];
	m_Euler = m_state[ORI];
	m_VelB = m_state[VEL];
	m_AVelB = m_state[AVEL];

	//  Perform validation check to make sure all values are within MAX values
	// TODO: add your code here

	float speed = sqrt(m_VelB[0] * m_VelB[0] + m_VelB[1] * m_VelB[1] + m_VelB[2] * m_VelB[2]);
	float angspeed = sqrt(m_AVelB[0] * m_AVelB[0] + m_AVelB[1] * m_AVelB[1] + m_AVelB[2] * m_AVelB[2]);
	if (gMaxSpeed < speed) {
		m_VelB = (m_VelB / speed)*gMaxSpeed;//Convert to unit vector, apply new magnitude
	}
	if (gMaxAngularSpeed < angspeed) {
		m_AVelB = (m_AVelB / angspeed)*gMaxAngularSpeed;//Convert to unit vector, apply new magnitude
	}

	//For Vel0
	float zerospeed = sqrt(m_Vel0[0] * m_Vel0[0] + m_Vel0[1] * m_Vel0[1] + m_Vel0[2] * m_Vel0[2]);
	if (gMaxSpeed < zerospeed) {
		m_Vel0 = (m_Vel0 / zerospeed)*gMaxSpeed;//Convert to unit vector, apply new magnitude
	}





	// update the guide orientation
	// compute direction from nonzero velocity vector
	vec3 dir;
	if (m_Vel0.Length() < 1.0)
	{
		dir = m_lastVel0;
		dir.Normalize();;
		m_state[ORI] = atan2(dir[_Z], dir[_X]);
	}
	else
	{
		dir = m_Vel0;
		m_lastVel0 = m_Vel0;
	}

	dir.Normalize();
	vec3 up(0.0, 1.0, 0.0);
	vec3 right = up.Cross(dir);
	right.Normalize();
	mat3 rot(right, up, dir);
	m_Guide.setLocalRotation(rot.Transpose());
	m_Guide.setLocalTranslation(m_Guide.getLocalTranslation() + m_Vel0*deltaT);

}


void BehaviorController::setTarget(AJoint& target)
{
	m_pBehaviorTarget = &target;
	for (unsigned int i = 0; i < m_BehaviorList.size(); i++)
	{
		BehaviorType index = (BehaviorType) i;
		m_BehaviorList[index]->setTarget(m_pBehaviorTarget);
	}



}

void BehaviorController::setActiveBehavior(Behavior* pBehavior)
{
	mpActiveBehavior = pBehavior;
}

void BehaviorController::setActiveBehaviorType(BehaviorType type)
{
	m_BehaviorType = type;
	Behavior* pActiveBehavior = m_BehaviorList[type];
	setActiveBehavior(pActiveBehavior);

}

void BehaviorController::display()
{ // helps with debugging behaviors.  red line is actual velocity vector, green line is desired velocity vector
	
	vec3 pos = getPosition();
	double scale = 1.0;
	vec3 vel = scale* getVelocity();
	double velMag = vel.Length();
	vec3 dvel = scale* getDesiredVelocity();
	vec3 angle = getOrientation() * (180.0 / 3.14159);

	glBegin(GL_LINES);
	glColor3f(1, 0, 0);
	glVertex3f(pos[0], pos[1], pos[2]);
	glVertex3f(pos[0] + vel[0], pos[1] + vel[1], pos[2] + vel[2]);
	glColor3f(0, 1, 0);
	glVertex3f(pos[0], pos[1], pos[2]);
	glVertex3f(pos[0] + dvel[0], pos[1] + dvel[1], pos[2] + dvel[2]);
	glEnd();

	if (this->isLeader())
		glColor3f(0, 0, 1);
	else glColor3f(0.5, 0, 0);

	glPushMatrix();
	glTranslatef(pos[0], pos[1], pos[2]);
	glRotatef(90 - angle[1], 0, 1, 0);
	glutSolidCone(40, 80, 10, 10);
	glutSolidSphere(35, 10, 10);
	glPopMatrix();

	BehaviorType active = getActiveBehaviorType();
	Behavior* pBehavior = m_BehaviorList[active];
	pBehavior->display(this);

}

