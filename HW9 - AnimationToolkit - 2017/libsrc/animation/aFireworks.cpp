// ParticleSystem.cpp: implementation of the AFireworks class.
//
//////////////////////////////////////////////////////////////////////

#include "aFireworks.h"
#include <stdlib.h>
#include <math.h>
#include <iostream>


#ifndef RAD
#define PI 3.14159265358979f
#define RAD (PI / 180.0f)
#endif
#ifndef GRAVITY
#define GRAVITY 9.8f
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

AFireworks::AFireworks()
{
    m_deltaT = 0.033f;
	m_rocketMass = 50.0;
	m_sparkMass = 1.0;

	m_attractorPos = vec3(0.0, 500.0, 0.0);
	m_repellerPos = vec3(0.0, 500.0, 0.0);
	m_windForce = vec3(250.0, 0.0, 0.0);

}

AFireworks::~AFireworks()
{

	for (unsigned int i = 0; i < sparks.size(); i++)
	{
		ASpark* pSpark = sparks[i];
		delete pSpark;
	}
	for (unsigned int i = 0; i < rockets.size(); i++)
	{
		ARocket* pRocket = rockets[i];
		delete pRocket;
	}

	rockets.clear();
	sparks.clear();

}

void AFireworks::display()
{

	for (unsigned int i = 0; i < sparks.size(); i++)
	{
		ASpark* pSpark = sparks[i];
		pSpark->display();
	}
	
	for (unsigned int i = 0; i < rockets.size(); i++)
	{
		ARocket* pRocket = rockets[i];
		pRocket->display();
	}

}

int AFireworks::getNumParticles()
{
	return sparks.size() + rockets.size();
}


void AFireworks::fireRocket(float posx, float* color)
	/*	fireRocket is called by AFireworksViewer::onKeyboard() function, when user presses the space key.
	*  Input:	float posx. X launch position of the rocket. Y launch position is always set to 0.
	*			float* color. color[0], color[1] and color[2] are the RGB color of this rocket.
	*                        It changes according to posx and it is provided for you.
	*  In this function, you want to generate a ARocket object and add it to the  rockets vector
	*  The initial state of the ARocket object is as follows:
	*      Position is posx and posy.
	*      launch angle is a random value between 80 and 110 degrees, assume the X axis is 0 degrees.
	*      Speed is a random number from 60 to 120.
	*      vertical force is mass * GRAVITY.
	*      rocket mass is 50.
	*      timeSpan (i.e. time to live) is 10.0
	*/
{
	float posy = 0.0f;
	ARocket *newRocket = new ARocket(color);

	// need to compute and set values for initial state of rocket  (including time to live)
	float stateVals[12] = { 0.0 };
	
	//TODO: Add your code here
	stateVals[0] = posx;
	stateVals[1] = posy;
	stateVals[2] = 0;

	float Speed = rand() % 61 + 60;//Random number between 60 and 120
	float angbase = rand() % 41 + 70;//Random number between 70 and 110
	stateVals[3] = cos(angbase*PI/ 180.0)*Speed;
	stateVals[4] = sin(angbase*PI/180.0)*Speed;
	stateVals[5] = 0.0;

	stateVals[6] = 0;
	stateVals[7] = -GRAVITY*50;
	stateVals[8] = 0;

	stateVals[9] = 50.0;
	stateVals[10] = 10.0;





	newRocket->setState(stateVals);
	newRocket->setAttractor(m_attractorPos);
	newRocket->setRepeller(m_repellerPos);
	newRocket->setWind(m_windForce);
	rockets.push_back(newRocket);
}



void AFireworks::explode(float rocketPosx, float rocketPosy, float rocketPosz, 
	                     float rocketVelx, float rocketVely, float rocketVelz, float* rocketColor)

	/*	explode is called in AFireworks::update() when a rocket reaches its top height.
	*  It is called ARocket::TOTALEXPLOSIONS times to generate a series of rings of sparks.
	*  Input: float posx. X position where a ring of sparks are generated.
	*		   float posy. Y position where a ring of sparks are generated.
	*		   float posy. Z position where a ring of sparks are generated.
	*  Input: float velx. X velocity of rocket when sparks are generated.
	*		   float vely. Y velocity of rocket when sparks are generated.
	*		   float velz. Z velocity of rocket when sparks are generated.
	*         float* color. color[0], color[1] and color[2] are the RGB color of the rocket. It will also be the color of the sparks it generates.
	*
	*  In this function, you want to generate a number of sparks that are uniformily distributed on a ring at [posx, posy, posz]
	*  then append them to the sparks vector using a push_back function call. Since this is a 2D demo you can set posz = 0.0
	*  The initial state vector for each spark should accommodate the constraints below:
	*   They should be evenly distribute on a ring.
	*
	*  At the time of the explosion:
	*      the number of sparks generated should be based on a random number between 10 and 60.
	*      the position of the sparks is determined by [posx, posy, posz]
	*      the magnitude of the inital velocity of each spark should  be based on a random value between 20 and 40
	*      the direction of the initial velocity of each spark should be uniformly distributed between 0 and 360 degrees
	*      the total velocity of the spark at the time of its creation is the sum of the rocket velocity and the intial spark velocity
	*  force on every spark is just the gravity.
	*  spark mass is 1
	*  Total timeToLive of every spark is 10.
	*/
		
	// The default mass of the rocket is 50.0, it should explode for a total of explosionCount = TOTALEXPLOSIONS time steps
{


	float stateVals[12] = { 0.0 };
	int numSparks = MAXSPARKS;
	float velocity = MAXVELOCITY;

	// TODO: Add  code here to randomize the number of sparks and their initial velocity
	numSparks = rand() % 51 + 10;//Random number between 10 and 60
	float velocitymagnitude = rand() % 21 + 20;//Random number between 20 and 40
	float pibasedconstant = (2.0*PI)/ numSparks;


	for (int i = 0; i < numSparks; i++)
	{
		ASpark* newSpark = new ASpark(rocketColor);
		// TODO: Add your code here

	
		stateVals[0] = rocketPosx;
		stateVals[1] = rocketPosy;
		stateVals[2] = 0.0;

		stateVals[3] = rocketVelx+sin(i * pibasedconstant)*velocitymagnitude;
		stateVals[4] = rocketVely+cos(i * pibasedconstant)*velocitymagnitude;
		stateVals[5] = 0.0;

		stateVals[6] = 0.0;
		stateVals[7] = -GRAVITY;//Technically gravity times mass, which is 1.
		stateVals[8] = 0.0;

		stateVals[9] = 1.0;
		stateVals[10] = 10.0;




		newSpark->setState(stateVals);
		newSpark->setAttractor(m_attractorPos);
		newSpark->setRepeller(m_repellerPos);
		newSpark->setWind(m_windForce);
		sparks.push_back(newSpark);
	}
}

void AFireworks::trail(float rocketPosx, float rocketPosy, float rocketPosz,
	float rocketVelx, float rocketVely, float rocketVelz, float* rocketColor)

	/*	trail is called in AFireworks::update() while the rocket is flying.
	*  Input: float posx. X position where smoke is generated.
	*		   float posy. Y position where smoke is generated.
	*		   float posy. Z position where smoke is generated.
	*  Input: float velx. X velocity of rocket when smoke is generated.
	*		   float vely. Y velocity of rocket when smoke is generated.
	*		   float velz. Z velocity of rocket when smoke is generated.
	*         float* color. color[0], color[1] and color[2] are the RGB color of the rocket. It will also be the color of the smoke it generates.
	*/
{

	//Heavily based on explode.

	float stateVals[12] = { 0.0 };

	int numSparks = rand() % 2;//Random number between 0 and 2 (simulate irregular release)


	for (int i = 0; i < numSparks; i++)
	{
		ASpark* newSpark = new ASpark(rocketColor);

		stateVals[0] = rocketPosx;
		stateVals[1] = rocketPosy;
		stateVals[2] = rocketPosz;

		stateVals[3] = -rocketVelx + rand() % 10;//Should shoot out in opposite direction, plus random number 0 to 10 to create a spread effect
		stateVals[4] = -rocketVely + rand() % 10;
		stateVals[5] = 0;

		stateVals[6] = 0;
		stateVals[7] = -GRAVITY;//Technically gravity times mass, which is 1.
		stateVals[8] = 0;

		stateVals[9] = 1;
		stateVals[10] = 8;//Slight change for rather slight visual effect


		//Make immune to bottom bouncing
		newSpark->m_COR = -1;

		newSpark->setState(stateVals);
		newSpark->setAttractor(m_attractorPos);
		newSpark->setRepeller(m_repellerPos);
		newSpark->setWind(m_windForce);
		sparks.push_back(newSpark);
	}
}

// One simulation step 
void AFireworks::update(float deltaT, int extForceMode)
{
	//Step 1. Iterate through every ASpark in sparks. If the spark is dead(life is 0), erase it from sparks.
	//        Code is given. It is also an example of erasing an element from a vector.
	ASpark* pSpark;
	int index = 0;
	m_deltaT = deltaT;

	for (unsigned int i = 0; i < sparks.size(); i++)
	{
		pSpark = sparks[index];
		if (!pSpark->m_alive)
		{
			sparks.erase(sparks.begin() + index);
			delete pSpark;
		}
		else index++;
	}


	//Step 2. TODO: Iterate through every ARocket in rockets. If the rocket is dead, erase it from rockets.
	          //If the rocket explosionCount does not equal - 1 then explode another ring of sparks.
	
	ARocket* pRocket;	
	index = 0;

	// Add you code here

	for (unsigned int i = 0; i < rockets.size(); i++)
	{
		pRocket = rockets[index];

		//Smoke generation
		if (pRocket->m_mode == FLYING)
		{
			//This is the key line you would comment out to turn smoke generation off.
			//Note that smoke does not bounce off the bottom and disappears slightly faster than an explosion (aesthetic choices, see trail function)
			trail(pRocket->m_Pos[0], pRocket->m_Pos[1], pRocket->m_Pos[2], pRocket->m_Vel[0], pRocket->m_Vel[1], pRocket->m_Vel[2], pRocket->m_color);
		}
		
		if (pRocket->m_explosionCount != -1)//if ((pRocket->m_explosionCount > -1)&&(pRocket->m_mode == EXPLOSION))
		{
			explode(pRocket->m_Pos[0], pRocket->m_Pos[1], pRocket->m_Pos[2], pRocket->m_Vel[0], pRocket->m_Vel[1], pRocket->m_Vel[2], pRocket->m_color);
		}

		if (pRocket->m_mode == DEAD)
		{
			rockets.erase(rockets.begin() + index);
			delete pRocket;
		}
		else index++;
	}










	//Step 3. update valid sparks and rockets.
	//        Code is given.

	for (unsigned int i = 0; i < sparks.size(); i++)
	{
		ASpark* pSpark = sparks[i];
		pSpark->update(m_deltaT, extForceMode);
	}
	
	for (unsigned int i = 0; i < rockets.size(); i++)
	{
		ARocket* pRocket = rockets[i];
		pRocket->update(m_deltaT, extForceMode);
	}
}
