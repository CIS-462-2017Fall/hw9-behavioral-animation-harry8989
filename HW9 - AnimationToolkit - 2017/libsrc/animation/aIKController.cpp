#include "aIKController.h"
#include "GL/glut.h"

#include "aActor.h"

#pragma warning (disable : 4018)

int IKController::gIKmaxIterations = 5;
double IKController::gIKEpsilon = 0.1;

// AIKchain class functions
/////////////////////////////////////////////////////////////////////////////////////////////////////////
AIKchain::AIKchain()
{
	mWeight0 = 0.1;
}

AIKchain::~AIKchain()
{

}

AJoint* AIKchain::getJoint(int index) 
{ 
	return mChain[index]; 
}

void AIKchain::setJoint(int index, AJoint* pJoint) 
{ 
	mChain[index] = pJoint; 
}

double AIKchain::getWeight(int index) 
{ 
	return mWeights[index]; 
}

void AIKchain::setWeight(int index, double weight) 
{ 
	mWeights[index] = weight; 
}

int AIKchain::getSize() 
{ 
	return mChain.size(); 
}

std::vector<AJoint*>& AIKchain::getChain() 
{ 
	return mChain; 
}

std::vector<double>& AIKchain::getWeights() 
{ 
	return mWeights; 
}

void AIKchain::setChain(std::vector<AJoint*> chain) 
{
	mChain = chain; 
}

void AIKchain::setWeights(std::vector<double> weights) 
{ 
	mWeights = weights; 
}

// AIKController class functions
/////////////////////////////////////////////////////////////////////////////////////////////////////////

IKController::IKController()
{
	m_pActor = NULL;
	m_pSkeleton = NULL;
	mvalidLimbIKchains = false;
	mvalidCCDIKchains = false;

	// Limb IK
	m_pEndJoint = NULL;
	m_pMiddleJoint = NULL;
	m_pBaseJoint = NULL;
	m_rotationAxis = vec3(0.0, 1.0, 0.0);

	ATransform desiredTarget = ATransform();
	mTarget0.setLocal2Parent(desiredTarget);  // target associated with end joint
	mTarget1.setLocal2Parent(desiredTarget);  // optional target associated with middle joint - used to specify rotation of middle joint about end/base axis
	mTarget0.setLocal2Global(desiredTarget);
	mTarget1.setLocal2Global(desiredTarget);

	//CCD IK
	mWeight0 = 0.1;  // default joint rotation weight value

}

IKController::~IKController()
{
}

ASkeleton* IKController::getSkeleton()
{
	return m_pSkeleton;
}

const ASkeleton* IKController::getSkeleton() const
{
	return m_pSkeleton;
}

ASkeleton* IKController::getIKSkeleton()
{
	return &mIKSkeleton;
}

const ASkeleton* IKController::getIKSkeleton() const
{
	return &mIKSkeleton;
}

AActor* IKController::getActor()
{
	return m_pActor;
}

void IKController::setActor(AActor* actor)

{
	m_pActor = actor;
	m_pSkeleton = m_pActor->getSkeleton();
}


AIKchain IKController::createIKchain(int endJointID, int desiredChainSize, ASkeleton* pSkeleton)
{
	// TODO: given the end joint ID and the desired size (i.e. length) of the IK chain, 
	// 1. add the corresponding skeleton joint pointers to the AIKChain "chain" vector data member starting with the end joint
	// 2. also add weight values to the associated AIKChain "weights" vector data member for use in the CCD IK implemention
	// Note: desiredChainSize = -1 should create an IK chain of maximum length (i.e. where the last chain joint is the joint before the root joint)
	bool getMaxSize = false;

	int EndJointID = endJointID;
	std::vector<AJoint*> chain;
	std::vector<double> weights;

	chain.clear();
	weights.clear();
	if (desiredChainSize == -1)
		getMaxSize = true;

	if ((EndJointID >= 0) && (EndJointID < pSkeleton->getNumJoints()))
	{
		AJoint* pJoint = pSkeleton->getJointByID(endJointID);

		// TODO: add code here to generate chain of desired size or terminate at the joint before root joint, so that root will not change during IK	
		// also add weight values to corresponding weights vector  (default value = 0.1)
		AJoint* Parent = pJoint->getParent();
		int chainsize = 0;
		while ((Parent != NULL)&&((getMaxSize)||(chainsize<=desiredChainSize))) {//Parent is null when this joint is the root
			//Adds to chain if current node is before the root and either we want the max size or we haven't yet added desiredChainSize joints
			chain.push_back(pJoint);
			weights.push_back(mWeight0);
			pJoint = Parent;
			Parent = pJoint->getParent();
			chainsize += 1;
		}

	}
	AIKchain result;
	result.setChain(chain);
	result.setWeights(weights);

	return result;
}



bool IKController::IKSolver_Limb(int endJointID, const ATarget& target)
{
	// Implements the analytic/geometric IK method assuming a three joint limb  

	if (!mvalidLimbIKchains)
	{
		mvalidLimbIKchains = createLimbIKchains();
		//assert(mvalidLimbIKchains);
	}

	// copy transforms from base skeleton
	mIKSkeleton.copyTransforms(m_pSkeleton);

	vec3 desiredRootPosition;

	switch (endJointID)
	{
	case mLhandID:
		mLhandTarget = target;
		computeLimbIK(mLhandTarget, mLhandIKchain, -axisY, &mIKSkeleton);
		break;
	case mRhandID:
		mRhandTarget = target;
		computeLimbIK(mRhandTarget, mRhandIKchain, axisY, &mIKSkeleton);
		break;
	case mLfootID:
		mLfootTarget = target;
		computeLimbIK(mLfootTarget, mLfootIKchain, axisX, &mIKSkeleton);
		break;
	case mRfootID:
		mRfootTarget = target;
		computeLimbIK(mRfootTarget, mRfootIKchain, axisX, &mIKSkeleton);
		break;
	case mRootID:
		desiredRootPosition = target.getGlobalTranslation();
		mIKSkeleton.getJointByID(mRootID)->setLocalTranslation(desiredRootPosition);
		mIKSkeleton.update();
		computeLimbIK(mLhandTarget, mLhandIKchain, -axisY, &mIKSkeleton);
		computeLimbIK(mRhandTarget, mRhandIKchain, axisY, &mIKSkeleton);
		computeLimbIK(mLfootTarget, mLfootIKchain, axisX, &mIKSkeleton);
		computeLimbIK(mRfootTarget, mRfootIKchain, axisX, &mIKSkeleton);
		break;
	default:
		mIKchain = createIKchain(endJointID, 3, &mIKSkeleton);
		computeLimbIK(target, mIKchain, axisY, &mIKSkeleton);
		break;
	}

	// update IK Skeleton transforms
	mIKSkeleton.update();

	// copy IK skeleton transforms to main skeleton
	m_pSkeleton->copyTransforms(&mIKSkeleton);

	return true;
}



int IKController::createLimbIKchains()
{
	bool validChains = false;
	int desiredChainSize = 3;

	// create IK chains for Lhand, Rhand, Lfoot and Rfoot 
	mLhandIKchain = createIKchain(mLhandID, desiredChainSize, &mIKSkeleton);
	mRhandIKchain = createIKchain(mRhandID, desiredChainSize, &mIKSkeleton);
	mLfootIKchain = createIKchain(mLfootID, desiredChainSize, &mIKSkeleton);
	mRfootIKchain = createIKchain(mRfootID, desiredChainSize, &mIKSkeleton);
	
	if (mLhandIKchain.getSize() == 3 && mRhandIKchain.getSize() == 3 && mLfootIKchain.getSize() == 3 && mRfootIKchain.getSize() == 3)
	{
		validChains = true;
		
		// initalize end joint target transforms for Lhand, Rhand, Lfoot and Rfoot based on current position and orientation of joints
		mIKSkeleton.copyTransforms(m_pSkeleton);
		mLhandTarget.setLocal2Global(mIKSkeleton.getJointByID(mLhandID)->getLocal2Global());
		mRhandTarget.setLocal2Global(mIKSkeleton.getJointByID(mRhandID)->getLocal2Global());
		mLfootTarget.setLocal2Global(mIKSkeleton.getJointByID(mLfootID)->getLocal2Global());
		mRfootTarget.setLocal2Global(mIKSkeleton.getJointByID(mRfootID)->getLocal2Global());
	}

	return validChains;
}




int IKController::computeLimbIK(ATarget target, AIKchain& IKchain, const vec3 midJointAxis, ASkeleton* pIKSkeleton)
{
	// TODO: Implement the analytic/geometric IK method assuming a three joint limb  
	// The actual position of the end joint should match the target position within some episilon error 
	// the variable "midJointAxis" contains the rotation axis for the middle joint
	
	bool result = false;
	int endJointID;
	mTarget0 = target;

	if (IKchain.getSize() > 0)
		 endJointID = IKchain.getJoint(0)->getID();
	else endJointID = -1;

	if ((endJointID >= 0) && (endJointID < pIKSkeleton->getNumJoints()))
	{
		m_pEndJoint = IKchain.getJoint(0);
		m_pMiddleJoint = IKchain.getJoint(1);
		m_pBaseJoint = IKchain.getJoint(2);

		//TODO:
		// 1. compute error vector between target and end joint
		vec3 errorvec = mTarget0.getGlobalTranslation() - m_pEndJoint->getGlobalTranslation();//Just for debug purposes
		// 2. compute vector between end Joint and base joint
		vec3 endbasevec = m_pEndJoint->getGlobalTranslation() - m_pBaseJoint->getGlobalTranslation();//Just for debug purposes
		// 3. compute vector between target and base joint
		vec3 r = mTarget0.getGlobalTranslation() - m_pBaseJoint->getGlobalTranslation();


		// 4. Compute desired angle for middle joint
		double lone = (m_pMiddleJoint->getGlobalTranslation() - m_pBaseJoint->getGlobalTranslation()).Length();
		double ltwo = (m_pEndJoint->getGlobalTranslation() - m_pMiddleJoint->getGlobalTranslation()).Length();
		double rd = r.Length();
		
		double acosme = -((lone*lone) + (ltwo*ltwo) - (rd*rd)) / (2 * lone*ltwo);
		double middleang = acos(acosme);
		if (acosme > 1.0) {//Covers acos range issues
			middleang = acos(1.0);
		}
		if (acosme < -1.0) {
			middleang = acos(-1.0);
		}

		//From slides, not actually needed
		//double asinme = (ltwo*sin(middleang))/(rd);
		//double angonez = asin(asinme);
		//if (asinme > 1.0) {//Covers asin range issues
		//	angonez = asin(1.0);
		//}
		//if (asinme < -1.0) {
		//	angonez = asin(-1.0);
		//}
		//double angtwoz = middleang - M_PI;

		// 5. given desired angle and midJointAxis, compute new local middle joint rotation matrix and update joint transform
		mat3 newmid;
		newmid.FromAxisAngle(midJointAxis, middleang);
		//m_pMiddleJoint->setLocalRotation(newmid); OR m_pMiddleJoint->setLocalRotation(m_pMiddleJoint->getLocalRotation()*newmid); Whichever is correct
		m_pMiddleJoint->setLocalRotation(newmid);

		// 5. update joint transform
		m_pMiddleJoint->updateTransform();
	
		// 6. compute vector between target and base joint
		//See step 3

		// 7. Compute base joint rotation axis (in global coords) and desired angle
		vec3 bjraxis;
		double baseang;
		//m_pBaseJoint->getGlobalRotation().ToAxisAngle(bjraxis, baseang);//Keep the axis, want new angle.
		
		vec3 newendbasevec = m_pEndJoint->getGlobalTranslation() - m_pBaseJoint->getGlobalTranslation();
		//vec3 newendbasevec = r;
		double keydotproduct = r[0] * newendbasevec[0] + r[1] * newendbasevec[1] + r[2] * newendbasevec[2];
		double acosmealso = (keydotproduct) / (newendbasevec.Length()*rd);
		if (abs(newendbasevec.Length()) < 0.0001f) {//Deal with divide by zero error
			acosmealso = 0;
		}
		baseang = acos(acosmealso);
		if (acosmealso > 1.0) {//Covers acos range issues
			baseang = acos(1.0);
		}
		if (acosmealso < -1.0) {
			baseang = acos(-1.0);
		}
		bjraxis=((newendbasevec.Cross(r)).Normalize());

		// 8. transform base joint rotation axis to local coordinates
		vec3 localbjraxis = m_pBaseJoint->getGlobalRotation().Inverse()*bjraxis;

		// 9. given desired angle and local rotation axis, compute new local rotation matrix and update base joint transform
		mat3 newbase;
		newbase.FromAxisAngle(localbjraxis, baseang);
		//m_pBaseJoint->setLocalRotation(newbase); OR m_pBaseJoint->setLocalRotation(m_pBaseJoint->getLocalRotation()*newbase); Whichever is correct
		m_pBaseJoint->setLocalRotation(m_pBaseJoint->getLocalRotation()*newbase);
		//m_pBaseJoint->setLocalRotation(newbase);
		m_pBaseJoint->updateTransform();
	
	}
	return result;

}

bool IKController::IKSolver_CCD(int endJointID, const ATarget& target)
{
	// Implements the CCD IK method assuming a three joint limb 

	bool validChains = false;

	if (!mvalidCCDIKchains)
	{
		mvalidCCDIKchains = createCCDIKchains();
		//assert(mvalidCCDIKchains);
	}

	// copy transforms from base skeleton
	mIKSkeleton.copyTransforms(m_pSkeleton);

	vec3 desiredRootPosition;

	switch (endJointID)
	{
	case mLhandID:
		mLhandTarget = target;
		computeCCDIK(mLhandTarget, mLhandIKchain, &mIKSkeleton);
		break;
	case mRhandID:
		mRhandTarget = target;
		computeCCDIK(mRhandTarget, mRhandIKchain, &mIKSkeleton);
		break;
	case mLfootID:
		mLfootTarget = target;
		computeCCDIK(mLfootTarget, mLfootIKchain, &mIKSkeleton);
		break;
	case mRfootID:
		mRfootTarget = target;
		computeCCDIK(mRfootTarget, mRfootIKchain, &mIKSkeleton);
		break;
	case mRootID:
		desiredRootPosition = target.getGlobalTranslation();
		mIKSkeleton.getJointByID(mRootID)->setLocalTranslation(desiredRootPosition);
		mIKSkeleton.update();
		computeCCDIK(mLhandTarget, mLhandIKchain, &mIKSkeleton);
		computeCCDIK(mRhandTarget, mRhandIKchain, &mIKSkeleton);
		computeCCDIK(mLfootTarget, mLfootIKchain, &mIKSkeleton);
		computeCCDIK(mRfootTarget, mRfootIKchain, &mIKSkeleton);
		break;
	default:
		mIKchain = createIKchain(endJointID, -1, &mIKSkeleton);
		computeCCDIK(target, mIKchain, &mIKSkeleton);
		break;
	}

	// update IK Skeleton transforms
	mIKSkeleton.update();

	// copy IK skeleton transforms to main skeleton
	m_pSkeleton->copyTransforms(&mIKSkeleton);

	return true;
}

int IKController::createCCDIKchains()
{
	bool validChains = false;

	int desiredChainSize = -1;  // default of -1 creates IK chain of maximum length from end joint to child joint of root


	// create IK chains for Lhand, Rhand, Lfoot and Rfoot 
	mLhandIKchain = createIKchain(mLhandID, desiredChainSize, &mIKSkeleton);
	mRhandIKchain = createIKchain(mRhandID, desiredChainSize, &mIKSkeleton);
	mLfootIKchain = createIKchain(mLfootID, desiredChainSize, &mIKSkeleton);
	mRfootIKchain = createIKchain(mRfootID, desiredChainSize, &mIKSkeleton);

	if (mLhandIKchain.getSize() > 1 && mRhandIKchain.getSize() > 1 && mLfootIKchain.getSize() > 1 && mRfootIKchain.getSize() > 1)
	{
		validChains = true;

		// initalize end joint target transforms for Lhand, Rhand, Lfoot and Rfoot based on current position and orientation of joints
		mIKSkeleton.copyTransforms(m_pSkeleton);
		mLhandTarget.setLocal2Global(mIKSkeleton.getJointByID(mLhandID)->getLocal2Global());
		mRhandTarget.setLocal2Global(mIKSkeleton.getJointByID(mRhandID)->getLocal2Global());
		mLfootTarget.setLocal2Global(mIKSkeleton.getJointByID(mLfootID)->getLocal2Global());
		mRfootTarget.setLocal2Global(mIKSkeleton.getJointByID(mRfootID)->getLocal2Global());
	}

	return validChains;
}


int IKController::computeCCDIK(ATarget target, AIKchain& IKchain, ASkeleton* pIKSkeleton)
{

	// TODO: Implement CCD IK  
	// The actual position of the end joint should match the desiredEndPos within some episilon error 

	bool result = false;

	mTarget0 = target;
	vec3 desiredEndPos = mTarget0.getGlobalTranslation();  // Get desired position of EndJoint

	int chainSize = IKchain.getSize();
	if (chainSize == 0) // There are no joints in the IK chain for manipulation
		return false;

	double epsilon = gIKEpsilon;
	int maxIterations = gIKmaxIterations;
	int numIterations = 0;

	m_pEndJoint = IKchain.getJoint(0);
	int endJointID = m_pEndJoint->getID();
	m_pBaseJoint = IKchain.getJoint(chainSize - 1);

	pIKSkeleton->copyTransforms(m_pSkeleton);

	if ((endJointID >= 0) && (endJointID < pIKSkeleton->getNumJoints()))
	{
		//TODO:
		
		while (numIterations <= maxIterations) {
			numIterations += 1;//Repeat the following at most maxIterations times

			int curjointid = 1;//0 causes us to use endjoint first, causing deltaangj to have a divide by zero problem
			vec3 endjointloc = m_pEndJoint->getGlobalTranslation();//Defined outside to allow early break
			while (curjointid < chainSize) {

				// 1. compute axis and angle for each joint in the IK chain (distal to proximal) in global coordinates

				endjointloc = m_pEndJoint->getGlobalTranslation();

				AJoint* curjoint = IKchain.getJoint(curjointid);
				vec3 ej = desiredEndPos - endjointloc;
				vec3 rjn = endjointloc-curjoint->getGlobalTranslation();
				double rjnrjndotproduct = rjn[0] * rjn[0] + rjn[1] * rjn[1] + rjn[2] * rjn[2];
				double rjnejdotproduct = rjn[0] * ej[0] + rjn[1] * ej[1] + rjn[2] * ej[2];;
				//double cj = 0.2;//Scalar gain that helps with convergence

				//Angle and axis
				//double deltaangj = cj*(((rjn.Cross(ej)).Length())/(rjnrjndotproduct+ rjnejdotproduct));
				double deltaangj = (((rjn.Cross(ej)).Length()) / (rjnrjndotproduct + rjnejdotproduct));
				vec3 ahat = ((rjn.Cross(ej)).Normalize());

				// 2. once you have the desired axis and angle, convert axis to local joint coords 
				vec3 localahat = curjoint->getGlobalRotation().Inverse()*ahat;

				// 3. multiply angle by corresponding joint weight value
				double weightedang = deltaangj*(IKchain.getWeight(curjointid));

				// 4. compute new local joint rotation matrix
				mat3 deltarj;
				deltarj.FromAxisAngle(localahat, weightedang);
				curjoint->setLocalRotation(curjoint->getLocalRotation()*deltarj);

				// 5. update joint transform
				curjoint->updateTransform();

				// 6. repeat same operations above for each joint in the IKchain from end to base joint
				//Inherent result of while loop
				curjointid += 1;
				endjointloc = m_pEndJoint->getGlobalTranslation();
				if (abs((endjointloc - desiredEndPos).Length()) < epsilon) {
					return true;
					break;
				}
			}
			//Check if we can just stop now
			if (abs((endjointloc- desiredEndPos).Length()) < epsilon) {
				return true;
				break;
			}
		}

	}
	return result;

}


bool IKController::IKSolver_PseudoInv(int endJointID, const ATarget& target)
{
	bool result = false;

	// TODO: Implement Pseudo Inverse-based IK  
	// The actual position of the end joint should match the target position after the skeleton is updated with the new joint angles

	return result;
}

bool IKController::IKSolver_Other(int endJointID, const ATarget& target)
{
	
	bool result = false;
	
	// TODO: Put Optional IK implementation or enhancements here
	 
	return result;
}