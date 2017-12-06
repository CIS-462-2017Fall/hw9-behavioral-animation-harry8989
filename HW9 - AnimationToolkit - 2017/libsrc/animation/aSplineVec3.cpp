#include "aSplineVec3.h"
#include <algorithm>
#include <Eigen/Dense>

#pragma warning(disable:4018)
#pragma warning(disable:4244)

ASplineVec3::ASplineVec3() : mInterpolator(new ALinearInterpolatorVec3())
{
}

ASplineVec3::~ASplineVec3()
{
    delete mInterpolator;
}

void ASplineVec3::setFramerate(double fps)
{
    mInterpolator->setFramerate(fps);
}

double ASplineVec3::getFramerate() const
{
    return mInterpolator->getFramerate();
}

void ASplineVec3::setLooping(bool loop)
{
    mLooping = loop;
}

bool ASplineVec3::getLooping() const
{
    return mLooping;
}

void ASplineVec3::setInterpolationType(ASplineVec3::InterpolationType type)
{
    double fps = getFramerate();

    delete mInterpolator;
    switch (type)
    {
    case LINEAR: mInterpolator = new ALinearInterpolatorVec3(); break;
    case CUBIC_BERNSTEIN: mInterpolator = new ABernsteinInterpolatorVec3(); break;
    case CUBIC_CASTELJAU: mInterpolator = new ACasteljauInterpolatorVec3(); break;
	case CUBIC_MATRIX: mInterpolator = new AMatrixInterpolatorVec3(); break; 
	case CUBIC_HERMITE: mInterpolator = new AHermiteInterpolatorVec3(); break;
	case CUBIC_BSPLINE: mInterpolator = new ABSplineInterpolatorVec3(); break;
    };
    
    mInterpolator->setFramerate(fps);
    computeControlPoints();
    cacheCurve();
}

ASplineVec3::InterpolationType ASplineVec3::getInterpolationType() const
{
    return mInterpolator->getType();
}

void ASplineVec3::editKey(int keyID, const vec3& value)
{
    assert(keyID >= 0 && keyID < mKeys.size());
    mKeys[keyID].second = value;
    computeControlPoints();
    cacheCurve();
}

void ASplineVec3::editControlPoint(int ID, const vec3& value)
{
    assert(ID >= 0 && ID < mCtrlPoints.size()+2);
    if (ID == 0)
    {
        mStartPoint = value;
        computeControlPoints();
    }
    else if (ID == mCtrlPoints.size() + 1)
    {
        mEndPoint = value;
        computeControlPoints();
    }
    else mCtrlPoints[ID-1] = value;
    cacheCurve();
}

void ASplineVec3::appendKey(double time, const vec3& value, bool updateCurve)
{
    mKeys.push_back(Key(time, value));

    if (mKeys.size() >= 2)
    {
        int totalPoints = mKeys.size();

        //If there are more than 1 interpolation point, set up the 2 end points to help determine the curve.
        //They lie on the tangent of the first and last interpolation points.
        vec3 tmp = mKeys[0].second - mKeys[1].second;
        double n = tmp.Length();
        mStartPoint = mKeys[0].second + (tmp / n) * n * 0.25; // distance to endpoint is 25% of distance between first 2 points

        tmp = mKeys[totalPoints - 1].second - mKeys[totalPoints - 2].second;
        n = tmp.Length();
        mEndPoint = mKeys[totalPoints - 1].second + (tmp / n) * n * 0.25;
    }

    if (updateCurve)
    {
        computeControlPoints();
        cacheCurve();
    }
}

void ASplineVec3::appendKey(const vec3& value, bool updateCurve)
{
    if (mKeys.size() == 0)
    {
        appendKey(0, value, updateCurve);
    }
    else
    {
        double lastT = mKeys[mKeys.size() - 1].first;
        appendKey(lastT + 1, value, updateCurve);
    }
}

void ASplineVec3::deleteKey(int keyID)
{
    assert(keyID >= 0 && keyID < mKeys.size());
    mKeys.erase(mKeys.begin() + keyID);
    computeControlPoints();
    cacheCurve();
}

vec3 ASplineVec3::getKey(int keyID)
{
    assert(keyID >= 0 && keyID < mKeys.size());
    return mKeys[keyID].second;
}

int ASplineVec3::getNumKeys() const
{
    return mKeys.size();
}

vec3 ASplineVec3::getControlPoint(int ID)
{
    assert(ID >= 0 && ID < mCtrlPoints.size()+2);
    if (ID == 0) return mStartPoint;
    else if (ID == mCtrlPoints.size() + 1) return mEndPoint;
    else return mCtrlPoints[ID-1];
}

int ASplineVec3::getNumControlPoints() const
{
    return mCtrlPoints.size()+2; // include endpoints
}

void ASplineVec3::clear()
{
    mKeys.clear();
}

double ASplineVec3::getDuration() const 
{
    return mKeys[mKeys.size()-1].first;
}

double ASplineVec3::getNormalizedTime(double t) const 
{
    return (t / getDuration());
}

vec3 ASplineVec3::getValue(double t)
{
    if (mCachedCurve.size() == 0) return vec3();

    double dt = mInterpolator->getDeltaTime();
    int rawi = (int)(t / dt); // assumes uniform spacing
    int i = rawi % mCachedCurve.size();
    double frac = t - rawi*dt;
    int inext = i + 1;
    if (!mLooping) inext = std::min<int>(inext, mCachedCurve.size() - 1);
    else inext = inext % mCachedCurve.size();

    vec3 v1 = mCachedCurve[i];
    vec3 v2 = mCachedCurve[inext];
    vec3 v = v1*(1 - frac) + v2 * frac;
    return v;
}

void ASplineVec3::cacheCurve()
{
    mInterpolator->interpolate(mKeys, mCtrlPoints, mCachedCurve);
}

void ASplineVec3::computeControlPoints()
{
    mInterpolator->computeControlPoints(mKeys, mCtrlPoints, mStartPoint, mEndPoint);
}

int ASplineVec3::getNumCurveSegments() const
{
    return mCachedCurve.size();
}

vec3 ASplineVec3::getCurvePoint(int i) const
{
    return mCachedCurve[i];
}

//---------------------------------------------------------------------
AInterpolatorVec3::AInterpolatorVec3(ASplineVec3::InterpolationType t) : mDt(1.0 / 120.0), mType(t)
{
}

void AInterpolatorVec3::setFramerate(double fps)
{
    mDt = 1.0 / fps;
}

double AInterpolatorVec3::getFramerate() const
{
    return 1.0 / mDt;
}

double AInterpolatorVec3::getDeltaTime() const
{
    return mDt;
}

void AInterpolatorVec3::interpolate(const std::vector<ASplineVec3::Key>& keys, 
    const std::vector<vec3>& ctrlPoints, std::vector<vec3>& curve)
{
	vec3 val = 0.0;
	double u = 0.0; 

	curve.clear();
	
	int numSegments = keys.size() - 1;
    for (int segment = 0; segment < numSegments; segment++)
    {
        for (double t = keys[segment].first; t < keys[segment+1].first - FLT_EPSILON; t += mDt)
        {
            
			// TODO: Compute u, fraction of duration between segment and segmentnext, for example,
            // u = 0.0 when t = keys[segment-1].first  
            // u = 1.0 when t = keys[segment].first
			u = (t - keys[segment].first) / (keys[segment + 1].first - keys[segment].first);


            val = interpolateSegment(keys, ctrlPoints, segment, u);
            curve.push_back(val);
        }
    }
	// add last point
	if (keys.size() > 1)
	{
		u = 1.0;
		val = interpolateSegment(keys, ctrlPoints, numSegments-1, u);
		curve.push_back(val);
	}
	
    
}


vec3 ALinearInterpolatorVec3::interpolateSegment(
    const std::vector<ASplineVec3::Key>& keys,
    const std::vector<vec3>& ctrlPoints, 
    int segment, double u)
{
   
	vec3 curveValue(0, 0, 0);
	vec3 key0 = keys[segment].second;
    vec3 key1 = keys[segment+1].second;

    // TODO: 
	//Step 1: Create a Lerp helper function
	//Step 2: Linear interpolate between key0 and key1 so that u = 0 returns key0 and u = 1 returns key1
	curveValue = key0 + (key1 - key0)*u;
    
	return curveValue;
}

vec3 ABernsteinInterpolatorVec3::interpolateSegment(
    const std::vector<ASplineVec3::Key>& keys,
    const std::vector<vec3>& ctrlPoints, 
    int segment, double t)
{
	vec3 b0; 
	vec3 b1;
	vec3 b2; 
	vec3 b3;
    vec3 curveValue(0,0,0);
    // TODO: 
	// Step1: Get the 4 control points, b0, b1, b2 and b3 from the ctrlPoints vector
	// Step2: Compute the interpolated value f(u) point using  Bernstein polynomials
	b0 = ctrlPoints[4*segment];
	b1 = ctrlPoints[4 * segment + 1];
	b2 = ctrlPoints[4 * segment + 2];
	b3 = ctrlPoints[4 * segment + 3];
	//Bernstein Polynomials
	float B30 = pow((1.0 - t), 3.0);
	float B31 = 3.0*t*pow((1.0 - t), 2.0);
	float B32 = 3.0*pow(t, 2.0)*(1.0 - t);
	float B33 = pow(t,3.0);
	//Combine
	curveValue = b0*B30 + b1*B31 + b2*B32 + b3*B33;

    
	return curveValue;

}


vec3 ACasteljauInterpolatorVec3::interpolateSegment(
    const std::vector<ASplineVec3::Key>& keys,
    const std::vector<vec3>& ctrlPoints, 
    int segment, double t)
{
	vec3 b0;
	vec3 b1;
	vec3 b2;
	vec3 b3;
	vec3 curveValue(0, 0, 0);
	
	// TODO: 
	// Step1: Get the 4 control points, b0, b1, b2 and b3 from the ctrlPoints vector
	// Step2: Compute the interpolated value f(u) point using  deCsteljau alogithm
	b0 = ctrlPoints[4 * segment];
	b1 = ctrlPoints[4 * segment + 1];
	b2 = ctrlPoints[4 * segment + 2];
	b3 = ctrlPoints[4 * segment + 3];
	//Start linearly interpolating between these points
	vec3 b10 = b0*(1.0 - t) + b1*t;
	vec3 b11 = b1*(1.0 - t) + b2*t;
	vec3 b12 = b2*(1.0 - t) + b3*t;
	//Second level of linear interpolation
	vec3 b20 = b10*(1.0 - t) + b11*t;
	vec3 b21 = b11*(1.0 - t) + b12*t;
	//Third level of linear interpolation
	vec3 b30 = b20*(1.0 - t) + b21*t;
	//Use result
	curveValue = b30;
	
	return curveValue;
}

vec3 AMatrixInterpolatorVec3::interpolateSegment(
    const std::vector<ASplineVec3::Key>& keys,
    const std::vector<vec3>& ctrlPoints, 
    int segment, double t)
{
	vec3 b0;
	vec3 b1;
	vec3 b2;
	vec3 b3;
	vec3 curveValue(0, 0, 0);

	// TODO: 
	// Step1: Get the 4 control points, b0, b1, b2 and b3 from the ctrlPoints vector
	// Step2: Compute the interpolated value f(u) point using  matrix method f(u) = GMU
	// Hint: Using Eigen::MatrixXd data representations for a matrix operations

	b0 = ctrlPoints[4 * segment];
	b1 = ctrlPoints[4 * segment + 1];
	b2 = ctrlPoints[4 * segment + 2];
	b3 = ctrlPoints[4 * segment + 3];
	//Create Matrices
	Eigen::MatrixXd GBezier(3,4);
	GBezier(0, 0) = b0[0];
	GBezier(0, 1) = b1[0];
	GBezier(0, 2) = b2[0];
	GBezier(0, 3) = b3[0];
	GBezier(1, 0) = b0[1];
	GBezier(1, 1) = b1[1];
	GBezier(1, 2) = b2[1];
	GBezier(1, 3) = b3[1];
	GBezier(2, 0) = b0[2];
	GBezier(2, 1) = b1[2];
	GBezier(2, 2) = b2[2];
	GBezier(2, 3) = b3[2];
	Eigen::MatrixXd MBezier(4, 4);
	MBezier(0, 0) = 1;
	MBezier(0, 1) = -3;
	MBezier(0, 2) = 3;
	MBezier(0, 3) = -1;
	MBezier(1, 0) = 0;
	MBezier(1, 1) = 3;
	MBezier(1, 2) = -6;
	MBezier(1, 3) = 3;
	MBezier(2, 0) = 0;
	MBezier(2, 1) = 0;
	MBezier(2, 2) = 3;
	MBezier(2, 3) = -3;
	MBezier(3, 0) = 0;
	MBezier(3, 1) = 0;
	MBezier(3, 2) = 0;
	MBezier(3, 3) = 1;
	Eigen::Vector4d WeirdU;
	WeirdU(0) = 1;
	WeirdU(1) = t;
	WeirdU(2) = pow(t, 2);
	WeirdU(3) = pow(t, 3);
	//Combine
	Eigen::MatrixXd GMU = GBezier*MBezier*WeirdU;
	
	//Extract
	curveValue.set(GMU(0), GMU(1), GMU(2));
	//std::cout << MBezier << std::endl;

	return curveValue;
}

vec3 AHermiteInterpolatorVec3::interpolateSegment(
    const std::vector<ASplineVec3::Key>& keys,
    const std::vector<vec3>& ctrlPoints, 
    int segment, double t)
{

    vec3 p0 = keys[segment].second;
    vec3 p1 = keys[segment + 1].second;
    vec3 q0 = ctrlPoints[segment]; // slope at p0
    vec3 q1 = ctrlPoints[segment + 1]; // slope at p1
	vec3 curveValue(0, 0, 0);

    // TODO: Compute the interpolated value h(u) using a cubic Hermite polynomial
	curveValue = (2.0*pow(t, 3.0) - 3.0*pow(t, 2.0) + 1.0)*p0 + (-2.0*pow(t, 3.0) + 3.0*pow(t, 2.0))*p1 + (pow(t, 3.0) - 2.0*pow(t, 2.0) + t)*q0 + (pow(t, 3.0) - pow(t, 2.0))*q1;

    return curveValue;
}

//Create helper function for BSplines
double dN(Eigen::MatrixXd knots, int n, int j, double t, int l)
{
	if (n == 0) {
		if ((t >= knots(j)) && (t<knots(j + 1))) {
			return 1.0;
		}
		else {
			return 0.0;
		}
	}
	else {
		if (l == 0) {
			return dN(knots, (n - 1), j, t, l)*((t - knots(j)) / (knots((j + n)) - knots(j))) + dN(knots, (n - 1), (j + 1), t, l)*((knots((j + n + 1)) - t) / (knots((j + n + 1)) - knots((j + 1))));
		}
		else {
			return n*(dN(knots, (n - 1), j, t, l - 1)*(1 / (knots((j + n)) - knots(j))) - dN(knots, (n - 1), (j + 1), t, l - 1)*(1 / (knots((j + n + 1)) - knots((j + 1)))));
		}
	}
}
int fauxknots(int input) {
	return input - 3;
}
double dNN(int n, int j, double t, int l)
{
	
	if (n == 0) {
		if ((t >= fauxknots(j)) && (t<fauxknots(j + 1))) {
			return 1.0;
		}
		else {
			return 0.0;
		}
	}
	else {
		if (l == 0) {
			return dNN((n - 1), j, t, l)*((t - fauxknots(j)) / (fauxknots((j + n)) - fauxknots(j))) + dNN((n - 1), (j + 1), t, l)*((fauxknots((j + n + 1)) - t) / (fauxknots((j + n + 1)) - fauxknots((j + 1))));
		}
		else {
			return n*(dNN((n - 1), j, t, l - 1)*(1 / (fauxknots((j + n)) - fauxknots(j))) - dNN((n - 1), (j + 1), t, l - 1)*(1 / (fauxknots((j + n + 1)) - fauxknots((j + 1)))));
		}
	}
}

vec3 ABSplineInterpolatorVec3::interpolateSegment(
    const std::vector<ASplineVec3::Key>& keys,
    const std::vector<vec3>& ctrlPoints, 
    int segment, double t)
{
	vec3 curveValue(0, 0, 0);
	
	// Hint: Create a recursive helper function N(knots,n,j,t) to calculate BSpline basis function values at t, where
	//     knots = knot array
	//	   n = degree of the spline curves (n =3 for cubic)
	//     j = curve interval on knot vector in which to interpolate
	//     t = time value	

	// Step 1: determine the index j
	// Step 2: compute the n nonzero Bspline Basis functions N given j
	// Step 3: get the corresponding control points from the ctrlPoints vector
	// Step 4: compute the Bspline curveValue at time t

	int j = 3 + segment;

	if (keys.size() > 1) {
		curveValue = ctrlPoints[(j - 3)] * dNN(3, (j - 3), t, 0) + ctrlPoints[(j - 2)] * dNN(3, (j - 2), t, 0) + ctrlPoints[(j - 1)] * dNN(3, (j - 1), t, 0) + ctrlPoints[j] * dNN(3, j, t, 0);
	}

	
	return curveValue;
}

void ACubicInterpolatorVec3::computeControlPoints(
    const std::vector<ASplineVec3::Key>& keys, 
    std::vector<vec3>& ctrlPoints, 
    vec3& startPoint, vec3& endPoint)
{
    ctrlPoints.clear();
    if (keys.size() <= 1) return;

    for (int i = 1; i < keys.size(); i++)
    {
        vec3 b0, b1, b2, b3;

        // TODO: compute b0, b1, b2, b3
		b0 = keys[i - 1].second;
		b3 = keys[i].second;
		if (i - 2 == -1) {
			b1= keys[i - 1].second + ((keys[i].second - startPoint) / 2) / 3;
		}
		else {
			b1 = keys[i - 1].second + ((keys[i].second - keys[i - 2].second) / 2) / 3;
		}
		if (i + 1 == keys.size()) {
			b2 = keys[i].second - ((endPoint - keys[i - 1].second) / 2) / 3;
		}
		else {
			b2 = keys[i].second - ((keys[i + 1].second - keys[i - 1].second) / 2) / 3;
		}

        ctrlPoints.push_back(b0);
        ctrlPoints.push_back(b1);
        ctrlPoints.push_back(b2);
        ctrlPoints.push_back(b3);
    }
}

void AHermiteInterpolatorVec3::computeControlPoints(
    const std::vector<ASplineVec3::Key>& keys,
    std::vector<vec3>& ctrlPoints,
    vec3& startPoint, vec3& endPoint)
{
    ctrlPoints.clear();
    if (keys.size() <= 1) return;

    int numKeys = keys.size();


    // TODO: 
	// For each key point pi, compute the corresonding value of the slope pi_prime.
	// Hints: Using Eigen::MatrixXd for a matrix data structures, 
	// this can be accomplished by solving the system of equations AC=D for C.
	// Don't forget to save the values computed for C in ctrlPoints
	// For clamped endpoint conditions, set 1st derivative at first and last points (p0 and pm) to s0 and s1, respectively
	// For natural endpoints, set 2nd derivative at first and last points (p0 and pm) equal to 0

	// Step 1: Initialize A
	// Step 2: Initialize D
	// Step 3: Solve AC=D for C
	// Step 4: Save control points in ctrlPoints
	Eigen::MatrixXd A(numKeys, numKeys);
	Eigen::MatrixXd D(numKeys, 3);

	//Initialize top and bottom rows of A and D
	//Top and bottom of A
	A(0, 0) = 2.0;
	A(0, 1) = 1.0;
	A((numKeys-1), (numKeys - 2)) = 1.0;
	A((numKeys - 1), (numKeys - 1)) = 2.0;
	for (int k = 2; k < numKeys; k++)
	{
		A(0, k) = 0.0;
		A((numKeys - 1), k-2) = 0.0;
	}
	//Top and bottom of D
	vec3 p0 = keys[0].second;
	vec3 p1 = keys[1].second;
	D(0, 0) = 3.0 * (p1[0] - p0[0]);
	D(0, 1) = 3.0 * (p1[1] - p0[1]);
	D(0, 2) = 3.0 * (p1[2] - p0[2]);
	vec3 pN = keys[(numKeys - 1)].second;
	vec3 pNMinusOne = keys[(numKeys - 2)].second;
	D((numKeys - 1), 0) = 3.0 * (pN[0] - pNMinusOne[0]);
	D((numKeys - 1), 1) = 3.0 * (pN[1] - pNMinusOne[1]);
	D((numKeys - 1), 2) = 3.0 * (pN[2] - pNMinusOne[2]);

	//Initialize the rest of A and D
	for (int i = 1; i < (numKeys-1); i++)
	{
		//Row of A
		for (int m = 0; m < (i-1); m++)
		{
			A(i, m) = 0.0;
		}
		A(i, (i - 1)) = 1.0;
		A(i, i) = 4.0;
		A(i, (i + 1)) = 1.0;
		for (int j = (i+2); j < numKeys; j++)
		{
			A(i, j) = 0.0;
		}
		//Row of D
		vec3 pNMinusTwo = keys[i-1].second;
		vec3 pNN = keys[i+1].second;//So as not to be confused with pN in the code, still pN symbolically
		D(i, 0) = 3.0 * (pNN[0] - pNMinusTwo[0]);
		D(i, 1) = 3.0 * (pNN[1] - pNMinusTwo[1]);
		D(i, 2) = 3.0 * (pNN[2] - pNMinusTwo[2]);
		
	}

	//Calculate C
	Eigen::MatrixXd C = A.inverse()*D;

	//Extract control points
	for (int i = 0; i < numKeys; i++)
	{
		vec3 pNPrime;

		pNPrime.set(C(i,0), C(i,1), C(i,2));

		ctrlPoints.push_back(pNPrime);
	}

}



void ABSplineInterpolatorVec3::computeControlPoints(
    const std::vector<ASplineVec3::Key>& keys,
    std::vector<vec3>& ctrlPoints, 
    vec3& startPt, vec3& endPt)
{
    ctrlPoints.clear();
    if (keys.size() <= 1) return;

    // TODO: c
    // Hints: 
	// 1. use Eigen::MatrixXd to calculate the control points by solving the system of equations AC=D for C
	
    // 2. Create a recursive helper function dN(knots,n,t,l) to calculate derivative BSpline values at t, where
	//     knots = knot array
	//	   n = degree of the spline curves (n =3 for cubic)
	//     j = interval on knot vector in which to interpolate
	//     t = time value
	//     l = derivative (l = 1 => 1st derivative)

	// Step 1: Calculate knot vector using a uniform BSpline
	//         (assune knots are evenly spaced 1 apart and the start knot is at time = 0.0)
  
	// Step 2: Calculate A matrix  for a natural BSpline
	//         (Set 2nd derivative at t0 and tm to zero, where tm is the last point knot; m = #segments)

	// Step 3: Calculate  D matrix composed of our target points to interpolate

	// Step 4: Solve AC=D for C 
	
	// Step 5: save control points in ctrlPoints
	


	//Get m, number of partitions
	int m = keys.size() - 1;

	//Create knot vector
	//You could theoretically do this by just appending and prepending to a copy of keys- I'm not confident enough to try
	Eigen::MatrixXd KnotVector(1, (keys.size() + 6));
	//Initialize the first three and last three lambdas
	double t0 = keys[0].first;
	double t1 = keys[1].first;
	double tm = keys[m].first;
	double tmminusone = keys[(m - 1)].first;
	KnotVector(2) = t0 - (t1 - t0);
	KnotVector(1) = t0 - 2.0*(t1 - t0);
	KnotVector(0) = t0 - 3.0*(t1 - t0);
	KnotVector((m + 4)) = tm + (tm - tmminusone);
	KnotVector((m + 5)) = tm + 2.0*(tm - tmminusone);
	KnotVector((m + 6)) = tm + 3.0*(tm - tmminusone);
	//Extract the rest of the lambdas from the times
	for (int kk = 3; kk < keys.size()+3; kk++)
	{
		KnotVector(kk) = keys[(kk - 3)].first;
	}
	

	Eigen::MatrixXd Ab((m + 3), (m + 3));
	Eigen::MatrixXd Db((m + 3), 3);

	//Initialize top and bottom rows of A and D
	//Top and bottom of A
	Ab(0, 0) = dN(KnotVector, 3, 0, t0, 2);
	Ab(0, 1) = dN(KnotVector, 3, 1, t0, 2);
	Ab(0, 2) = dN(KnotVector, 3, 2, t0, 2);
	Ab(0, 3) = dN(KnotVector, 3, 3, t0, 2);
	Ab(((m + 3) - 1), ((m + 3) - 4)) = dN(KnotVector, 3, (m-1), tm, 2);
	Ab(((m + 3) - 1), ((m + 3) - 3)) = dN(KnotVector, 3, m, tm, 2);
	Ab(((m + 3) - 1), ((m + 3) - 2)) = dN(KnotVector, 3, (m+1), tm, 2);
	Ab(((m + 3) - 1), ((m + 3) - 1)) = dN(KnotVector, 3, (m+2), tm, 2);
	for (int k = 4; k < (m+3); k++)
	{
		Ab(0, k) = 0.0;
		Ab(((m+3) - 1), k - 4) = 0.0;
	}
	//Top and bottom of D
	Db(0, 0) = 0.0;
	Db(0, 1) = 0.0;
	Db(0, 2) = 0.0;
	Db(((m + 3) - 1), 0) = 0.0;
	Db(((m + 3) - 1), 1) = 0.0;
	Db(((m + 3) - 1), 2) = 0.0;

	//Initialize the rest of A and D
	for (int i = 1; i < ((m + 3) - 1); i++)
	{
		//Row of A
		
		Ab(i, (i - 1)) = dN(KnotVector, 3, (i - 1), keys[(i - 1)].first, 0);
		Ab(i, i) = dN(KnotVector, 3, i, keys[(i - 1)].first, 0);
		Ab(i, (i + 1)) = dN(KnotVector, 3, (i + 1), keys[(i - 1)].first, 0);
		if ((i + 2) == (m + 3)) {
			//Special case for second-to-last-row
			Ab(i, (i - 2)) = dN(KnotVector, 3, (i - 2), keys[(i - 1)].first, 0);
			for (int jj = 0; jj < (i - 2); jj++)
			{
				Ab(i, jj) = 0.0;
			}
		}
		else {
			Ab(i, (i + 2)) = dN(KnotVector, 3, (i + 2), keys[(i - 1)].first, 0);
			for (int jj = 0; jj < (i - 1); jj++)
			{
				Ab(i, jj) = 0.0;
			}
		}
		for (int j = (i + 3); j < (m + 3); j++)
		{
			Ab(i, j) = 0.0;
		}
		//Row of D
		vec3 pj = keys[(i - 1)].second;
		Db(i, 0) = pj[0];
		Db(i, 1) = pj[1];
		Db(i, 2) = pj[2];

	}

	//Calculate C
	Eigen::MatrixXd Cb = Ab.inverse()*Db;

	//std::cout << Ab << std::endl;

	//Extract control points
	for (int i = 0; i < m+3; i++)
	{
		vec3 cj;

		cj.set(Cb(i, 0), Cb(i, 1), Cb(i, 2));

		ctrlPoints.push_back(cj);
	}
}
