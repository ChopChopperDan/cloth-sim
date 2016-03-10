#ifdef SendMessage
#undef SendMessage
#endif


#include <RobotRaconteur.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

#include "edu__rpi__cats__utilities__clothsim.h"
#include "edu__rpi__cats__utilities__clothsim_stubskel.h"

#include <Windows.h>
#include <iostream>
#include <vector>
#include <string>
#include <cmath>

#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody\btSoftRigidDynamicsWorld.h"
#include "BulletSoftBody\btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletSoftBody\btSoftBodyHelpers.h"
#include "BulletSoftBody\btSoftBody.h"

#include <boost/enable_shared_from_this.hpp>
#include <map>

class ClothSimImpl : public edu::rpi::cats::utilities::clothsim::ClothSimulator, public boost::enable_shared_from_this < ClothSimImpl >
{
public:

	ClothSimImpl();
	~ClothSimImpl();

	ClothSimImpl(uint16_t nX, uint16_t nY, btScalar width, btScalar length, btScalar mass, btScalar stiffness, btScalar bending_stiffness, btScalar damping);
	void shutdown();

	int initWorldAndCloth();

	virtual RR_SHARED_PTR<edu::rpi::cats::utilities::clothsim::ClothState > getClothState();

	virtual void setGraspVelocities(RR_SHARED_PTR<edu::rpi::cats::utilities::clothsim::Velocity > v00, 
									RR_SHARED_PTR<edu::rpi::cats::utilities::clothsim::Velocity > v10,
									RR_SHARED_PTR<edu::rpi::cats::utilities::clothsim::Velocity > v01,
									RR_SHARED_PTR<edu::rpi::cats::utilities::clothsim::Velocity > v11);

private:
	double *x, *y, *z;
	double *fx, *fy, *fz;
	uint16_t numX, numY;
	uint32_t n_points;
	btScalar cloth_width, cloth_length, cloth_mass, cloth_stiffness, cloth_bending_stiffness, cloth_damping;
	double t;
	float dt;
	btVector3 v00, v10, v01, v11;

	boost::mutex mtx_;
	boost::thread t1;

	btBroadphaseInterface *broadphase;
	btCollisionConfiguration *collisionConfiguration;
	btCollisionDispatcher *dispatcher;
	btSequentialImpulseConstraintSolver *solver;
	btSoftRigidDynamicsWorld *world;
	btSoftBodyWorldInfo world_info;
	btSoftBody *cloth;

	void stepThroughSimulation();
};