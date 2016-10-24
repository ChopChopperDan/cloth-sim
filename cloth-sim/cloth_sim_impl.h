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

struct node_relationship
{
	int index;
	btVector3 vec;
	btVector3 x_des;
};

class ClothSimImpl : public edu::rpi::cats::utilities::clothsim::ClothSimulator, public boost::enable_shared_from_this < ClothSimImpl >
{
public:

	ClothSimImpl();
	~ClothSimImpl();

	ClothSimImpl(uint16_t nX, uint16_t nY, btScalar width, btScalar length, btScalar mass, btScalar stiffness, btScalar bending_stiffness, btScalar damping);
	void shutdown();

	virtual RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > get_grasped_nodes00();
	virtual void set_grasped_nodes00(RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > value);

	virtual RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > get_grasped_nodes10();
	virtual void set_grasped_nodes10(RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > value);

	virtual RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > get_grasped_nodes01();
	virtual void set_grasped_nodes01(RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > value);

	virtual RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > get_grasped_nodes11();
	virtual void set_grasped_nodes11(RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > value);

	int initWorldAndCloth();

	virtual RR_SHARED_PTR<edu::rpi::cats::utilities::clothsim::ClothState > getClothState();

	virtual void setGraspPoses(RR_SHARED_PTR<edu::rpi::cats::utilities::clothsim::Pose > p00,
									RR_SHARED_PTR<edu::rpi::cats::utilities::clothsim::Pose > p10,
									RR_SHARED_PTR<edu::rpi::cats::utilities::clothsim::Pose > p01,
									RR_SHARED_PTR<edu::rpi::cats::utilities::clothsim::Pose > p11);

	virtual RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > getFaceStructure();

	virtual void setCameraPose(RR_SHARED_PTR<edu::rpi::cats::utilities::clothsim::Pose > pk);
	virtual RR_SHARED_PTR<edu::rpi::cats::utilities::clothsim::DepthImage > getRenderedImage();

private:
	double *x, *y, *z;
	double *fx, *fy, *fz;
	uint16_t numX, numY;
	uint32_t n_points;
	btScalar cloth_width, cloth_length, cloth_mass, cloth_stiffness, cloth_bending_stiffness, cloth_damping;
	double t;
	float dt;
	bool new_grasp_positions, grasp_positions_updated;
	std::vector<node_relationship> grasp00_nodes, grasp10_nodes, grasp01_nodes, grasp11_nodes;
	float *image_data;
	uint16_t image_height, image_width;
	btMatrix3x3 ROk, K;
	btVector3 pOk_O;
	

	boost::mutex mtx_;
	boost::thread th1;


	btBroadphaseInterface *broadphase;
	btCollisionConfiguration *collisionConfiguration;
	btCollisionDispatcher *dispatcher;
	btSequentialImpulseConstraintSolver *solver;
	btSoftRigidDynamicsWorld *world;
	btSoftBodyWorldInfo world_info;
	btSoftBody *cloth;

	void stepThroughSimulation();
	void setNewGraspPositions(RR_SHARED_PTR<edu::rpi::cats::utilities::clothsim::Pose > p, std::vector<node_relationship> &grasp_nodes);
	void updateGraspPoints(std::vector<node_relationship> grasp_nodes, btScalar time_step);
	void resetGraspVelocities(std::vector<node_relationship> grasp_nodes);
	void solveSpringForces(btScalar structural_stiffness, btScalar bending_stiffness);
	void renderDepthImage();
};