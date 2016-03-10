#include "cloth_sim_impl.h"

ClothSimImpl::ClothSimImpl() : edu::rpi::cats::utilities::clothsim::ClothSimulator()
{
	this->numX = 0;
	this->numY = 0;
	this->n_points = 0;
	this->cloth_width = 0;
	this->cloth_length = 0;
	this->cloth_mass = 0;

	this->v00 = btVector3(0.0f, 0.0f, 0.0f);
	this->v10 = btVector3(0.0f, 0.0f, 0.0f);
	this->v01 = btVector3(0.0f, 0.0f, 0.0f);
	this->v11 = btVector3(0.0f, 0.0f, 0.0f);

	this->dt = 1.f / 60.f;
}

ClothSimImpl::ClothSimImpl(uint16_t nX, uint16_t nY, btScalar width, btScalar length, btScalar mass, btScalar stiffness, btScalar bending_stiffness, btScalar damping) : edu::rpi::cats::utilities::clothsim::ClothSimulator()
{
	std::cout << "Generating object" << std::endl;
	this->numX = nX;
	this->numY = nY;
	this->n_points = ((uint32_t)nX)*((uint32_t)nY);
	this->cloth_width = width;
	this->cloth_length = length;
	this->cloth_mass = mass;
	this->cloth_stiffness = stiffness;
	this->cloth_bending_stiffness = bending_stiffness;
	this->cloth_damping = damping;

	this->v00 = btVector3(0.0f, 0.0f, 0.0f);
	this->v10 = btVector3(0.0f, 0.0f, 0.0f);
	this->v01 = btVector3(0.0f, 0.0f, 0.0f);
	this->v11 = btVector3(0.0f, 0.0f, 0.0f);

	this->dt = 1.f / 60.f;

	if (initWorldAndCloth() > 0)
	{
		this->x = new double[this->cloth->m_nodes.size()];
		this->y = new double[this->cloth->m_nodes.size()];
		this->z = new double[this->cloth->m_nodes.size()];
		this->fx = new double[this->cloth->m_nodes.size()];
		this->fy = new double[this->cloth->m_nodes.size()];
		this->fz = new double[this->cloth->m_nodes.size()];
		t1 = boost::thread(boost::bind(&ClothSimImpl::stepThroughSimulation, this));
	}
	else
		std::cout << "Something went wrong and things aren't simulating" << std::endl;
}

ClothSimImpl::~ClothSimImpl()
{
	shutdown();
}

int ClothSimImpl::initWorldAndCloth()
{
	std::cout << "Allocating dynamic world parameters (broadphase, dispatcher, solver)" << std::endl;
	// Using a Dynamic AABB Tree broadphase algorithm
	this->broadphase = new btDbvtBroadphase();
	// Using a Sweep and Prune broadphase algorithm
	//this->broadphase = new btAxisSweep3(btVector3(-100, -100, -100), btVector3(100, 100, 100));

	this->collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();
	this->dispatcher = new btCollisionDispatcher(collisionConfiguration);
	// Default constraint solver
	this->solver = new btSequentialImpulseConstraintSolver;

	std::cout << "Generating world" << std::endl;

	this->world = new btSoftRigidDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
	this->world->setGravity(btVector3(0, 0, -9.81f));
	this->world_info = this->world->getWorldInfo();
	this->world_info.m_gravity = this->world->getGravity();
	this->world_info.m_sparsesdf.Initialize();

	std::cout << "Generating cloth" << std::endl;

	cloth = btSoftBodyHelpers::CreatePatch(world_info, btVector3(-this->cloth_width / 2.f, -this->cloth_length / 2.f, 0),
														btVector3(this->cloth_width / 2.f, -this->cloth_length / 2.f, 0),
														btVector3(-this->cloth_width / 2.f, this->cloth_length / 2.f, 0),
														btVector3(this->cloth_width / 2.f, this->cloth_length / 2.f, 0),
															this->numX, this->numY, 1 + 2 + 4 + 8, true);
	std::cout << "Helper generated cloth with..." << std::endl;
	std::cout << "Nodes: " << cloth->m_nodes.size() << std::endl;
	std::cout << "Links: " << cloth->m_links.size() << std::endl;
	
	cloth->getCollisionShape()->setMargin(0.05);

	btSoftBody::Material *structural_material = cloth->appendMaterial();
	structural_material->m_kLST = this->cloth_stiffness;
	structural_material->m_flags -= btSoftBody::fMaterial::DebugDraw;
	for (int i = 0; i < cloth->m_links.size(); i++)
		cloth->m_links.at(i).m_material = structural_material;

	btSoftBody::Material *bending_material = cloth->appendMaterial();
	bending_material->m_kLST = this->cloth_bending_stiffness;
	bending_material->m_flags -= btSoftBody::fMaterial::DebugDraw;
	cloth->generateBendingConstraints(2, bending_material);
	
	cloth->setTotalMass(this->cloth_mass);
	cloth->m_cfg.kDP = this->cloth_damping;


	world->addSoftBody(cloth);

	return cloth->m_nodes.size();
}

void ClothSimImpl::shutdown()
{
	if (broadphase == 0)
		return;

	t1.interrupt();
	t1.join();

	if (world != NULL && cloth != NULL)
		world->removeSoftBody(cloth);
	delete cloth;

	delete world; 
	delete solver;
	delete dispatcher;
	delete collisionConfiguration;
	delete broadphase;
	world = 0;
	solver = 0;
	dispatcher = 0;
	collisionConfiguration = 0;
	broadphase = 0;
}

RR_SHARED_PTR<edu::rpi::cats::utilities::clothsim::ClothState > ClothSimImpl::getClothState()
{
	boost::lock_guard<boost::mutex> guard(mtx_);

	RR_SHARED_PTR<edu::rpi::cats::utilities::clothsim::ClothState> CState(new edu::rpi::cats::utilities::clothsim::ClothState());

	CState->numX = this->numX;
	CState->numY = this->numY;
	CState->n_points = this->n_points;
	CState->t = this->t;
	CState->x = RobotRaconteur::AttachRRArrayCopy<double>(this->x, this->n_points);
	CState->y = RobotRaconteur::AttachRRArrayCopy<double>(this->y, this->n_points);
	CState->z = RobotRaconteur::AttachRRArrayCopy<double>(this->z, this->n_points);
	CState->fx = RobotRaconteur::AttachRRArrayCopy<double>(this->fx, this->n_points);
	CState->fy = RobotRaconteur::AttachRRArrayCopy<double>(this->fy, this->n_points);
	CState->fz = RobotRaconteur::AttachRRArrayCopy<double>(this->fz, this->n_points);

	return CState;
}

void ClothSimImpl::setGraspVelocities(RR_SHARED_PTR<edu::rpi::cats::utilities::clothsim::Velocity > v00,
													RR_SHARED_PTR<edu::rpi::cats::utilities::clothsim::Velocity > v10,
													RR_SHARED_PTR<edu::rpi::cats::utilities::clothsim::Velocity > v01,
													RR_SHARED_PTR<edu::rpi::cats::utilities::clothsim::Velocity > v11)
{
	this->v00.setX(v00->vx);
	this->v00.setY(v00->vy);
	this->v00.setZ(v00->vz);

	this->v10.setX(v10->vx);
	this->v10.setY(v10->vy);
	this->v10.setZ(v10->vz);

	this->v01.setX(v01->vx);
	this->v01.setY(v01->vy);
	this->v01.setZ(v01->vz);

	this->v11.setX(v11->vx);
	this->v11.setY(v11->vy);
	this->v11.setZ(v11->vz);
}

void ClothSimImpl::stepThroughSimulation()
{
	std::cout << "Starting up background thread" << std::endl;
	boost::posix_time::ptime t0, t1;
	boost::posix_time::time_duration loop_time;
	btScalar time_step;
	int maxSubSteps;

	t0 = boost::posix_time::microsec_clock::local_time();
	t1 = boost::posix_time::microsec_clock::local_time();
	// Step through simulation and record data
	while (true)
	{
		try
		{
			loop_time = boost::posix_time::microsec_clock::local_time() - t1;
			t1 = boost::posix_time::microsec_clock::local_time();
			time_step = ((float)loop_time.total_microseconds()) / 1000000.f;

			// Manually move the grasped points
			this->cloth->m_nodes.at(0).m_x += this->v00 * time_step;
			this->cloth->m_nodes.at(0).m_v = this->v00;
						
			this->cloth->m_nodes.at(this->numX - 1).m_x += this->v10 * time_step;
			this->cloth->m_nodes.at(this->numX - 1).m_v = this->v10;
			
			this->cloth->m_nodes.at((this->numY - 1) * this->numX).m_x += this->v01 * time_step;
			this->cloth->m_nodes.at((this->numY - 1) * this->numX).m_v = this->v01;

			this->cloth->m_nodes.at(this->numY * this->numX - 1).m_x += this->v11 * time_step;
			this->cloth->m_nodes.at(this->numY * this->numX - 1).m_v = this->v11;

			maxSubSteps = std::max(1.f, ceil(time_step / this->dt));
			if (maxSubSteps > 20)
			{
				std::cout << "Simulation step exceeds maximum." << std::endl;
				break;
			}

			this->world->stepSimulation(time_step, maxSubSteps, this->dt);

			{
				boost::lock_guard<boost::mutex> guard(mtx_);

				for (int i = 0; i < this->n_points; i++)
				{
					this->x[i] = this->cloth->m_nodes.at(i).m_x.x();
					this->y[i] = this->cloth->m_nodes.at(i).m_x.y();
					this->z[i] = this->cloth->m_nodes.at(i).m_x.z();

					this->fx[i] = this->cloth->m_nodes.at(i).m_f.x();
					this->fy[i] = this->cloth->m_nodes.at(i).m_f.y();
					this->fz[i] = this->cloth->m_nodes.at(i).m_f.z();
				}
			}
			this->t = (t1 - t0).total_microseconds() / 1000000.f;
			std::cout << this->t << ": " << time_step << ", " << maxSubSteps << std::endl;


			boost::this_thread::interruption_point();
		}
		catch (...) {
			break;
		}
	}

	std::cout << "Exiting background thread" << std::endl;
}