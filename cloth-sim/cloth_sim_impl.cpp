#include "cloth_sim_impl.h"

ClothSimImpl::ClothSimImpl() : edu::rpi::cats::utilities::clothsim::ClothSimulator()
{
	this->numX = 0;
	this->numY = 0;
	this->n_points = 0;
	this->cloth_width = 0;
	this->cloth_length = 0;
	this->cloth_mass = 0;
	this->new_grasp_positions = false;
	this->grasp_positions_updated = false;

	this->dt = 1.f / 120.f;
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
	this->new_grasp_positions = false;
	this->grasp_positions_updated = false;


	this->dt = 1.f / 120.f;

	if (initWorldAndCloth() > 0)
	{
		this->x = new double[this->cloth->m_nodes.size()];
		this->y = new double[this->cloth->m_nodes.size()];
		this->z = new double[this->cloth->m_nodes.size()];
		this->fx = new double[this->cloth->m_nodes.size()];
		this->fy = new double[this->cloth->m_nodes.size()];
		this->fz = new double[this->cloth->m_nodes.size()];
		this->th1 = boost::thread(boost::bind(&ClothSimImpl::stepThroughSimulation, this));
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
	this->world_info.m_maxDisplacement = 2.f * this->dt;
	this->world_info.m_sparsesdf.Initialize();
	std::cout << "Generating cloth" << std::endl;

	cloth = btSoftBodyHelpers::CreatePatch(world_info, btVector3(-this->cloth_width / 2.f, -this->cloth_length / 2.f, 0),
														btVector3(this->cloth_width / 2.f, -this->cloth_length / 2.f, 0),
														btVector3(-this->cloth_width / 2.f, this->cloth_length / 2.f, 0),
														btVector3(this->cloth_width / 2.f, this->cloth_length / 2.f, 0),
															this->numX, this->numY, 0, true);
	std::cout << "SoftBodyHelper generated cloth with..." << std::endl;
	std::cout << "Nodes: " << cloth->m_nodes.size() << std::endl;
	std::cout << "Links: " << cloth->m_links.size() << std::endl;
	
	cloth->getCollisionShape()->setMargin(0.001f);

	btSoftBody::Material *structural_material = cloth->appendMaterial();
	structural_material->m_kLST = this->cloth_stiffness;
	structural_material->m_flags -= btSoftBody::fMaterial::DebugDraw;
	for (int i = 0; i < cloth->m_links.size(); i++)
		cloth->m_links.at(i).m_material = structural_material;

	btSoftBody::Material *bending_material = cloth->appendMaterial();
	bending_material->m_kLST = this->cloth_bending_stiffness;
	bending_material->m_kAST = 1;
	bending_material->m_kVST = 1;
	bending_material->m_flags -= btSoftBody::fMaterial::DebugDraw;
	cloth->generateBendingConstraints(2, bending_material);
	std::cout << "After adding bending constraints..." << std::endl;
	std::cout << "Links: " << cloth->m_links.size() << std::endl;
	

	cloth->updateConstants();

	double grasp_radius = 0.04f;

	for (int i = 0; i < this->cloth->m_nodes.size(); i++)
	{
		// testing within grasp00 origin
		if ((this->cloth->m_nodes.at(i).m_x - this->cloth->m_nodes.at(0).m_x).length() <= grasp_radius)
		{
			node_relationship this_node;
			this_node.index = i;
			this_node.vec = (this->cloth->m_nodes.at(i).m_x - this->cloth->m_nodes.at(0).m_x);
			this_node.x_des = this->cloth->m_nodes.at(i).m_x;
			grasp00_nodes.push_back(this_node);
			this->cloth->m_nodes.at(i).m_im = 0.f;
		}
		// testing within grasp10 origin
		if ((this->cloth->m_nodes.at(i).m_x - this->cloth->m_nodes.at(this->numX - 1).m_x).length() <= grasp_radius)
		{
			node_relationship this_node;
			this_node.index = i;
			this_node.vec = (this->cloth->m_nodes.at(i).m_x - this->cloth->m_nodes.at(this->numX - 1).m_x);
			this_node.x_des = this->cloth->m_nodes.at(i).m_x;
			grasp10_nodes.push_back(this_node);
			this->cloth->m_nodes.at(i).m_im = 0.f;
		}
		// testing within grasp01 origin
		if ((this->cloth->m_nodes.at(i).m_x - this->cloth->m_nodes.at((this->numY - 1)*this->numX).m_x).length() <= grasp_radius)
		{
			node_relationship this_node;
			this_node.index = i;
			this_node.vec = (this->cloth->m_nodes.at(i).m_x - this->cloth->m_nodes.at((this->numY - 1)*this->numX).m_x);
			this_node.x_des = this->cloth->m_nodes.at(i).m_x;
			grasp01_nodes.push_back(this_node);
			this->cloth->m_nodes.at(i).m_im = 0.f;
		}
		// testing within grasp11 origin
		if ((this->cloth->m_nodes.at(i).m_x - this->cloth->m_nodes.at(this->numY*this->numX - 1).m_x).length() <= grasp_radius)
		{
			node_relationship this_node;
			this_node.index = i;
			this_node.vec = (this->cloth->m_nodes.at(i).m_x - this->cloth->m_nodes.at(this->numY*this->numX - 1).m_x);
			this_node.x_des = this->cloth->m_nodes.at(i).m_x;
			grasp11_nodes.push_back(this_node);
			this->cloth->m_nodes.at(i).m_im = 0.f;
		}
	}


	cloth->setTotalMass(this->cloth_mass);
	cloth->m_cfg.kDP = this->cloth_damping;
	cloth->m_cfg.piterations = 5;
	cloth->updateConstants();

	world->addSoftBody(cloth);

	return cloth->m_nodes.size();
}

void ClothSimImpl::shutdown()
{
	if (broadphase == 0)
		return;

	th1.interrupt();
	th1.join();

	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

	if (world == 0)
		return;

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

RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > ClothSimImpl::get_grasped_nodes00()
{
	boost::lock_guard<boost::mutex> guard(mtx_);
	RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > g;
	uint16_t *g_nodes = new uint16_t[this->grasp00_nodes.size()];
	for (unsigned int i = 0; i < this->grasp00_nodes.size(); i++)
		g_nodes[i] = this->grasp00_nodes.at(i).index;

	g = RobotRaconteur::AttachRRArray<uint16_t>(g_nodes, this->grasp00_nodes.size(), true);

	return g;
}
void ClothSimImpl::set_grasped_nodes00(RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > value)
{
	boost::lock_guard<boost::mutex> guard(mtx_);
}

RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > ClothSimImpl::get_grasped_nodes10()
{
	boost::lock_guard<boost::mutex> guard(mtx_);
	RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > g;
	uint16_t *g_nodes = new uint16_t[this->grasp10_nodes.size()];
	for (unsigned int i = 0; i < this->grasp10_nodes.size(); i++)
		g_nodes[i] = this->grasp10_nodes.at(i).index;

	g = RobotRaconteur::AttachRRArray<uint16_t>(g_nodes, this->grasp10_nodes.size(), true);

	return g;
}
void ClothSimImpl::set_grasped_nodes10(RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > value)
{
	boost::lock_guard<boost::mutex> guard(mtx_);
}

RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > ClothSimImpl::get_grasped_nodes01()
{
	boost::lock_guard<boost::mutex> guard(mtx_);
	RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > g;
	uint16_t *g_nodes = new uint16_t[this->grasp01_nodes.size()];
	for (unsigned int i = 0; i < this->grasp01_nodes.size(); i++)
		g_nodes[i] = this->grasp01_nodes.at(i).index;

	g = RobotRaconteur::AttachRRArray<uint16_t>(g_nodes, this->grasp01_nodes.size(), true);

	return g;
}
void ClothSimImpl::set_grasped_nodes01(RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > value)
{
	boost::lock_guard<boost::mutex> guard(mtx_);
}

RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > ClothSimImpl::get_grasped_nodes11()
{
	boost::lock_guard<boost::mutex> guard(mtx_);
	RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > g;
	uint16_t *g_nodes = new uint16_t[this->grasp11_nodes.size()];
	for (unsigned int i = 0; i < this->grasp11_nodes.size(); i++)
		g_nodes[i] = this->grasp11_nodes.at(i).index;

	g = RobotRaconteur::AttachRRArray<uint16_t>(g_nodes, this->grasp11_nodes.size(), true);

	return g;
}
void ClothSimImpl::set_grasped_nodes11(RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > value)
{
	boost::lock_guard<boost::mutex> guard(mtx_);
}

RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > ClothSimImpl::getFaceStructure()
{
	boost::lock_guard<boost::mutex> guard(mtx_);


	RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > F;
	uint16_t *face_nodes = new uint16_t[3 * this->cloth->m_faces.size()];
	btSoftBody::Node *node_address0 = &this->cloth->m_nodes.at(0);

	for (int i = 0; i < this->cloth->m_faces.size(); i++)
	{
		face_nodes[3 * i] = this->cloth->m_faces.at(i).m_n[0] - node_address0;
		face_nodes[3 * i + 1] = this->cloth->m_faces.at(i).m_n[1] - node_address0;
		face_nodes[3 * i + 2] = this->cloth->m_faces.at(i).m_n[2] - node_address0;
	}

	F = RobotRaconteur::AttachRRArray<uint16_t>(face_nodes, 3 * this->cloth->m_faces.size(), true);

	return F;
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

void ClothSimImpl::setGraspPoses(RR_SHARED_PTR<edu::rpi::cats::utilities::clothsim::Pose > p00,
								RR_SHARED_PTR<edu::rpi::cats::utilities::clothsim::Pose > p10,
								RR_SHARED_PTR<edu::rpi::cats::utilities::clothsim::Pose > p01,
								RR_SHARED_PTR<edu::rpi::cats::utilities::clothsim::Pose > p11)
{

	setNewGraspPositions(p00, this->grasp00_nodes);
	setNewGraspPositions(p10, this->grasp10_nodes);
	setNewGraspPositions(p01, this->grasp01_nodes);
	setNewGraspPositions(p11, this->grasp11_nodes);
	this->new_grasp_positions = true;
}


void ClothSimImpl::setNewGraspPositions(RR_SHARED_PTR<edu::rpi::cats::utilities::clothsim::Pose > p, std::vector<node_relationship> &grasp_nodes)
{
	btMatrix3x3 Rot(p->R->ptr()[0], p->R->ptr()[1], p->R->ptr()[2],
					p->R->ptr()[3], p->R->ptr()[4], p->R->ptr()[5],
					p->R->ptr()[6], p->R->ptr()[7], p->R->ptr()[8]);
	btVector3 Tran(p->p->ptr()[0], p->p->ptr()[1], p->p->ptr()[2]);

	for (unsigned int i = 0; i < grasp_nodes.size(); i++)
		grasp_nodes.at(i).x_des = Tran + Rot * grasp_nodes.at(i).vec;
	
}

void ClothSimImpl::updateGraspPoints(std::vector<node_relationship> grasp_nodes, btScalar time_step)
{
	for (unsigned int i = 0; i < grasp_nodes.size(); i++)
	{
		int idx = grasp_nodes.at(i).index;
		this->cloth->m_nodes.at(idx).m_v = (grasp_nodes.at(i).x_des - this->cloth->m_nodes.at(idx).m_x) / time_step;
		this->cloth->m_nodes.at(idx).m_x = grasp_nodes.at(i).x_des;
	}
}

void ClothSimImpl::resetGraspVelocities(std::vector<node_relationship> grasp_nodes)
{
	for (unsigned int i = 0; i < grasp_nodes.size(); i++)
		this->cloth->m_nodes.at(grasp_nodes.at(i).index).m_v = btVector3(0.f, 0.f, 0.f);
}

void ClothSimImpl::solveSpringForces(btScalar structural_stiffness, btScalar bending_stiffness)
{
	// clear out current force magnitudes
	for (int i = 0; i < this->n_points; i++)
		this->fx[i] = this->fy[i] = this->fz[i] = 0.0;

	for (int i = 0; i < this->cloth->m_links.size(); i++)
	{
		btScalar k = this->cloth->m_links.at(i).m_bbending ? bending_stiffness : structural_stiffness;
		int a = this->cloth->m_links.at(i).m_n[0] - &this->cloth->m_nodes.at(0);
		int b = this->cloth->m_links.at(i).m_n[1] - &this->cloth->m_nodes.at(0);
		btVector3 dx = this->cloth->m_links.at(i).m_n[1]->m_x - this->cloth->m_links.at(i).m_n[0]->m_x;
		btVector3 f = k*(1.f - this->cloth->m_links.at(i).m_rl / dx.norm())*dx;
		this->fx[a] += f.x();
		this->fy[a] += f.y();
		this->fz[a] += f.z();
		this->fx[b] += -f.x();
		this->fy[b] += -f.y();
		this->fz[b] += -f.z();
	}
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
			
			maxSubSteps = std::max(1, (int)ceil(time_step / this->dt));
			if (maxSubSteps > 20)
			{
				std::cout << "Simulation step exceeds maximum." << std::endl;
				break;
			}

			// If flagged, set grasped nodes to new positions
			if (time_step > 0.f && this->new_grasp_positions && !this->grasp_positions_updated)
			{
				//std::cout << "Update time step: " << time_step << std::endl;
				updateGraspPoints(this->grasp00_nodes, time_step);
				updateGraspPoints(this->grasp10_nodes, time_step);
				updateGraspPoints(this->grasp01_nodes, time_step);
				updateGraspPoints(this->grasp11_nodes, time_step);
				this->new_grasp_positions = false;
				this->grasp_positions_updated = true;
			}
			{
				boost::lock_guard<boost::mutex> guard(mtx_);

				this->world->stepSimulation(time_step, maxSubSteps, this->dt);
			}

			// Reset grasped node velocities back to zero
			if (this->grasp_positions_updated)
			{
				resetGraspVelocities(this->grasp00_nodes);
				resetGraspVelocities(this->grasp10_nodes);
				resetGraspVelocities(this->grasp01_nodes);
				resetGraspVelocities(this->grasp11_nodes);
				this->grasp_positions_updated = false;
			}

			// Update local variables storing the mesh positions
			{
				boost::lock_guard<boost::mutex> guard(mtx_);

				for (unsigned int i = 0; i < this->n_points; i++)
				{
					this->x[i] = this->cloth->m_nodes.at(i).m_x.x();
					this->y[i] = this->cloth->m_nodes.at(i).m_x.y();
					this->z[i] = this->cloth->m_nodes.at(i).m_x.z();
				}
				solveSpringForces(10.0, 0.1);
			}
			this->t = (t1 - t0).total_microseconds() / 1000000.f;
			boost::this_thread::sleep(boost::posix_time::milliseconds((uint64_t)(this->dt * 2000.f)));
			boost::this_thread::interruption_point();
		}
		catch (...) {
			break;
		}
	}

	std::cout << "Exiting background thread" << std::endl;
}