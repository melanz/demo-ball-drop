#include "common/common.h"

template<class T>
void RunTimeStep(T* mSys, const int frame) {
	// add code here that you want to execute at each timestep
}

int main(int argc, char* argv[]) 
{
	bool visualize = true;
	int threads = 8;
	int config = 0;
	real gravity = -9.81;			//acceleration due to gravity
	real timestep = .01;			//step size
	real time_to_run = 1;			//length of simulation
	real current_time = 0;

	int num_steps = time_to_run / timestep;
	int max_iteration = 15;
	int tolerance = 0;

	//=========================================================================================================
	// Create system
	//=========================================================================================================
	ChSystemParallel * system_gpu = new ChSystemParallel;

	//=========================================================================================================
	// Populate the system with bodies/constraints/forces/etc.
	//=========================================================================================================
	ChVector<> lpos(0, 0, 0);
	ChQuaternion<> quat(1, 0, 0, 0);
	real container_width = 5;		//width of area with particles
	real container_length = 25;		//length of area that roller will go over
	real container_thickness = .25;     	//thickness of container walls
	real container_height = 2;		//height of the outer walls
	real particle_radius = .58;

	// Create a material (will be used by both objects)
	ChSharedPtr<ChMaterialSurface> material;
	material = ChSharedPtr<ChMaterialSurface>(new ChMaterialSurface);
	material->SetFriction(0.4);

	// Create a ball
	ChSharedBodyPtr ball = ChSharedBodyPtr(new ChBody(new ChCollisionModelParallel));
	InitObject(ball, 
		1, 				// mass
		ChVector<>(0, 10, 0), 		// position
		ChQuaternion<>(1, 0, 0, 0), 	// rotation
		material, 			// material
		true, 				// collide?
		false, 				// static?
		-15, -15);			// collision family
	ball->SetPos_dt(ChVector<>(0,0,10));
	AddCollisionGeometry(ball, SPHERE, particle_radius, lpos, quat);
	FinalizeObject(ball, (ChSystemParallel *) system_gpu);

	// Create a bin for the ball to fall into
	ChSharedBodyPtr bin = ChSharedBodyPtr(new ChBody(new ChCollisionModelParallel));
	InitObject(bin, 
		1, 				// mass
		ChVector<>(0, 0, 0), 		// position
		ChQuaternion<>(1, 0, 0, 0), 	// rotation
		material, 			// material
		true, 				// collide?
		true, 				// static?
		-20, -20); 			// collision family
	AddCollisionGeometry(bin, BOX, ChVector<>(container_width, container_thickness, container_length), lpos, quat);
	AddCollisionGeometry(bin, BOX, Vector(container_thickness, container_height, container_length), Vector(-container_width + container_thickness, container_height, 0), quat);
	AddCollisionGeometry(bin, BOX, Vector(container_thickness, container_height, container_length), Vector(container_width - container_thickness, container_height, 0), quat);
	AddCollisionGeometry(bin, BOX, Vector(container_width, container_height, container_thickness), Vector(0, container_height, -container_length + container_thickness), quat);
	AddCollisionGeometry(bin, BOX, Vector(container_width, container_height, container_thickness), Vector(0, container_height, container_length - container_thickness), quat);
	FinalizeObject(bin, (ChSystemParallel *) system_gpu);

	//=========================================================================================================
	// Edit system settings
	//=========================================================================================================
	system_gpu->SetIntegrationType(ChSystem::INT_ANITESCU);
	system_gpu->SetParallelThreadNumber(threads);
	system_gpu->SetMaxiter(max_iteration);
	system_gpu->SetIterLCPmaxItersSpeed(max_iteration);
	system_gpu->SetTol(1e-3);
	system_gpu->SetTolSpeeds(1e-3);
	system_gpu->Set_G_acc(ChVector<>(0, gravity, 0));
	system_gpu->SetStep(timestep);

	((ChLcpSolverParallel *) (system_gpu->GetLcpSolverSpeed()))->SetMaxIteration(max_iteration);
	((ChLcpSolverParallel *) (system_gpu->GetLcpSolverSpeed()))->SetTolerance(0);
	((ChLcpSolverParallel *) (system_gpu->GetLcpSolverSpeed()))->SetCompliance(0, 0, 0);
	((ChLcpSolverParallel *) (system_gpu->GetLcpSolverSpeed()))->SetContactRecoverySpeed(300);
	((ChLcpSolverParallel *) (system_gpu->GetLcpSolverSpeed()))->SetSolverType(ACCELERATED_PROJECTED_GRADIENT_DESCENT);

	((ChCollisionSystemParallel *) (system_gpu->GetCollisionSystem()))->SetCollisionEnvelope(particle_radius * .05);
	((ChCollisionSystemParallel *) (system_gpu->GetCollisionSystem()))->setBinsPerAxis(R3(10, 10, 10));
	((ChCollisionSystemParallel *) (system_gpu->GetCollisionSystem()))->setBodyPerBin(100, 50);

	omp_set_num_threads(threads);

	//=========================================================================================================
	// Enter the time loop and render the simulation
	//=========================================================================================================
	if (visualize) {
		ChOpenGLManager * window_manager = new ChOpenGLManager();
		ChOpenGL openGLView(window_manager, system_gpu, 800, 600, 0, 0, "Test_Solvers");
		openGLView.render_camera->camera_pos = Vector(0, 5, -20);
		openGLView.render_camera->look_at = Vector(0, 0, 0);
		openGLView.SetCustomCallback(RunTimeStep);
		openGLView.StartSpinning(window_manager);
		window_manager->CallGlutMainLoop();
	}

	return 0;
}

