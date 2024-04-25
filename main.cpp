// STL includes
#include <iostream>
#include <cstdarg>
#include <thread>

// sgd
#define SGD_DYNAMIC 1
#include "sgd/sgd.h"

// imgui
#include "imgui.h"
#define IMGUI_IMPL_SGD_IMPLEMENTATION 1
#include "sgd/imgui_impl_sgd.h"

// jolt
#include "Jolt/jolt.h"
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>
#include "physics.h"

int main(int, char**) {

	// sgd init
	sgd_Init();
	// sgd_CreateWindow(1280, 720, "SGD and Jolt Examples",0);
	sgd_CreateWindow(1920, 1080, "SGD and Jolt Examples", 0);
	sgd_SetWebGPUBackend("D3D12");
	sgd_CreateScene();

	// lights 
	SGD_Light mainlight = sgd_CreateDirectionalLight();
	SGD_Light p1 = sgd_CreatePointLight();
	sgd_MoveEntity(p1, 0, 5, 0);
	sgd_SetLightCastsShadow(p1, true);

	// camera
	SGD_Camera cam = sgd_CreatePerspectiveCamera();
	SGD_Model pivot = sgd_CreateModel();
	sgd_SetEntityParent(cam, pivot);
	sgd_MoveEntity(pivot, 0, 5, -20);
	sgd_TurnEntity(cam, -15, 0, 0);

	// floor
	SGD_Material floorMaterial = sgd_LoadPBRMaterial("assets/Planks037B_1K-JPG");
	SGD_Mesh floorMesh = sgd_CreateBoxMesh(-100, -1, -100, 100, 0, 100, floorMaterial);
	sgd_TransformMeshTexCoords(floorMesh, 16, 16, 0, 0);
	SGD_Model floorModel = sgd_CreateModel();
	sgd_SetModelMesh(floorModel, floorMesh);

	// spheres
	SGD_Material sphereMaterial1 = sgd_LoadPBRMaterial("assets/Marble006_1K-JPG");	
	SGD_Material sphereMaterial2= sgd_LoadPBRMaterial("assets/Marble020_1K-JPG");
	SGD_Material sphereMaterial3 = sgd_LoadPBRMaterial("assets/Marble022_1K-JPG");
	SGD_Material sphereMaterial4 = sgd_LoadPBRMaterial("assets/Tiles074_1K-JPG");
	SGD_Material sphereMaterial5 = sgd_LoadPBRMaterial("assets/Travertine013_1K-JPG");

	SGD_Mesh sphereMesh1 = sgd_CreateSphereMesh(0.5, 16, 16, sphereMaterial1);
	SGD_Mesh sphereMesh2 = sgd_CreateSphereMesh(0.5, 16, 16, sphereMaterial2);
	SGD_Mesh sphereMesh3 = sgd_CreateSphereMesh(0.5, 16, 16, sphereMaterial3);
	SGD_Mesh sphereMesh4 = sgd_CreateSphereMesh(0.5, 16, 16, sphereMaterial4);
	SGD_Mesh sphereMesh5 = sgd_CreateSphereMesh(0.5, 16, 16, sphereMaterial5);

	sgd_SetMeshCastsShadow(sphereMesh1, true);
	sgd_SetMeshCastsShadow(sphereMesh2, true);
	sgd_SetMeshCastsShadow(sphereMesh3, true);
	sgd_SetMeshCastsShadow(sphereMesh4, true);
	sgd_SetMeshCastsShadow(sphereMesh5, true);

	const int numspheres = 1000;
	SGD_Model spheres[numspheres];
	for (int i = 0; i < numspheres - 1; i++) {
		spheres[i] = sgd_CreateModel();
		int r = rand() % 5;
		if (r == 0) sgd_SetModelMesh(spheres[i], sphereMesh1);
		if (r == 1) sgd_SetModelMesh(spheres[i], sphereMesh2);
		if (r == 2) sgd_SetModelMesh(spheres[i], sphereMesh3);	
		if (r == 3) sgd_SetModelMesh(spheres[i], sphereMesh4);
		if (r == 4) sgd_SetModelMesh(spheres[i], sphereMesh5);
	}

	// runtime flags
	bool loop = true;

	// imgui init
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	auto& io = ImGui::GetIO();
	ImGui::StyleColorsDark();
	ImGui_ImplSGD_Init();
	bool show_demo = false;

	// Jolt init 
	JPH::RegisterDefaultAllocator();
	JPH::Factory::sInstance = new JPH::Factory();
	JPH::RegisterTypes();
	JPH::TempAllocatorImpl temp_allocator(10 * 1024 * 1024);
	JPH::JobSystemThreadPool job_system(JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers, JPH::thread::hardware_concurrency() - 1);
	const JPH::uint cMaxBodies = 1024;
	const JPH::uint cNumBodyMutexes = 0;
	const JPH::uint cMaxBodyPairs = 1024;
	const JPH::uint cMaxContactConstraints = 1024;

	BPLayerInterfaceImpl broad_phase_layer_interface;
	ObjectVsBroadPhaseLayerFilterImpl object_vs_broadphase_layer_filter;
	ObjectLayerPairFilterImpl object_vs_object_layer_filter;

	PhysicsSystem physics_system;
	physics_system.Init(cMaxBodies, cNumBodyMutexes, cMaxBodyPairs, cMaxContactConstraints, broad_phase_layer_interface, object_vs_broadphase_layer_filter, object_vs_object_layer_filter);
	
	MyBodyActivationListener body_activation_listener;
	physics_system.SetBodyActivationListener(&body_activation_listener);

	MyContactListener contact_listener;
	physics_system.SetContactListener(&contact_listener);

	BodyInterface& body_interface = physics_system.GetBodyInterface();
	BoxShapeSettings floor_shape_settings(Vec3(100.0f, 1.0f, 100.0f));
	floor_shape_settings.SetEmbedded(); 
	ShapeSettings::ShapeResult floor_shape_result = floor_shape_settings.Create();
	ShapeRefC floor_shape = floor_shape_result.Get();

	BodyCreationSettings floor_settings(floor_shape, RVec3(0.0_r, -1.0_r, 0.0_r), Quat::sIdentity(), EMotionType::Static, Layers::NON_MOVING);
	Body* floor = body_interface.CreateBody(floor_settings);
	body_interface.AddBody(floor->GetID(), EActivation::DontActivate);
	
	BodyID sphere_ids[numspheres];
	// Seed the random number generator
	srand(time(NULL));
	// Generate a random float between 0.0 and 1.0	
	for (int i = 0; i < numspheres - 1; i++) {
		float x = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
		float z = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
		BodyCreationSettings sphere_settings(new SphereShape(0.5f), RVec3(x,20+i, z), Quat::sIdentity(), EMotionType::Dynamic, Layers::MOVING);
		sphere_ids[i] = body_interface.CreateAndAddBody(sphere_settings, EActivation::Activate);
	}

	// body_interface.SetLinearVelocity(sphere_id, Vec3(0.0f, -5.0f, 0.0f));
	// body_interface.SetLinearVelocity(sphere2_id, Vec3(0.0f, -5.0f, 0.0f));

	const float cDeltaTime = 1.0f / 60.0f;
	physics_system.OptimizeBroadPhase();
	uint step = 0;

	while (loop) {
		// events
		int e = sgd_PollEvents();
		if (e == SGD_EVENT_MASK_CLOSE_CLICKED || sgd_KeyHit(SGD_KEY_ESCAPE)) loop = false;
		if (sgd_KeyHit(SGD_KEY_F1)) show_demo = !show_demo;
		
		// update 3D objects
		++step;
		for (int i = 0; i < numspheres - 1; i++) {
			Vec3 position = body_interface.GetCenterOfMassPosition(sphere_ids[i]);
			Quat rotation = body_interface.GetRotation(sphere_ids[i]);
			float sx = position.GetX(); float sy = position.GetY(); float sz = position.GetZ();
			float srx = rotation.GetX(); float sry = rotation.GetY(); float srz = rotation.GetZ(); float srw = rotation.GetW();
			sgd_SetEntityPosition(spheres[i], sx, sy, sz);
			sgd_SetEntityRotationQuat(spheres[i], srx, sry, srz, srw);
		}

		const int cCollisionSteps = 1;
		physics_system.Update(cDeltaTime, cCollisionSteps, &temp_allocator, &job_system);
	
		// render 3D objects
		sgd_RenderScene();

		// 2D overlay 
		// sgd_Clear2D();
		// sgd_Draw2DText("Hello SGD!", 10, 10);

		// ImGui
		ImGui_ImplSGD_NewFrame();
		ImGui::NewFrame();		
		if (show_demo) ImGui::ShowDemoWindow(&show_demo);
		ImGui::Begin("Main Console");
		ImGui::Text("Physics Step : %d", step);
		ImGui::Text("Bodies Activated : %d", bodies_activated);
		ImGui::Text("Bodies Deactivated : %d", bodies_deactivated);
		ImGui::Text("Contacts Activated : %d", contacts_activated);
		ImGui::Text("Contacts Persisted : %d", contacts_persisted);
		ImGui::Text("Contacts Removed : %d", contacts_removed);
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
		ImGui::End();
		ImGui::Render();
		sgd_ImGui_ImplSGD_RenderDrawData(ImGui::GetDrawData());

		// swap buffers
		sgd_Present();
	}
	// cleanup	
	// 
	// Jolt Stuff
	// Remove the sphere from the physics system. Note that the sphere itself keeps all of its state and can be re-added at any time.
	for (int i = 0; i < numspheres - 1; i++) {
		body_interface.RemoveBody(sphere_ids[i]);
		// Destroy the sphere. After this the sphere ID is no longer valid.
		body_interface.DestroyBody(sphere_ids[i]);
	}
	// Remove and destroy the floor
	body_interface.RemoveBody(floor->GetID());
	body_interface.DestroyBody(floor->GetID());
	JPH::UnregisterTypes();	
	delete JPH::Factory::sInstance;
	JPH::Factory::sInstance = nullptr;

	// imgui 
	ImGui_ImplSGD_Shutdown();
	ImGui::DestroyContext();

	// sgd
	sgd_ClearScene();
	sgd_Terminate();
	return 0;
}