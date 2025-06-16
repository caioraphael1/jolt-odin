package hello_world


import jolt ".."

import "core:fmt"
import "base:runtime"

import rl "vendor:raylib"


Layers :: enum {
	Non_Moving = 0,
	Moving = 1,
	Num_Layers = 2,
}


BroadPhaseLayers :: enum {
	Non_Moving = 0,
	Moving = 1,
	Num_Layers = 2,
}


trace_impl :: proc "c" (message: cstring) {
	context = runtime.default_context()
	fmt.println("TRACE: %v", message)
}


main :: proc() {
    if !jolt.Init() {
		fmt.println("Not initialized.")
		return
	}
    defer jolt.Shutdown()

	jolt.SetTraceHandler(trace_impl)

	job_system := jolt.JobSystemThreadPool_Create(nil)
	defer jolt.JobSystem_Destroy(job_system)

    // We use only 2 layers: one for non-moving objects and one for moving objects
    object_layer_pair_filter := jolt.ObjectLayerPairFilterTable_Create(2)
	jolt.ObjectLayerPairFilterTable_EnableCollision(object_layer_pair_filter, u32(Layers.Moving),     u32(Layers.Non_Moving))
		// Spheres collide with the floor.
	jolt.ObjectLayerPairFilterTable_EnableCollision(object_layer_pair_filter, u32(Layers.Non_Moving), u32(Layers.Moving))
		// Floor collide with the spheres.
		// Note: I don't know if this is necessary. 
	jolt.ObjectLayerPairFilterTable_EnableCollision(object_layer_pair_filter, u32(Layers.Moving), u32(Layers.Moving))
		// Spheres collide with the spheres.

	// We use a 1-to-1 mapping between object layers and broadphase layers
    broadphase_layer_interface := jolt.BroadPhaseLayerInterfaceTable_Create(2, 2)
	jolt.BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer(broadphase_layer_interface, u32(Layers.Non_Moving), u8(BroadPhaseLayers.Non_Moving))
	jolt.BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer(broadphase_layer_interface, u32(Layers.Moving),     u8(BroadPhaseLayers.Moving))

    object_vs_broadphase_layer_filter := jolt.ObjectVsBroadPhaseLayerFilterTable_Create(broadphase_layer_interface, 2, object_layer_pair_filter, 2)


    physics_system_config := jolt.PhysicsSystemSettings{
        maxBodies = 10240,
        numBodyMutexes = 0,
        maxBodyPairs = 65536,
        maxContactConstraints = 10240,
        _padding = 0,
        broadPhaseLayerInterface = broadphase_layer_interface,
        objectLayerPairFilter = object_layer_pair_filter,
        objectVsBroadPhaseLayerFilter = object_vs_broadphase_layer_filter,
    }
    physics_system := jolt.PhysicsSystem_Create(&physics_system_config)
	defer jolt.PhysicsSystem_Destroy(physics_system)

	// Optional: override the gravity.
	// jolt.PhysicsSystem_SetGravity(physics_system, &{0, -100, 0})

    body_interface := jolt.PhysicsSystem_GetBodyInterface(physics_system)


    floor_id: jolt.BodyID
	defer jolt.BodyInterface_RemoveAndDestroyBody(body_interface, floor_id)
    {
		// Next we can create a rigid body to serve as the floor, we make a large box
		// Create the settings for the collision volume (the shape). 
		// Note that for simple shapes (like boxes) you can also directly construct a BoxShape.
        box_half_extends := jolt.Vec3{ 2, 0.5, 2 }
        floor_shape := jolt.BoxShape_Create(&box_half_extends, jolt.DEFAULT_CONVEX_RADIUS)
        floor_position := jolt.Vec3{ 0, -0.5, 0 }
    
        floor_settings := jolt.BodyCreationSettings_Create3(
			cast(^jolt.Shape)floor_shape, 
			&floor_position, 
			nil, 
			.Static, 
			u32(Layers.Non_Moving)
		)

		// Create the actual rigid body
		floor_id = jolt.BodyInterface_CreateAndAddBody(body_interface, floor_settings, .DontActivate)
		jolt.BodyCreationSettings_Destroy(floor_settings)
	}


	spheres_id := make([dynamic]jolt.BodyID)
	defer {
		for id in spheres_id {
			jolt.BodyInterface_RemoveAndDestroyBody(body_interface, id)
		}
		delete(spheres_id)
	}

	{
		sphere_shape := jolt.SphereShape_Create(0.5)
		sphere_position := jolt.Vec3{ 0, 1, -1 }
		sphere_settings := jolt.BodyCreationSettings_Create3(
			cast(^jolt.Shape)sphere_shape,
			&sphere_position,
			nil, // Identity, 
			.Dynamic,
			u32(Layers.Moving),
		)

		// Optional: Limit the body movement to not move in the z axis.
		// This is important if you only want a 2D simulation.
		// jolt.BodyCreationSettings_SetAllowedDOFs(sphere_settings, .Plane2D)

		for i in -5..<5 {
			jolt.BodyCreationSettings_SetPosition(sphere_settings, &{ f32(i) * 0.1, 2 + f32(i + 5), f32(i) * 0.05 })

			id := jolt.BodyInterface_CreateAndAddBody(body_interface, sphere_settings, .Activate)
			append(&spheres_id, id)

			// Now you can interact with the dynamic body, in this case we're going to give it a velocity.
			// (note that if we had used CreateBody then we could have set the velocity straight on the body before adding it to the physics system)
			jolt.BodyInterface_SetLinearVelocity(body_interface, id, &{ 0, 10, 0 })
		}

		jolt.BodyCreationSettings_Destroy(sphere_settings)
	}


	// Optional step: Before starting the physics simulation you can optimize the broad phase. This improves collision detection performance (it's pointless here because we only have 2 bodies).
	// You should definitely not call this every frame or when e.g. streaming in a new level section as it is an expensive operation.
	// Instead insert all new objects in batches instead of 1 at a time to keep the broad phase efficient.
	jolt.PhysicsSystem_OptimizeBroadPhase(physics_system)

	// Raylib, for visuals
	raylib_init()
	defer raylib_deinit()

	// We simulate the physics world in discrete time steps. 60 Hz is a good rate to update the physics system.
	delta_time := f32(1) / f32(60)

	for !rl.WindowShouldClose() {
		// Physics update
		{
			// Step the world
			// If you take larger steps than 1 / 60th of a second you need to do multiple collision steps in order to keep the simulation stable. Do 1 collision step per 1 / 60th of a second (round up).
			jolt.PhysicsSystem_Update(system = physics_system, deltaTime = delta_time, collisionSteps = 1, jobSystem = job_system)
		}

		// Draw
		{
			rl.BeginDrawing()
			defer rl.EndDrawing()
			rl.ClearBackground(rl.WHITE)

			{
				rl.BeginMode3D(world_cam)
				defer rl.EndMode3D()

				rl.DrawGrid(10, 1)

				// Cube
				{
					floor_position: jolt.Vec3
					jolt.BodyInterface_GetPosition(body_interface, floor_id, &floor_position)
					floor_shape := jolt.BodyInterface_GetShape(body_interface, floor_id)
					half_extend: jolt.Vec3
					jolt.BoxShape_GetHalfExtent(cast(^jolt.BoxShape)floor_shape, &half_extend)
					rl.DrawCubeV({ floor_position.x, floor_position.y, floor_position.z },      rl.Vector3{ half_extend.x, half_extend.y, half_extend.z } * 2, {0, 200, 0, 255})
					rl.DrawCubeWiresV({ floor_position.x, floor_position.y, floor_position.z }, rl.Vector3{ half_extend.x, half_extend.y, half_extend.z } * 2.01, rl.BLACK)
				}
				
				// Axis
				rl.DrawCubeV({0.5, 0 + 0.05,   0  }, {1, 0.05, 0.05}, rl.RED)
				rl.DrawCubeV({0,   0.5 + 0.05, 0  }, {0.05, 1, 0.05}, rl.GREEN)
				rl.DrawCubeV({0,   0 + 0.05,   0.5}, {0.05, 0.05, 1}, rl.BLUE)

				// Spheres
				for id in spheres_id {
					sphere_position: jolt.Vec3
					jolt.BodyInterface_GetPosition(body_interface, id, &sphere_position)
					sphere_shape := jolt.BodyInterface_GetShape(body_interface, id)
					radius := jolt.SphereShape_GetRadius(cast(^jolt.SphereShape)sphere_shape)

					rl.DrawSphere({ sphere_position.x, sphere_position.y, sphere_position.z }, radius, rl.RED)
					rl.DrawSphereWires({ sphere_position.x, sphere_position.y, sphere_position.z }, radius * 1.02, 6, 12, rl.BLACK)
				}
			}
		}
	}

	fmt.printfln("Finished")
}


world_cam: rl.Camera3D

raylib_init :: proc() {
    rl.SetTraceLogLevel(rl.TraceLogLevel.WARNING)
    rl.InitWindow(1920, 1080, "Hello World")
    rl.SetWindowState({
        .WINDOW_ALWAYS_RUN,
    })
    rl.SetExitKey(.ESCAPE)
	rl.SetTargetFPS(60)

	world_cam = rl.Camera3D{
		position = rl.Vector3{ 3, 5, 4 } * 2,
        target = { 0, 0, 0 },
        up = { 0, 1, 0 },
        fovy = 45,
		projection = .PERSPECTIVE,
    }
}


raylib_deinit :: proc() {
	rl.CloseWindow()
}
