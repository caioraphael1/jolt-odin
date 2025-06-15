// Copyright (c) Amer Koleci and Contributors.
// Licensed under the MIT License (MIT). See LICENSE in the repository root for more information.
package jolt


// Compilation tests
// main :: proc() {
// 	Init()
// 	// BodyInterface_CreateBody()
// 	// ObjectLayerPairFilterMask_Create()

// 	fmt.printfln("hey... ;)")
// }


import "core:c"

import "core:fmt"

_ :: c

JOLT_C_H_ :: 1

// CAPI :: EXTERN EXPORT

DEFAULT_COLLISION_TOLERANCE :: 1.0-4 // float cDefaultCollisionTolerance = 1.0e-4f
DEFAULT_PENETRATION_TOLERANCE :: 1.0-4 // float cDefaultPenetrationTolerance = 1.0e-4f
DEFAULT_CONVEX_RADIUS :: 0.05 // float cDefaultConvexRadius = 0.05f
CAPSULE_PROJECTION_SLOP :: 0.02 // float cCapsuleProjectionSlop = 0.02f
MAX_PHYSICS_JOBS :: 2048 // int cMaxPhysicsJobs = 2048
MAX_PHYSICS_BARRIERS :: 8 // int cMaxPhysicsBarriers = 8
// INVALID_COLLISION_GROUP_ID :: ~0
// INVALID_COLLISION_SUBGROUP_ID :: ~0


// Não obtido via return
SharedMutex :: struct {}
GroupFilter :: struct {}
ShapeSettings :: struct {}

Shape :: struct {}
	// Obtido via getshape de um body ou bodyinterface

// Obtido via return
JobSystem :: struct {}
PhysicsSystem :: struct {}
BroadPhaseLayerInterface :: struct {}
ObjectLayerPairFilter :: struct {}
ObjectVsBroadPhaseLayerFilter :: struct {}
PhysicsMaterial :: struct {}
GroupFilterTable :: struct {}
ShapeFilter :: struct {}

// Parei aqui
ConvexShapeSettings :: struct {}
ConvexShape :: struct {}
BoxShapeSettings :: struct {}
BoxShape :: struct {}
SphereShapeSettings :: struct {}
SphereShape :: struct {}
PlaneShapeSettings :: struct {}
PlaneShape :: struct {}
TriangleShapeSettings :: struct {}
TriangleShape :: struct {}
CapsuleShapeSettings :: struct {}
CapsuleShape :: struct {}
CylinderShapeSettings :: struct {}
CylinderShape :: struct {}
TaperedCylinderShapeSettings :: struct {}
TaperedCylinderShape :: struct {}
ConvexHullShapeSettings :: struct {}
ConvexHullShape :: struct {}
MeshShapeSettings :: struct {}
MeshShape :: struct {}
HeightFieldShapeSettings :: struct {}
HeightFieldShape :: struct {}
TaperedCapsuleShapeSettings :: struct {}
TaperedCapsuleShape :: struct {}
CompoundShapeSettings :: struct {}
CompoundShape :: struct {}
StaticCompoundShapeSettings :: struct {}
StaticCompoundShape :: struct {}
MutableCompoundShapeSettings :: struct {}
MutableCompoundShape :: struct {}
DecoratedShape :: struct {}
RotatedTranslatedShapeSettings :: struct {}
RotatedTranslatedShape :: struct {}
ScaledShapeSettings :: struct {}
ScaledShape :: struct {}
OffsetCenterOfMassShapeSettings :: struct {}
OffsetCenterOfMassShape :: struct {}
EmptyShapeSettings :: struct {}
EmptyShape :: struct {}
BodyCreationSettings :: struct {}
SoftBodyCreationSettings :: struct {}
Constraint :: struct {}
TwoBodyConstraint :: struct {}
Body :: struct {}
FixedConstraint :: struct {}
DistanceConstraint :: struct {}
PointConstraint :: struct {}
HingeConstraint :: struct {}
ConeConstraint :: struct {}
SliderConstraint :: struct {}
SwingTwistConstraint :: struct {}
SixDOFConstraint :: struct {}
GearConstraint :: struct {}
BodyInterface :: struct {}
BodyLockInterface :: struct {}
BodyLockMultiRead :: struct {}
BodyLockMultiWrite :: struct {}
MotionProperties :: struct {}
BroadPhaseQuery :: struct {}
BroadPhaseLayerFilter :: struct {}
ObjectLayerFilter :: struct {}
NarrowPhaseQuery :: struct {}
BodyFilter :: struct {}
SimShapeFilter :: struct {}
ContactListener :: struct {}
BodyActivationListener :: struct {}
BodyDrawFilter :: struct {}
ContactManifold :: struct {}
ContactSettings :: struct {}
CharacterBase :: struct {}
Character :: struct {}
CharacterVirtual :: struct {}
CharacterContactListener :: struct {}
CharacterVsCharacterCollision :: struct {}
DebugRenderer :: struct {}
Skeleton :: struct {}
RagdollSettings :: struct {}
Ragdoll :: struct {}
VehicleConstraint :: struct {}
PhysicsStepListener :: struct {}
WheeledVehicleController :: struct {}
VehicleCollisionTester :: struct {}
Wheel :: struct {}
WheelWV :: struct {}
VehicleTransmissionSettings :: struct {}
VehicleCollisionTesterRay :: struct {}
VehicleCollisionTesterCastSphere :: struct {}
VehicleCollisionTesterCastCylinder :: struct {}
VehicleControllerSettings :: struct {}
WheeledVehicleControllerSettings :: struct {}


BodyID :: u32
SubShapeID :: u32
ObjectLayer :: u32
BroadPhaseLayer :: u8
CollisionGroupID :: u32
CollisionSubGroupID :: u32
CharacterID :: u32

/* Enums */
PhysicsUpdateError :: enum c.int {
	PhysicsUpdateError_None = 0,
	PhysicsUpdateError_ManifoldCacheFull = 1,
	PhysicsUpdateError_BodyPairCacheFull = 2,
	PhysicsUpdateError_ContactConstraintsFull = 4,
	_PhysicsUpdateError_Count,
	_PhysicsUpdateError_Force32 = 2147483647,
}

BodyType :: enum c.int {
	BodyType_Rigid = 0,
	BodyType_Soft = 1,
	_BodyType_Count,
	_BodyType_Force32 = 2147483647,
}

MotionType :: enum c.int {
	MotionType_Static = 0,
	MotionType_Kinematic = 1,
	MotionType_Dynamic = 2,
	_MotionType_Count,
	_MotionType_Force32 = 2147483647,
}

Activation :: enum c.int {
	Activation_Activate = 0,
	Activation_DontActivate = 1,
	_Activation_Count,
	_Activation_Force32 = 2147483647,
}

ValidateResult :: enum c.int {
	ValidateResult_AcceptAllContactsForThisBodyPair = 0,
	ValidateResult_AcceptContact = 1,
	ValidateResult_RejectContact = 2,
	ValidateResult_RejectAllContactsForThisBodyPair = 3,
	_ValidateResult_Count,
	_ValidateResult_Force32 = 2147483647,
}

ShapeType :: enum c.int {
	ShapeType_Convex = 0,
	ShapeType_Compound = 1,
	ShapeType_Decorated = 2,
	ShapeType_Mesh = 3,
	ShapeType_HeightField = 4,
	ShapeType_SoftBody = 5,
	ShapeType_User1 = 6,
	ShapeType_User2 = 7,
	ShapeType_User3 = 8,
	ShapeType_User4 = 9,
	_ShapeType_Count,
	_ShapeType_Force32 = 2147483647,
}

ShapeSubType :: enum c.int {
	ShapeSubType_Sphere = 0,
	ShapeSubType_Box = 1,
	ShapeSubType_Triangle = 2,
	ShapeSubType_Capsule = 3,
	ShapeSubType_TaperedCapsule = 4,
	ShapeSubType_Cylinder = 5,
	ShapeSubType_ConvexHull = 6,
	ShapeSubType_StaticCompound = 7,
	ShapeSubType_MutableCompound = 8,
	ShapeSubType_RotatedTranslated = 9,
	ShapeSubType_Scaled = 10,
	ShapeSubType_OffsetCenterOfMass = 11,
	ShapeSubType_Mesh = 12,
	ShapeSubType_HeightField = 13,
	ShapeSubType_SoftBody = 14,
	_ShapeSubType_Count,
	_ShapeSubType_Force32 = 2147483647,
}

ConstraintType :: enum c.int {
	ConstraintType_Constraint = 0,
	ConstraintType_TwoBodyConstraint = 1,
	_ConstraintType_Count,
	_ConstraintType_Force32 = 2147483647,
}

ConstraintSubType :: enum c.int {
	ConstraintSubType_Fixed = 0,
	ConstraintSubType_Point = 1,
	ConstraintSubType_Hinge = 2,
	ConstraintSubType_Slider = 3,
	ConstraintSubType_Distance = 4,
	ConstraintSubType_Cone = 5,
	ConstraintSubType_SwingTwist = 6,
	ConstraintSubType_SixDOF = 7,
	ConstraintSubType_Path = 8,
	ConstraintSubType_Vehicle = 9,
	ConstraintSubType_RackAndPinion = 10,
	ConstraintSubType_Gear = 11,
	ConstraintSubType_Pulley = 12,
	ConstraintSubType_User1 = 13,
	ConstraintSubType_User2 = 14,
	ConstraintSubType_User3 = 15,
	ConstraintSubType_User4 = 16,
	_ConstraintSubType_Count,
	_ConstraintSubType_Force32 = 2147483647,
}

ConstraintSpace :: enum c.int {
	ConstraintSpace_LocalToBodyCOM = 0,
	ConstraintSpace_WorldSpace = 1,
	_ConstraintSpace_Count,
	_ConstraintSpace_Force32 = 2147483647,
}

MotionQuality :: enum c.int {
	MotionQuality_Discrete = 0,
	MotionQuality_LinearCast = 1,
	_MotionQuality_Count,
	_MotionQuality_Force32 = 2147483647,
}

OverrideMassProperties :: enum c.int {
	OverrideMassProperties_CalculateMassAndInertia,
	OverrideMassProperties_CalculateInertia,
	OverrideMassProperties_MassAndInertiaProvided,
	_OverrideMassProperties_Count,
	_OverrideMassProperties_Force32 = 2147483647,
}

AllowedDOFs :: enum c.int {
	AllowedDOFs_All = 63,
	AllowedDOFs_TranslationX = 1,
	AllowedDOFs_TranslationY = 2,
	AllowedDOFs_TranslationZ = 4,
	AllowedDOFs_RotationX = 8,
	AllowedDOFs_RotationY = 16,
	AllowedDOFs_RotationZ = 32,
	AllowedDOFs_Plane2D = 35,
	_AllowedDOFs_Count,
	_AllowedDOFs_Force32 = 2147483647,
}

GroundState :: enum c.int {
	GroundState_OnGround = 0,
	GroundState_OnSteepGround = 1,
	GroundState_NotSupported = 2,
	GroundState_InAir = 3,
	_GroundState_Count,
	_GroundState_Force32 = 2147483647,
}

BackFaceMode :: enum c.int {
	BackFaceMode_IgnoreBackFaces,
	BackFaceMode_CollideWithBackFaces,
	_BackFaceMode_Count,
	_BackFaceMode_Force32 = 2147483647,
}

ActiveEdgeMode :: enum c.int {
	ActiveEdgeMode_CollideOnlyWithActive,
	ActiveEdgeMode_CollideWithAll,
	_ActiveEdgeMode_Count,
	_ActiveEdgeMode_Force32 = 2147483647,
}

CollectFacesMode :: enum c.int {
	CollectFacesMode_CollectFaces,
	CollectFacesMode_NoFaces,
	_CollectFacesMode_Count,
	_CollectFacesMode_Force32 = 2147483647,
}

MotorState :: enum c.int {
	MotorState_Off = 0,
	MotorState_Velocity = 1,
	MotorState_Position = 2,
	_MotorState_Count,
	_MotorState_Force32 = 2147483647,
}

CollisionCollectorType :: enum c.int {
	CollisionCollectorType_AllHit = 0,
	CollisionCollectorType_AllHitSorted = 1,
	CollisionCollectorType_ClosestHit = 2,
	CollisionCollectorType_AnyHit = 3,
	_CollisionCollectorType_Count,
	_CollisionCollectorType_Force32 = 2147483647,
}

SwingType :: enum c.int {
	SwingType_Cone,
	SwingType_Pyramid,
	_SwingType_Count,
	_SwingType_Force32 = 2147483647,
}

SixDOFConstraintAxis :: enum c.int {
	SixDOFConstraintAxis_TranslationX,
	SixDOFConstraintAxis_TranslationY,
	SixDOFConstraintAxis_TranslationZ,
	SixDOFConstraintAxis_RotationX,
	SixDOFConstraintAxis_RotationY,
	SixDOFConstraintAxis_RotationZ,
	_SixDOFConstraintAxis_Num,
	_SixDOFConstraintAxis_NumTranslation = 3,
	_SixDOFConstraintAxis_Force32 = 2147483647,
}

SpringMode :: enum c.int {
	SpringMode_FrequencyAndDamping = 0,
	SpringMode_StiffnessAndDamping = 1,
	_SpringMode_Count,
	_SpringMode_Force32 = 2147483647,
}

/// Defines how to color soft body constraints
SoftBodyConstraintColor :: enum c.int {
	SoftBodyConstraintColor_ConstraintType,

	/// Draw different types of constraints in different colors
	SoftBodyConstraintColor_ConstraintGroup,

	/// Draw constraints in the same group in the same color, non-parallel group will be red
	SoftBodyConstraintColor_ConstraintOrder,

	/// Draw constraints in the same group in the same color, non-parallel group will be red, and order within each group will be indicated with gradient
	_SoftBodyConstraintColor_Count,

	/// Draw constraints in the same group in the same color, non-parallel group will be red, and order within each group will be indicated with gradient
	_SoftBodyConstraintColor_Force32 = 2147483647,
}

BodyManager_ShapeColor :: enum c.int {
	BodyManager_ShapeColor_InstanceColor,   ///< Random color per instance
	BodyManager_ShapeColor_ShapeTypeColor,  ///< Convex = green, scaled = yellow, compound = orange, mesh = red
	BodyManager_ShapeColor_MotionTypeColor, ///< Static = grey, keyframed = green, dynamic = random color per instance
	BodyManager_ShapeColor_SleepColor,      ///< Static = grey, keyframed = green, dynamic = yellow, sleeping = red
	BodyManager_ShapeColor_IslandColor,     ///< Static = grey, active = random color per island, sleeping = light grey
	BodyManager_ShapeColor_MaterialColor,   ///< Color as defined by the PhysicsMaterial of the shape
	_BodyManager_ShapeColor_Count,
	_BodyManager_ShapeColor_Force32 = 2147483647,
}

DebugRenderer_CastShadow :: enum c.int {
	DebugRenderer_CastShadow_On = 0,  ///< This shape should cast a shadow
	DebugRenderer_CastShadow_Off = 1, ///< This shape should not cast a shadow
	_DebugRenderer_CastShadow_Count,
	_DebugRenderer_CastShadow_Force32 = 2147483647,
}

DebugRenderer_DrawMode :: enum c.int {
	DebugRenderer_DrawMode_Solid = 0,     ///< Draw as a solid shape
	DebugRenderer_DrawMode_Wireframe = 1, ///< Draw as wireframe
	_DebugRenderer_DrawMode_Count,
	_DebugRenderer_DrawMode_Force32 = 2147483647,
}

Mesh_Shape_BuildQuality :: enum c.int {
	Mesh_Shape_BuildQuality_FavorRuntimePerformance = 0,
	Mesh_Shape_BuildQuality_FavorBuildSpeed = 1,
	_Mesh_Shape_BuildQuality_Count,
	_Mesh_Shape_BuildQuality_Force32 = 2147483647,
}

TransmissionMode :: enum c.int {
	TransmissionMode_Auto = 0,
	TransmissionMode_Manual = 1,
	_TransmissionMode_Count,
	_TransmissionMode_Force32 = 2147483647,
}

Vec3 :: struct {
	x: f32,
	y: f32,
	z: f32,
}

Vec4 :: struct {
	x: f32,
	y: f32,
	z: f32,
	w: f32,
}

Quat :: struct {
	x: f32,
	y: f32,
	z: f32,
	w: f32,
}

Plane :: struct {
	normal:   Vec3,
	distance: f32,
}

Matrix4x4 :: struct {
	m11, m12, m13, m14: f32,
	m21, m22, m23, m24: f32,
	m31, m32, m33, m34: f32,
	m41, m42, m43, m44: f32,
}

RVec3 :: Vec3

RMatrix4x4 :: Matrix4x4

Color :: u32

AABox :: struct {
	min: Vec3,
	max: Vec3,
}

Triangle :: struct {
	v1:            Vec3,
	v2:            Vec3,
	v3:            Vec3,
	materialIndex: u32,
}

IndexedTriangleNoMaterial :: struct {
	i1: u32,
	i2: u32,
	i3: u32,
}

IndexedTriangle :: struct {
	i1:            u32,
	i2:            u32,
	i3:            u32,
	materialIndex: u32,
	userData:      u32,
}

MassProperties :: struct {
	mass:    f32,
	inertia: Matrix4x4,
}

CollideSettingsBase :: struct {
	activeEdgeMode:              ActiveEdgeMode,   /* = ActiveEdgeMode_CollideOnlyWithActive*/
	collectFacesMode:            CollectFacesMode, /* = CollectFacesMode_NoFaces*/
	collisionTolerance:          f32,              /* = DEFAULT_COLLISION_TOLERANCE*/
	penetrationTolerance:        f32,              /* = DEFAULT_PENETRATION_TOLERANCE*/
	activeEdgeMovementDirection: Vec3,             /* = Vec3::sZero()*/
}

/* CollideShapeSettings */
CollideShapeSettings :: struct {
	base:                  CollideSettingsBase, /* Inherics CollideSettingsBase */
	maxSeparationDistance: f32,                 /* = 0.0f*/
	backFaceMode:          BackFaceMode,        /* = BackFaceMode_IgnoreBackFaces*/
}

/* ShapeCastSettings */
ShapeCastSettings :: struct {
	base:                            CollideSettingsBase, /* Inherics CollideSettingsBase */
	backFaceModeTriangles:           BackFaceMode,        /* = BackFaceMode_IgnoreBackFaces*/
	backFaceModeConvex:              BackFaceMode,        /* = BackFaceMode_IgnoreBackFaces*/
	useShrunkenShapeAndConvexRadius: bool,                /* = false*/
	returnDeepestPoint:              bool,                /* = false*/
}

RayCastSettings :: struct {
	backFaceModeTriangles: BackFaceMode, /* = BackFaceMode_IgnoreBackFaces*/
	backFaceModeConvex:    BackFaceMode, /* = BackFaceMode_IgnoreBackFaces*/
	treatConvexAsSolid:    bool,         /* = true*/
}

SpringSettings :: struct {
	mode:                 SpringMode,
	frequencyOrStiffness: f32,
	damping:              f32,
}

MotorSettings :: struct {
	springSettings: SpringSettings,
	minForceLimit:  f32,
	maxForceLimit:  f32,
	minTorqueLimit: f32,
	maxTorqueLimit: f32,
}

SubShapeIDPair :: struct {
	Body1ID:     BodyID,
	subShapeID1: SubShapeID,
	Body2ID:     BodyID,
	subShapeID2: SubShapeID,
}

BroadPhaseCastResult :: struct {
	bodyID:   BodyID,
	fraction: f32,
}

RayCastResult :: struct {
	bodyID:      BodyID,
	fraction:    f32,
	subShapeID2: SubShapeID,
}

CollidePointResult :: struct {
	bodyID:      BodyID,
	subShapeID2: SubShapeID,
}

CollideShapeResult :: struct {
	contactPointOn1:  Vec3,
	contactPointOn2:  Vec3,
	penetrationAxis:  Vec3,
	penetrationDepth: f32,
	subShapeID1:      SubShapeID,
	subShapeID2:      SubShapeID,
	bodyID2:          BodyID,
	shape1FaceCount:  u32,
	shape1Faces:      ^Vec3,
	shape2FaceCount:  u32,
	shape2Faces:      ^Vec3,
}

ShapeCastResult :: struct {
	contactPointOn1:  Vec3,
	contactPointOn2:  Vec3,
	penetrationAxis:  Vec3,
	penetrationDepth: f32,
	subShapeID1:      SubShapeID,
	subShapeID2:      SubShapeID,
	bodyID2:          BodyID,
	fraction:         f32,
	isBackFaceHit:    bool,
}

DrawSettings :: struct {
	drawGetSupportFunction:        bool,                    ///< Draw the GetSupport() function, used for convex collision detection
	drawSupportDirection:          bool,                    ///< When drawing the support function, also draw which direction mapped to a specific support point
	drawGetSupportingFace:         bool,                    ///< Draw the faces that were found colliding during collision detection
	drawShape:                     bool,                    ///< Draw the shapes of all bodies
	drawShapeWireframe:            bool,                    ///< When mDrawShape is true and this is true, the shapes will be drawn in wireframe instead of solid.
	drawShapeColor:                BodyManager_ShapeColor,  ///< Coloring scheme to use for shapes
	drawBoundingBox:               bool,                    ///< Draw a bounding box per body
	drawCenterOfMassTransform:     bool,                    ///< Draw the center of mass for each body
	drawWorldTransform:            bool,                    ///< Draw the world transform (which can be different than the center of mass) for each body
	drawVelocity:                  bool,                    ///< Draw the velocity vector for each body
	drawMassAndInertia:            bool,                    ///< Draw the mass and inertia (as the box equivalent) for each body
	drawSleepStats:                bool,                    ///< Draw stats regarding the sleeping algorithm of each body
	drawSoftBodyVertices:          bool,                    ///< Draw the vertices of soft bodies
	drawSoftBodyVertexVelocities:  bool,                    ///< Draw the velocities of the vertices of soft bodies
	drawSoftBodyEdgeConstraints:   bool,                    ///< Draw the edge constraints of soft bodies
	drawSoftBodyBendConstraints:   bool,                    ///< Draw the bend constraints of soft bodies
	drawSoftBodyVolumeConstraints: bool,                    ///< Draw the volume constraints of soft bodies
	drawSoftBodySkinConstraints:   bool,                    ///< Draw the skin constraints of soft bodies
	drawSoftBodyLRAConstraints:    bool,                    ///< Draw the LRA constraints of soft bodies
	drawSoftBodyPredictedBounds:   bool,                    ///< Draw the predicted bounds of soft bodies
	drawSoftBodyConstraintColor:   SoftBodyConstraintColor, ///< Coloring scheme to use for soft body constraints
}

SupportingFace :: struct {
	count:    u32,
	vertices: [32]Vec3,
}

CollisionGroup :: struct {
	groupFilter: ^GroupFilter,
	groupID:     CollisionGroupID,
	subGroupID:  CollisionSubGroupID,
}

CastRayResultCallback :: proc "c" (rawptr, ^RayCastResult)

RayCastBodyResultCallback :: proc "c" (rawptr, ^BroadPhaseCastResult)

CollideShapeBodyResultCallback :: proc "c" (rawptr, BodyID)

CollidePointResultCallback :: proc "c" (rawptr, ^CollidePointResult)

CollideShapeResultCallback :: proc "c" (rawptr, ^CollideShapeResult)

CastShapeResultCallback :: proc "c" (rawptr, ^ShapeCastResult)

CastRayCollectorCallback :: proc "c" (rawptr, ^RayCastResult) -> f32

RayCastBodyCollectorCallback :: proc "c" (rawptr, ^BroadPhaseCastResult) -> f32

CollideShapeBodyCollectorCallback :: proc "c" (rawptr, BodyID) -> f32

CollidePointCollectorCallback :: proc "c" (rawptr, ^CollidePointResult) -> f32

CollideShapeCollectorCallback :: proc "c" (rawptr, ^CollideShapeResult) -> f32

CastShapeCollectorCallback :: proc "c" (rawptr, ^ShapeCastResult) -> f32

CollisionEstimationResultImpulse :: struct {
	contactImpulse:   f32,
	frictionImpulse1: f32,
	frictionImpulse2: f32,
}

CollisionEstimationResult :: struct {
	linearVelocity1:  Vec3,
	angularVelocity1: Vec3,
	linearVelocity2:  Vec3,
	angularVelocity2: Vec3,
	tangent1:         Vec3,
	tangent2:         Vec3,
	impulseCount:     u32,
	impulses:         ^CollisionEstimationResultImpulse,
}

ConstraintSettings :: struct {
	enabled:                  bool,
	constraintPriority:       u32,
	numVelocityStepsOverride: u32,
	numPositionStepsOverride: u32,
	drawConstraintSize:       f32,
	userData:                 u64,
}

FixedConstraintSettings :: struct {
	base:            ConstraintSettings, /* Inherics ConstraintSettings */
	space:           ConstraintSpace,
	autoDetectPoint: bool,
	point1:          RVec3,
	axisX1:          Vec3,
	axisY1:          Vec3,
	point2:          RVec3,
	axisX2:          Vec3,
	axisY2:          Vec3,
}

DistanceConstraintSettings :: struct {
	base:                 ConstraintSettings, /* Inherics ConstraintSettings */
	space:                ConstraintSpace,
	point1:               RVec3,
	point2:               RVec3,
	minDistance:          f32,
	maxDistance:          f32,
	limitsSpringSettings: SpringSettings,
}

PointConstraintSettings :: struct {
	base:   ConstraintSettings, /* Inherics ConstraintSettings */
	space:  ConstraintSpace,
	point1: RVec3,
	point2: RVec3,
}

HingeConstraintSettings :: struct {
	base:                 ConstraintSettings, /* Inherics ConstraintSettings */
	space:                ConstraintSpace,
	point1:               RVec3,
	hingeAxis1:           Vec3,
	normalAxis1:          Vec3,
	point2:               RVec3,
	hingeAxis2:           Vec3,
	normalAxis2:          Vec3,
	limitsMin:            f32,
	limitsMax:            f32,
	limitsSpringSettings: SpringSettings,
	maxFrictionTorque:    f32,
	motorSettings:        MotorSettings,
}

SliderConstraintSettings :: struct {
	base:                 ConstraintSettings, /* Inherics ConstraintSettings */
	space:                ConstraintSpace,
	autoDetectPoint:      bool,
	point1:               RVec3,
	sliderAxis1:          Vec3,
	normalAxis1:          Vec3,
	point2:               RVec3,
	sliderAxis2:          Vec3,
	normalAxis2:          Vec3,
	limitsMin:            f32,
	limitsMax:            f32,
	limitsSpringSettings: SpringSettings,
	maxFrictionForce:     f32,
	motorSettings:        MotorSettings,
}

ConeConstraintSettings :: struct {
	base:          ConstraintSettings, /* Inherics ConstraintSettings */
	space:         ConstraintSpace,
	point1:        RVec3,
	twistAxis1:    Vec3,
	point2:        RVec3,
	twistAxis2:    Vec3,
	halfConeAngle: f32,
}

SwingTwistConstraintSettings :: struct {
	base:                ConstraintSettings, /* Inherics ConstraintSettings */
	space:               ConstraintSpace,
	position1:           RVec3,
	twistAxis1:          Vec3,
	planeAxis1:          Vec3,
	position2:           RVec3,
	twistAxis2:          Vec3,
	planeAxis2:          Vec3,
	swingType:           SwingType,
	normalHalfConeAngle: f32,
	planeHalfConeAngle:  f32,
	twistMinAngle:       f32,
	twistMaxAngle:       f32,
	maxFrictionTorque:   f32,
	swingMotorSettings:  MotorSettings,
	twistMotorSettings:  MotorSettings,
}

SixDOFConstraintSettings :: struct {
	base:                 ConstraintSettings, /* Inherics ConstraintSettings */
	space:                ConstraintSpace,
	position1:            RVec3,
	axisX1:               Vec3,
	axisY1:               Vec3,
	position2:            RVec3,
	axisX2:               Vec3,
	axisY2:               Vec3,
	maxFriction:          [6]f32,
	swingType:            SwingType,
	limitMin:             [6]f32,
	limitMax:             [6]f32,
	limitsSpringSettings: [3]SpringSettings,
	motorSettings:        [6]MotorSettings,
}

GearConstraintSettings :: struct {
	base:       ConstraintSettings, /* Inherics ConstraintSettings */
	space:      ConstraintSpace,
	hingeAxis1: Vec3,
	hingeAxis2: Vec3,
	ratio:      f32,
}

BodyLockRead :: struct {
	lockInterface: ^BodyLockInterface,
	mutex:         ^SharedMutex,
	body:          ^Body,
}

BodyLockWrite :: struct {
	lockInterface: ^BodyLockInterface,
	mutex:         ^SharedMutex,
	body:          ^Body,
}

ExtendedUpdateSettings :: struct {
	stickToFloorStepDown:             Vec3,
	walkStairsStepUp:                 Vec3,
	walkStairsMinStepForward:         f32,
	walkStairsStepForwardTest:        f32,
	walkStairsCosAngleForwardContact: f32,
	walkStairsStepDownExtra:          Vec3,
}

CharacterBaseSettings :: struct {
	up:                          Vec3,
	supportingVolume:            Plane,
	maxSlopeAngle:               f32,
	enhancedInternalEdgeRemoval: bool,
	shape:                       ^Shape,
}

/* Character */
CharacterSettings :: struct {
	base:          CharacterBaseSettings, /* Inherics CharacterBaseSettings */
	layer:         ObjectLayer,
	mass:          f32,
	friction:      f32,
	gravityFactor: f32,
	allowedDOFs:   AllowedDOFs,
}

/* CharacterVirtual */
CharacterVirtualSettings :: struct {
	base:                      CharacterBaseSettings, /* Inherics CharacterBaseSettings */
	ID:                        CharacterID,
	mass:                      f32,
	maxStrength:               f32,
	shapeOffset:               Vec3,
	backFaceMode:              BackFaceMode,
	predictiveContactDistance: f32,
	maxCollisionIterations:    u32,
	maxConstraintIterations:   u32,
	minTimeRemaining:          f32,
	collisionTolerance:        f32,
	characterPadding:          f32,
	maxNumHits:                u32,
	hitReductionCosMaxAngle:   f32,
	penetrationRecoverySpeed:  f32,
	innerBodyShape:            ^Shape,
	innerBodyIDOverride:       BodyID,
	innerBodyLayer:            ObjectLayer,
}

CharacterContactSettings :: struct {
	canPushCharacter:   bool,
	canReceiveImpulses: bool,
}

CharacterVirtualContact :: struct {
	hash:             u64,
	bodyB:            BodyID,
	characterIDB:     CharacterID,
	subShapeIDB:      SubShapeID,
	position:         RVec3,
	linearVelocity:   Vec3,
	contactNormal:    Vec3,
	surfaceNormal:    Vec3,
	distance:         f32,
	fraction:         f32,
	motionTypeB:      MotionType,
	isSensorB:        bool,
	characterB:       ^CharacterVirtual,
	userData:         u64,
	material:         ^PhysicsMaterial,
	hadCollision:     bool,
	wasDiscarded:     bool,
	canPushCharacter: bool,
}

TraceFunc :: proc "c" (cstring)

AssertFailureFunc :: proc "c" (cstring, cstring, cstring, u32) -> bool

JobFunction :: proc "c" (rawptr)

QueueJobCallback :: proc "c" (rawptr, JobFunction, rawptr)

QueueJobsCallback :: proc "c" (rawptr, JobFunction, ^rawptr, u32)

JobSystemThreadPoolConfig :: struct {
	maxJobs:     u32,
	maxBarriers: u32,
	numThreads:  i32,
}

JobSystemConfig :: struct {
	_context:       rawptr,
	queueJob:       QueueJobCallback,
	queueJobs:      QueueJobsCallback,
	maxConcurrency: u32,
	maxBarriers:    u32,
}

/* PhysicsSystem */
PhysicsSystemSettings :: struct {
	maxBodies:                     u32, /* 10240 */
	numBodyMutexes:                u32, /* 0 */
	maxBodyPairs:                  u32, /* 65536 */
	maxContactConstraints:         u32, /* 10240 */
	_padding:                      u32,
	broadPhaseLayerInterface:      ^BroadPhaseLayerInterface,
	objectLayerPairFilter:         ^ObjectLayerPairFilter,
	objectVsBroadPhaseLayerFilter: ^ObjectVsBroadPhaseLayerFilter,
}

PhysicsSettings :: struct {
	maxInFlightBodyPairs:                 c.int,
	stepListenersBatchSize:               c.int,
	stepListenerBatchesPerJob:            c.int,
	baumgarte:                            f32,
	speculativeContactDistance:           f32,
	penetrationSlop:                      f32,
	linearCastThreshold:                  f32,
	linearCastMaxPenetration:             f32,
	manifoldTolerance:                    f32,
	maxPenetrationDistance:               f32,
	bodyPairCacheMaxDeltaPositionSq:      f32,
	bodyPairCacheCosMaxDeltaRotationDiv2: f32,
	contactNormalCosMaxDeltaRotation:     f32,
	contactPointPreserveLambdaMaxDistSq:  f32,
	numVelocitySteps:                     u32,
	numPositionSteps:                     u32,
	minVelocityForRestitution:            f32,
	timeBeforeSleep:                      f32,
	pointVelocitySleepThreshold:          f32,
	deterministicSimulation:              bool,
	constraintWarmStart:                  bool,
	useBodyPairContactCache:              bool,
	useManifoldReduction:                 bool,
	useLargeIslandSplitter:               bool,
	allowSleeping:                        bool,
	checkActiveEdges:                     bool,
}

/* PhysicsStepListener */
PhysicsStepListenerContext :: struct {
	deltaTime:     f32,
	isFirstStep:   bool,
	isLastStep:    bool,
	physicsSystem: ^PhysicsSystem,
}

PhysicsStepListener_Procs :: struct {
	OnStep: proc "c" (rawptr, ^PhysicsStepListenerContext),
}

/* BroadPhaseLayerFilter_Procs */
BroadPhaseLayerFilter_Procs :: struct {
	ShouldCollide: proc "c" (rawptr, BroadPhaseLayer) -> bool,
}

/* ObjectLayerFilter */
ObjectLayerFilter_Procs :: struct {
	ShouldCollide: proc "c" (rawptr, ObjectLayer) -> bool,
}

/* BodyFilter */
BodyFilter_Procs :: struct {
	ShouldCollide:       proc "c" (rawptr, BodyID) -> bool,
	ShouldCollideLocked: proc "c" (rawptr, ^Body) -> bool,
}

/* ShapeFilter */
ShapeFilter_Procs :: struct {
	ShouldCollide:  proc "c" (rawptr, ^Shape, ^SubShapeID) -> bool,
	ShouldCollide2: proc "c" (rawptr, ^Shape, ^SubShapeID, ^Shape, ^SubShapeID) -> bool,
}

/* SimShapeFilter */
SimShapeFilter_Procs :: struct {
	ShouldCollide: proc "c" (rawptr, ^Body, ^Shape, ^SubShapeID, ^Body, ^Shape, ^SubShapeID) -> bool,
}

/* Contact listener */
ContactListener_Procs :: struct {
	OnContactValidate:  proc "c" (rawptr, ^Body, ^Body, ^RVec3, ^CollideShapeResult) -> ValidateResult,
	OnContactAdded:     proc "c" (rawptr, ^Body, ^Body, ^ContactManifold, ^ContactSettings),
	OnContactPersisted: proc "c" (rawptr, ^Body, ^Body, ^ContactManifold, ^ContactSettings),
	OnContactRemoved:   proc "c" (rawptr, ^SubShapeIDPair),
}

/* BodyActivationListener */
BodyActivationListener_Procs :: struct {
	OnBodyActivated:   proc "c" (rawptr, BodyID, i64),
	OnBodyDeactivated: proc "c" (rawptr, BodyID, i64),
}

/* BodyDrawFilter */
BodyDrawFilter_Procs :: struct {
	ShouldDraw: proc "c" (rawptr, ^Body) -> bool,
}

/* CharacterContactListener */
CharacterContactListener_Procs :: struct {
	OnAdjustBodyVelocity:        proc "c" (rawptr, ^CharacterVirtual, ^Body, ^Vec3, ^Vec3),
	OnContactValidate:           proc "c" (rawptr, ^CharacterVirtual, BodyID, SubShapeID) -> bool,
	OnCharacterContactValidate:  proc "c" (rawptr, ^CharacterVirtual, ^CharacterVirtual, SubShapeID) -> bool,
	OnContactAdded:              proc "c" (rawptr, ^CharacterVirtual, BodyID, SubShapeID, ^RVec3, ^Vec3, ^CharacterContactSettings),
	OnContactPersisted:          proc "c" (rawptr, ^CharacterVirtual, BodyID, SubShapeID, ^RVec3, ^Vec3, ^CharacterContactSettings),
	OnContactRemoved:            proc "c" (rawptr, ^CharacterVirtual, BodyID, SubShapeID),
	OnCharacterContactAdded:     proc "c" (rawptr, ^CharacterVirtual, ^CharacterVirtual, SubShapeID, ^RVec3, ^Vec3, ^CharacterContactSettings),
	OnCharacterContactPersisted: proc "c" (rawptr, ^CharacterVirtual, ^CharacterVirtual, SubShapeID, ^RVec3, ^Vec3, ^CharacterContactSettings),
	OnCharacterContactRemoved:   proc "c" (rawptr, ^CharacterVirtual, CharacterID, SubShapeID),
	OnContactSolve:              proc "c" (rawptr, ^CharacterVirtual, BodyID, SubShapeID, ^RVec3, ^Vec3, ^Vec3, ^PhysicsMaterial, ^Vec3, ^Vec3),
	OnCharacterContactSolve:     proc "c" (rawptr, ^CharacterVirtual, ^CharacterVirtual, SubShapeID, ^RVec3, ^Vec3, ^Vec3, ^PhysicsMaterial, ^Vec3, ^Vec3),
}

/* CharacterVsCharacterCollision */
CharacterVsCharacterCollision_Procs :: struct {
	CollideCharacter: proc "c" (rawptr, ^CharacterVirtual, ^RMatrix4x4, ^CollideShapeSettings, ^RVec3),
	CastCharacter:    proc "c" (rawptr, ^CharacterVirtual, ^RMatrix4x4, ^Vec3, ^ShapeCastSettings, ^RVec3),
}

/* DebugRenderer */
DebugRenderer_Procs :: struct {
	DrawLine:     proc "c" (rawptr, ^RVec3, ^RVec3, Color),
	DrawTriangle: proc "c" (rawptr, ^RVec3, ^RVec3, ^RVec3, Color, DebugRenderer_CastShadow),
	DrawText3D:   proc "c" (rawptr, ^RVec3, cstring, Color, f32),
}

/* Skeleton */
SkeletonJoint :: struct {
	name:             cstring,
	parentName:       cstring,
	parentJointIndex: c.int,
}

WheelSettings :: struct {
	position:                   Vec3,
	suspensionForcePoint:       Vec3,
	suspensionDirection:        Vec3,
	steeringAxis:               Vec3,
	wheelUp:                    Vec3,
	wheelForward:               Vec3,
	suspensionMinLength:        f32,
	suspensionMaxLength:        f32,
	suspensionPreloadLength:    f32,
	suspensionSpring:           SpringSettings,
	radius:                     f32,
	width:                      f32,
	enableSuspensionForcePoint: bool,
}

WheelSettingsWV :: struct {
	base:               WheelSettings, /* Inherics CharacterBaseSettings */
	inertia:            f32,
	angularDamping:     f32,
	maxSteerAngle:      f32,

	//LinearCurve				longitudinalFriction;
	//LinearCurve				lateralFriction;
	maxBrakeTorque: f32,
	maxHandBrakeTorque: f32,
}

VehicleConstraintSettings :: struct {
	base:              ConstraintSettings, /* Inherics ConstraintSettings */
	up:                Vec3,
	forward:           Vec3,
	maxPitchRollAngle: f32,
	wheelsCount:       u32,
	wheels:            ^WheelSettingsWV,   /* TODO: Should be WheelSettings */

	//uint32_t					antiRollBarsCount;	// NOTE: BGE: just using default values for now.
	//VehicleAntiRollBars		antiRollBars;		// NOTE: BGE: just using default values for now.
	controller: ^VehicleControllerSettings,
}

VehicleEngineSettings :: struct {
	maxTorque:      f32,
	minRPM:         f32,
	maxRPM:         f32,

	//LinearCurve			normalizedTorque;
	inertia: f32,
	angularDamping: f32,
}


when ODIN_OS == .Windows {
	// @(extra_linker_flags="-ltcg")
	// @(extra_linker_flags="/NODEFAULTLIB:LIBCMTD")
    foreign import lib {
		"windows/Jolt.lib",
		// "windows/joltc.dll",
		"windows/joltc.lib",
    }
} else {
	#panic("No lib.")
    // @(extra_linker_flags="-lstdc++")
    // foreign import lib {
        // "build/jolt.lib",
    // }
}

@(default_calling_convention="c", link_prefix="JPH_")
foreign lib {
	JobSystemThreadPool_Create :: proc(config: ^JobSystemThreadPoolConfig) -> ^JobSystem ---
	JobSystemCallback_Create   :: proc(config: ^JobSystemConfig) -> ^JobSystem ---
	JobSystem_Destroy          :: proc(jobSystem: ^JobSystem) ---
	Init                       :: proc() -> bool ---
	Shutdown                   :: proc() ---
	SetTraceHandler            :: proc(handler: TraceFunc) ---
	SetAssertFailureHandler    :: proc(handler: AssertFailureFunc) ---

	/* Structs free members */
	CollideShapeResult_FreeMembers        :: proc(result: ^CollideShapeResult) ---
	CollisionEstimationResult_FreeMembers :: proc(result: ^CollisionEstimationResult) ---

	/* BroadPhaseLayerInterface */
	BroadPhaseLayerInterfaceMask_Create                      :: proc(numBroadPhaseLayers: u32) -> ^BroadPhaseLayerInterface ---
	BroadPhaseLayerInterfaceMask_ConfigureLayer              :: proc(bpInterface: ^BroadPhaseLayerInterface, broadPhaseLayer: BroadPhaseLayer, groupsToInclude: u32, groupsToExclude: u32) ---
	BroadPhaseLayerInterfaceTable_Create                     :: proc(numObjectLayers: u32, numBroadPhaseLayers: u32) -> ^BroadPhaseLayerInterface ---
	BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer :: proc(bpInterface: ^BroadPhaseLayerInterface, objectLayer: ObjectLayer, broadPhaseLayer: BroadPhaseLayer) ---

	/* ObjectLayerPairFilter */
	ObjectLayerPairFilterMask_Create            :: proc() -> ^ObjectLayerPairFilter ---
	ObjectLayerPairFilterMask_GetObjectLayer    :: proc(group: u32, mask: u32) -> ObjectLayer ---
	ObjectLayerPairFilterMask_GetGroup          :: proc(layer: ObjectLayer) -> u32 ---
	ObjectLayerPairFilterMask_GetMask           :: proc(layer: ObjectLayer) -> u32 ---
	ObjectLayerPairFilterTable_Create           :: proc(numObjectLayers: u32) -> ^ObjectLayerPairFilter ---
	ObjectLayerPairFilterTable_DisableCollision :: proc(objectFilter: ^ObjectLayerPairFilter, layer1: ObjectLayer, layer2: ObjectLayer) ---
	ObjectLayerPairFilterTable_EnableCollision  :: proc(objectFilter: ^ObjectLayerPairFilter, layer1: ObjectLayer, layer2: ObjectLayer) ---
	ObjectLayerPairFilterTable_ShouldCollide    :: proc(objectFilter: ^ObjectLayerPairFilter, layer1: ObjectLayer, layer2: ObjectLayer) -> bool ---

	/* ObjectVsBroadPhaseLayerFilter */
	ObjectVsBroadPhaseLayerFilterMask_Create   :: proc(broadPhaseLayerInterface: ^BroadPhaseLayerInterface) -> ^ObjectVsBroadPhaseLayerFilter ---
	ObjectVsBroadPhaseLayerFilterTable_Create  :: proc(broadPhaseLayerInterface: ^BroadPhaseLayerInterface, numBroadPhaseLayers: u32, objectLayerPairFilter: ^ObjectLayerPairFilter, numObjectLayers: u32) -> ^ObjectVsBroadPhaseLayerFilter ---
	DrawSettings_InitDefault                   :: proc(settings: ^DrawSettings) ---
	PhysicsSystem_Create                       :: proc(settings: ^PhysicsSystemSettings) -> ^PhysicsSystem ---
	PhysicsSystem_Destroy                      :: proc(system: ^PhysicsSystem) ---
	PhysicsSystem_SetPhysicsSettings           :: proc(system: ^PhysicsSystem, settings: ^PhysicsSettings) ---
	PhysicsSystem_GetPhysicsSettings           :: proc(system: ^PhysicsSystem, result: ^PhysicsSettings) ---
	PhysicsSystem_OptimizeBroadPhase           :: proc(system: ^PhysicsSystem) ---
	PhysicsSystem_Update                       :: proc(system: ^PhysicsSystem, deltaTime: f32, collisionSteps: c.int, jobSystem: ^JobSystem) -> PhysicsUpdateError ---
	PhysicsSystem_GetBodyInterface             :: proc(system: ^PhysicsSystem) -> ^BodyInterface ---
	PhysicsSystem_GetBodyInterfaceNoLock       :: proc(system: ^PhysicsSystem) -> ^BodyInterface ---
	PhysicsSystem_GetBodyLockInterface         :: proc(system: ^PhysicsSystem) -> ^BodyLockInterface ---
	PhysicsSystem_GetBodyLockInterfaceNoLock   :: proc(system: ^PhysicsSystem) -> ^BodyLockInterface ---
	PhysicsSystem_GetBroadPhaseQuery           :: proc(system: ^PhysicsSystem) -> ^BroadPhaseQuery ---
	PhysicsSystem_GetNarrowPhaseQuery          :: proc(system: ^PhysicsSystem) -> ^NarrowPhaseQuery ---
	PhysicsSystem_GetNarrowPhaseQueryNoLock    :: proc(system: ^PhysicsSystem) -> ^NarrowPhaseQuery ---
	PhysicsSystem_SetContactListener           :: proc(system: ^PhysicsSystem, listener: ^ContactListener) ---
	PhysicsSystem_SetBodyActivationListener    :: proc(system: ^PhysicsSystem, listener: ^BodyActivationListener) ---
	PhysicsSystem_SetSimShapeFilter            :: proc(system: ^PhysicsSystem, filter: ^SimShapeFilter) ---
	PhysicsSystem_WereBodiesInContact          :: proc(system: ^PhysicsSystem, body1: BodyID, body2: BodyID) -> bool ---
	PhysicsSystem_GetNumBodies                 :: proc(system: ^PhysicsSystem) -> u32 ---
	PhysicsSystem_GetNumActiveBodies           :: proc(system: ^PhysicsSystem, type: BodyType) -> u32 ---
	PhysicsSystem_GetMaxBodies                 :: proc(system: ^PhysicsSystem) -> u32 ---
	PhysicsSystem_GetNumConstraints            :: proc(system: ^PhysicsSystem) -> u32 ---
	PhysicsSystem_SetGravity                   :: proc(system: ^PhysicsSystem, value: ^Vec3) ---
	PhysicsSystem_GetGravity                   :: proc(system: ^PhysicsSystem, result: ^Vec3) ---
	PhysicsSystem_AddConstraint                :: proc(system: ^PhysicsSystem, constraint: ^Constraint) ---
	PhysicsSystem_RemoveConstraint             :: proc(system: ^PhysicsSystem, constraint: ^Constraint) ---
	PhysicsSystem_AddConstraints               :: proc(system: ^PhysicsSystem, constraints: ^^Constraint, count: u32) ---
	PhysicsSystem_RemoveConstraints            :: proc(system: ^PhysicsSystem, constraints: ^^Constraint, count: u32) ---
	PhysicsSystem_AddStepListener              :: proc(system: ^PhysicsSystem, listener: ^PhysicsStepListener) ---
	PhysicsSystem_RemoveStepListener           :: proc(system: ^PhysicsSystem, listener: ^PhysicsStepListener) ---
	PhysicsSystem_GetBodies                    :: proc(system: ^PhysicsSystem, ids: ^BodyID, count: u32) ---
	PhysicsSystem_GetConstraints               :: proc(system: ^PhysicsSystem, constraints: ^^Constraint, count: u32) ---
	PhysicsSystem_DrawBodies                   :: proc(system: ^PhysicsSystem, settings: ^DrawSettings, renderer: ^DebugRenderer, bodyFilter: ^BodyDrawFilter) ---
	PhysicsSystem_DrawConstraints              :: proc(system: ^PhysicsSystem, renderer: ^DebugRenderer) ---
	PhysicsSystem_DrawConstraintLimits         :: proc(system: ^PhysicsSystem, renderer: ^DebugRenderer) ---
	PhysicsSystem_DrawConstraintReferenceFrame :: proc(system: ^PhysicsSystem, renderer: ^DebugRenderer) ---
	PhysicsStepListener_SetProcs               :: proc(procs: ^PhysicsStepListener_Procs) ---
	PhysicsStepListener_Create                 :: proc(userData: rawptr) -> ^PhysicsStepListener ---
	PhysicsStepListener_Destroy                :: proc(listener: ^PhysicsStepListener) ---

	/* Math */
	Quaternion_FromTo                     :: proc(from: ^Vec3, to: ^Vec3, quat: ^Quat) ---
	Quat_GetAxisAngle                     :: proc(quat: ^Quat, outAxis: ^Vec3, outAngle: ^f32) ---
	Quat_GetEulerAngles                   :: proc(quat: ^Quat, result: ^Vec3) ---
	Quat_RotateAxisX                      :: proc(quat: ^Quat, result: ^Vec3) ---
	Quat_RotateAxisY                      :: proc(quat: ^Quat, result: ^Vec3) ---
	Quat_RotateAxisZ                      :: proc(quat: ^Quat, result: ^Vec3) ---
	Quat_Inversed                         :: proc(quat: ^Quat, result: ^Quat) ---
	Quat_GetPerpendicular                 :: proc(quat: ^Quat, result: ^Quat) ---
	Quat_GetRotationAngle                 :: proc(quat: ^Quat, axis: ^Vec3) -> f32 ---
	Quat_FromEulerAngles                  :: proc(angles: ^Vec3, result: ^Quat) ---
	Quat_Add                              :: proc(q1: ^Quat, q2: ^Quat, result: ^Quat) ---
	Quat_Subtract                         :: proc(q1: ^Quat, q2: ^Quat, result: ^Quat) ---
	Quat_Multiply                         :: proc(q1: ^Quat, q2: ^Quat, result: ^Quat) ---
	Quat_MultiplyScalar                   :: proc(q: ^Quat, scalar: f32, result: ^Quat) ---
	Quat_Divide                           :: proc(q1: ^Quat, q2: ^Quat, result: ^Quat) ---
	Quat_Dot                              :: proc(q1: ^Quat, q2: ^Quat, result: ^f32) ---
	Quat_Conjugated                       :: proc(quat: ^Quat, result: ^Quat) ---
	Quat_GetTwist                         :: proc(quat: ^Quat, axis: ^Vec3, result: ^Quat) ---
	Quat_GetSwingTwist                    :: proc(quat: ^Quat, outSwing: ^Quat, outTwist: ^Quat) ---
	Quat_LERP                             :: proc(from: ^Quat, to: ^Quat, fraction: f32, result: ^Quat) ---
	Quat_SLERP                            :: proc(from: ^Quat, to: ^Quat, fraction: f32, result: ^Quat) ---
	Quat_Rotate                           :: proc(quat: ^Quat, vec: ^Vec3, result: ^Vec3) ---
	Quat_InverseRotate                    :: proc(quat: ^Quat, vec: ^Vec3, result: ^Vec3) ---
	Vec3_IsClose                          :: proc(v1: ^Vec3, v2: ^Vec3, maxDistSq: f32) -> bool ---
	Vec3_IsNearZero                       :: proc(v: ^Vec3, maxDistSq: f32) -> bool ---
	Vec3_IsNormalized                     :: proc(v: ^Vec3, tolerance: f32) -> bool ---
	Vec3_IsNaN                            :: proc(v: ^Vec3) -> bool ---
	Vec3_Negate                           :: proc(v: ^Vec3, result: ^Vec3) ---
	Vec3_Normalized                       :: proc(v: ^Vec3, result: ^Vec3) ---
	Vec3_Cross                            :: proc(v1: ^Vec3, v2: ^Vec3, result: ^Vec3) ---
	Vec3_Abs                              :: proc(v: ^Vec3, result: ^Vec3) ---
	Vec3_Length                           :: proc(v: ^Vec3) -> f32 ---
	Vec3_LengthSquared                    :: proc(v: ^Vec3) -> f32 ---
	Vec3_DotProduct                       :: proc(v1: ^Vec3, v2: ^Vec3, result: ^f32) ---
	Vec3_Normalize                        :: proc(v: ^Vec3, result: ^Vec3) ---
	Vec3_Add                              :: proc(v1: ^Vec3, v2: ^Vec3, result: ^Vec3) ---
	Vec3_Subtract                         :: proc(v1: ^Vec3, v2: ^Vec3, result: ^Vec3) ---
	Vec3_Multiply                         :: proc(v1: ^Vec3, v2: ^Vec3, result: ^Vec3) ---
	Vec3_MultiplyScalar                   :: proc(v: ^Vec3, scalar: f32, result: ^Vec3) ---
	Vec3_Divide                           :: proc(v1: ^Vec3, v2: ^Vec3, result: ^Vec3) ---
	Vec3_DivideScalar                     :: proc(v: ^Vec3, scalar: f32, result: ^Vec3) ---
	Matrix4x4_Add                         :: proc(m1: ^Matrix4x4, m2: ^Matrix4x4, result: ^Matrix4x4) ---
	Matrix4x4_Subtract                    :: proc(m1: ^Matrix4x4, m2: ^Matrix4x4, result: ^Matrix4x4) ---
	Matrix4x4_Multiply                    :: proc(m1: ^Matrix4x4, m2: ^Matrix4x4, result: ^Matrix4x4) ---
	Matrix4x4_MultiplyScalar              :: proc(m: ^Matrix4x4, scalar: f32, result: ^Matrix4x4) ---
	Matrix4x4_Zero                        :: proc(result: ^Matrix4x4) ---
	Matrix4x4_Identity                    :: proc(result: ^Matrix4x4) ---
	Matrix4x4_Rotation                    :: proc(result: ^Matrix4x4, rotation: ^Quat) ---
	Matrix4x4_Translation                 :: proc(result: ^Matrix4x4, translation: ^Vec3) ---
	Matrix4x4_RotationTranslation         :: proc(result: ^Matrix4x4, rotation: ^Quat, translation: ^Vec3) ---
	Matrix4x4_InverseRotationTranslation  :: proc(result: ^Matrix4x4, rotation: ^Quat, translation: ^Vec3) ---
	Matrix4x4_Scale                       :: proc(result: ^Matrix4x4, scale: ^Vec3) ---
	Matrix4x4_Inversed                    :: proc(m: ^Matrix4x4, result: ^Matrix4x4) ---
	Matrix4x4_Transposed                  :: proc(m: ^Matrix4x4, result: ^Matrix4x4) ---
	RMatrix4x4_Zero                       :: proc(result: ^RMatrix4x4) ---
	RMatrix4x4_Identity                   :: proc(result: ^RMatrix4x4) ---
	RMatrix4x4_Rotation                   :: proc(result: ^RMatrix4x4, rotation: ^Quat) ---
	RMatrix4x4_Translation                :: proc(result: ^RMatrix4x4, translation: ^RVec3) ---
	RMatrix4x4_RotationTranslation        :: proc(result: ^RMatrix4x4, rotation: ^Quat, translation: ^RVec3) ---
	RMatrix4x4_InverseRotationTranslation :: proc(result: ^RMatrix4x4, rotation: ^Quat, translation: ^RVec3) ---
	RMatrix4x4_Scale                      :: proc(result: ^RMatrix4x4, scale: ^Vec3) ---
	RMatrix4x4_Inversed                   :: proc(m: ^RMatrix4x4, result: ^RMatrix4x4) ---
	Matrix4x4_GetAxisX                    :: proc(_matrix: ^Matrix4x4, result: ^Vec3) ---
	Matrix4x4_GetAxisY                    :: proc(_matrix: ^Matrix4x4, result: ^Vec3) ---
	Matrix4x4_GetAxisZ                    :: proc(_matrix: ^Matrix4x4, result: ^Vec3) ---
	Matrix4x4_GetTranslation              :: proc(_matrix: ^Matrix4x4, result: ^Vec3) ---
	Matrix4x4_GetQuaternion               :: proc(_matrix: ^Matrix4x4, result: ^Quat) ---

	/* Material */
	PhysicsMaterial_Create        :: proc(name: cstring, color: u32) -> ^PhysicsMaterial ---
	PhysicsMaterial_Destroy       :: proc(material: ^PhysicsMaterial) ---
	PhysicsMaterial_GetDebugName  :: proc(material: ^PhysicsMaterial) -> cstring ---
	PhysicsMaterial_GetDebugColor :: proc(material: ^PhysicsMaterial) -> u32 ---

	/* GroupFilter/GroupFilterTable */
	GroupFilter_Destroy                 :: proc(groupFilter: ^GroupFilter) ---
	GroupFilter_CanCollide              :: proc(groupFilter: ^GroupFilter, group1: ^CollisionGroup, group2: ^CollisionGroup) -> bool ---
	GroupFilterTable_Create             :: proc(numSubGroups: u32) -> ^GroupFilterTable ---
	GroupFilterTable_DisableCollision   :: proc(table: ^GroupFilterTable, subGroup1: CollisionSubGroupID, subGroup2: CollisionSubGroupID) ---
	GroupFilterTable_EnableCollision    :: proc(table: ^GroupFilterTable, subGroup1: CollisionSubGroupID, subGroup2: CollisionSubGroupID) ---
	GroupFilterTable_IsCollisionEnabled :: proc(table: ^GroupFilterTable, subGroup1: CollisionSubGroupID, subGroup2: CollisionSubGroupID) -> bool ---

	/* ShapeSettings */
	ShapeSettings_Destroy     :: proc(settings: ^ShapeSettings) ---
	ShapeSettings_GetUserData :: proc(settings: ^ShapeSettings) -> u64 ---
	ShapeSettings_SetUserData :: proc(settings: ^ShapeSettings, userData: u64) ---

	/* Shape */
	Shape_Destroy                    :: proc(shape: ^Shape) ---
	Shape_GetType                    :: proc(shape: ^Shape) -> ShapeType ---
	Shape_GetSubType                 :: proc(shape: ^Shape) -> ShapeSubType ---
	Shape_GetUserData                :: proc(shape: ^Shape) -> u64 ---
	Shape_SetUserData                :: proc(shape: ^Shape, userData: u64) ---
	Shape_MustBeStatic               :: proc(shape: ^Shape) -> bool ---
	Shape_GetCenterOfMass            :: proc(shape: ^Shape, result: ^Vec3) ---
	Shape_GetLocalBounds             :: proc(shape: ^Shape, result: ^AABox) ---
	Shape_GetSubShapeIDBitsRecursive :: proc(shape: ^Shape) -> u32 ---
	Shape_GetWorldSpaceBounds        :: proc(shape: ^Shape, centerOfMassTransform: ^RMatrix4x4, scale: ^Vec3, result: ^AABox) ---
	Shape_GetInnerRadius             :: proc(shape: ^Shape) -> f32 ---
	Shape_GetMassProperties          :: proc(shape: ^Shape, result: ^MassProperties) ---
	Shape_GetLeafShape               :: proc(shape: ^Shape, subShapeID: SubShapeID, remainder: ^SubShapeID) -> ^Shape ---
	Shape_GetMaterial                :: proc(shape: ^Shape, subShapeID: SubShapeID) -> ^PhysicsMaterial ---
	Shape_GetSurfaceNormal           :: proc(shape: ^Shape, subShapeID: SubShapeID, localPosition: ^Vec3, normal: ^Vec3) ---
	Shape_GetSupportingFace          :: proc(shape: ^Shape, subShapeID: SubShapeID, direction: ^Vec3, scale: ^Vec3, centerOfMassTransform: ^Matrix4x4, outVertices: ^SupportingFace) ---
	Shape_GetVolume                  :: proc(shape: ^Shape) -> f32 ---
	Shape_IsValidScale               :: proc(shape: ^Shape, scale: ^Vec3) -> bool ---
	Shape_MakeScaleValid             :: proc(shape: ^Shape, scale: ^Vec3, result: ^Vec3) ---
	Shape_ScaleShape                 :: proc(shape: ^Shape, scale: ^Vec3) -> ^Shape ---
	Shape_CastRay                    :: proc(shape: ^Shape, origin: ^Vec3, direction: ^Vec3, hit: ^RayCastResult) -> bool ---
	Shape_CastRay2                   :: proc(shape: ^Shape, origin: ^Vec3, direction: ^Vec3, rayCastSettings: ^RayCastSettings, collectorType: CollisionCollectorType, callback: CastRayResultCallback, userData: rawptr, shapeFilter: ^ShapeFilter) -> bool ---
	Shape_CollidePoint               :: proc(shape: ^Shape, point: ^Vec3, shapeFilter: ^ShapeFilter) -> bool ---
	Shape_CollidePoint2              :: proc(shape: ^Shape, point: ^Vec3, collectorType: CollisionCollectorType, callback: CollidePointResultCallback, userData: rawptr, shapeFilter: ^ShapeFilter) -> bool ---

	/* ConvexShape */
	ConvexShapeSettings_GetDensity :: proc(shape: ^ConvexShapeSettings) -> f32 ---
	ConvexShapeSettings_SetDensity :: proc(shape: ^ConvexShapeSettings, value: f32) ---
	ConvexShape_GetDensity         :: proc(shape: ^ConvexShape) -> f32 ---
	ConvexShape_SetDensity         :: proc(shape: ^ConvexShape, inDensity: f32) ---

	/* BoxShape */
	BoxShapeSettings_Create      :: proc(halfExtent: ^Vec3, convexRadius: f32) -> ^BoxShapeSettings ---
	BoxShapeSettings_CreateShape :: proc(settings: ^BoxShapeSettings) -> ^BoxShape ---
	BoxShape_Create              :: proc(halfExtent: ^Vec3, convexRadius: f32) -> ^BoxShape ---
	BoxShape_GetHalfExtent       :: proc(shape: ^BoxShape, halfExtent: ^Vec3) ---
	BoxShape_GetConvexRadius     :: proc(shape: ^BoxShape) -> f32 ---

	/* SphereShape */
	SphereShapeSettings_Create      :: proc(radius: f32) -> ^SphereShapeSettings ---
	SphereShapeSettings_CreateShape :: proc(settings: ^SphereShapeSettings) -> ^SphereShape ---
	SphereShapeSettings_GetRadius   :: proc(settings: ^SphereShapeSettings) -> f32 ---
	SphereShapeSettings_SetRadius   :: proc(settings: ^SphereShapeSettings, radius: f32) ---
	SphereShape_Create              :: proc(radius: f32) -> ^SphereShape ---
	SphereShape_GetRadius           :: proc(shape: ^SphereShape) -> f32 ---

	/* PlaneShape */
	PlaneShapeSettings_Create      :: proc(plane: ^Plane, material: ^PhysicsMaterial, halfExtent: f32) -> ^PlaneShapeSettings ---
	PlaneShapeSettings_CreateShape :: proc(settings: ^PlaneShapeSettings) -> ^PlaneShape ---
	PlaneShape_Create              :: proc(plane: ^Plane, material: ^PhysicsMaterial, halfExtent: f32) -> ^PlaneShape ---
	PlaneShape_GetPlane            :: proc(shape: ^PlaneShape, result: ^Plane) ---
	PlaneShape_GetHalfExtent       :: proc(shape: ^PlaneShape) -> f32 ---

	/* TriangleShape */
	TriangleShapeSettings_Create      :: proc(v1: ^Vec3, v2: ^Vec3, v3: ^Vec3, convexRadius: f32) -> ^TriangleShapeSettings ---
	TriangleShapeSettings_CreateShape :: proc(settings: ^TriangleShapeSettings) -> ^TriangleShape ---
	TriangleShape_Create              :: proc(v1: ^Vec3, v2: ^Vec3, v3: ^Vec3, convexRadius: f32) -> ^TriangleShape ---
	TriangleShape_GetConvexRadius     :: proc(shape: ^TriangleShape) -> f32 ---
	TriangleShape_GetVertex1          :: proc(shape: ^TriangleShape, result: ^Vec3) ---
	TriangleShape_GetVertex2          :: proc(shape: ^TriangleShape, result: ^Vec3) ---
	TriangleShape_GetVertex3          :: proc(shape: ^TriangleShape, result: ^Vec3) ---

	/* CapsuleShape */
	CapsuleShapeSettings_Create          :: proc(halfHeightOfCylinder: f32, radius: f32) -> ^CapsuleShapeSettings ---
	CapsuleShapeSettings_CreateShape     :: proc(settings: ^CapsuleShapeSettings) -> ^CapsuleShape ---
	CapsuleShape_Create                  :: proc(halfHeightOfCylinder: f32, radius: f32) -> ^CapsuleShape ---
	CapsuleShape_GetRadius               :: proc(shape: ^CapsuleShape) -> f32 ---
	CapsuleShape_GetHalfHeightOfCylinder :: proc(shape: ^CapsuleShape) -> f32 ---

	/* CylinderShape */
	CylinderShapeSettings_Create      :: proc(halfHeight: f32, radius: f32, convexRadius: f32) -> ^CylinderShapeSettings ---
	CylinderShapeSettings_CreateShape :: proc(settings: ^CylinderShapeSettings) -> ^CylinderShape ---
	CylinderShape_Create              :: proc(halfHeight: f32, radius: f32) -> ^CylinderShape ---
	CylinderShape_GetRadius           :: proc(shape: ^CylinderShape) -> f32 ---
	CylinderShape_GetHalfHeight       :: proc(shape: ^CylinderShape) -> f32 ---

	/* TaperedCylinderShape */
	TaperedCylinderShapeSettings_Create      :: proc(halfHeightOfTaperedCylinder: f32, topRadius: f32, bottomRadius: f32, convexRadius: f32, material: ^PhysicsMaterial) -> ^TaperedCylinderShapeSettings ---
	TaperedCylinderShapeSettings_CreateShape :: proc(settings: ^TaperedCylinderShapeSettings) -> ^TaperedCylinderShape ---
	TaperedCylinderShape_GetTopRadius        :: proc(shape: ^TaperedCylinderShape) -> f32 ---
	TaperedCylinderShape_GetBottomRadius     :: proc(shape: ^TaperedCylinderShape) -> f32 ---
	TaperedCylinderShape_GetConvexRadius     :: proc(shape: ^TaperedCylinderShape) -> f32 ---
	TaperedCylinderShape_GetHalfHeight       :: proc(shape: ^TaperedCylinderShape) -> f32 ---

	/* ConvexHullShape */
	ConvexHullShapeSettings_Create       :: proc(points: ^Vec3, pointsCount: u32, maxConvexRadius: f32) -> ^ConvexHullShapeSettings ---
	ConvexHullShapeSettings_CreateShape  :: proc(settings: ^ConvexHullShapeSettings) -> ^ConvexHullShape ---
	ConvexHullShape_GetNumPoints         :: proc(shape: ^ConvexHullShape) -> u32 ---
	ConvexHullShape_GetPoint             :: proc(shape: ^ConvexHullShape, index: u32, result: ^Vec3) ---
	ConvexHullShape_GetNumFaces          :: proc(shape: ^ConvexHullShape) -> u32 ---
	ConvexHullShape_GetNumVerticesInFace :: proc(shape: ^ConvexHullShape, faceIndex: u32) -> u32 ---
	ConvexHullShape_GetFaceVertices      :: proc(shape: ^ConvexHullShape, faceIndex: u32, maxVertices: u32, vertices: ^u32) -> u32 ---

	/* MeshShape */
	MeshShapeSettings_Create                         :: proc(triangles: ^Triangle, triangleCount: u32) -> ^MeshShapeSettings ---
	MeshShapeSettings_Create2                        :: proc(vertices: ^Vec3, verticesCount: u32, triangles: ^IndexedTriangle, triangleCount: u32) -> ^MeshShapeSettings ---
	MeshShapeSettings_GetMaxTrianglesPerLeaf         :: proc(settings: ^MeshShapeSettings) -> u32 ---
	MeshShapeSettings_SetMaxTrianglesPerLeaf         :: proc(settings: ^MeshShapeSettings, value: u32) ---
	MeshShapeSettings_GetActiveEdgeCosThresholdAngle :: proc(settings: ^MeshShapeSettings) -> f32 ---
	MeshShapeSettings_SetActiveEdgeCosThresholdAngle :: proc(settings: ^MeshShapeSettings, value: f32) ---
	MeshShapeSettings_GetPerTriangleUserData         :: proc(settings: ^MeshShapeSettings) -> bool ---
	MeshShapeSettings_SetPerTriangleUserData         :: proc(settings: ^MeshShapeSettings, value: bool) ---
	MeshShapeSettings_GetBuildQuality                :: proc(settings: ^MeshShapeSettings) -> Mesh_Shape_BuildQuality ---
	MeshShapeSettings_SetBuildQuality                :: proc(settings: ^MeshShapeSettings, value: Mesh_Shape_BuildQuality) ---
	MeshShapeSettings_Sanitize                       :: proc(settings: ^MeshShapeSettings) ---
	MeshShapeSettings_CreateShape                    :: proc(settings: ^MeshShapeSettings) -> ^MeshShape ---
	MeshShape_GetTriangleUserData                    :: proc(shape: ^MeshShape, id: SubShapeID) -> u32 ---

	/* HeightFieldShape */
	HeightFieldShapeSettings_Create                         :: proc(samples: ^f32, offset: ^Vec3, scale: ^Vec3, sampleCount: u32) -> ^HeightFieldShapeSettings ---
	HeightFieldShapeSettings_CreateShape                    :: proc(settings: ^HeightFieldShapeSettings) -> ^HeightFieldShape ---
	HeightFieldShapeSettings_DetermineMinAndMaxSample       :: proc(settings: ^HeightFieldShapeSettings, pOutMinValue: ^f32, pOutMaxValue: ^f32, pOutQuantizationScale: ^f32) ---
	HeightFieldShapeSettings_CalculateBitsPerSampleForError :: proc(settings: ^HeightFieldShapeSettings, maxError: f32) -> u32 ---
	HeightFieldShape_GetSampleCount                         :: proc(shape: ^HeightFieldShape) -> u32 ---
	HeightFieldShape_GetBlockSize                           :: proc(shape: ^HeightFieldShape) -> u32 ---
	HeightFieldShape_GetMaterial                            :: proc(shape: ^HeightFieldShape, x: u32, y: u32) -> ^PhysicsMaterial ---
	HeightFieldShape_GetPosition                            :: proc(shape: ^HeightFieldShape, x: u32, y: u32, result: ^Vec3) ---
	HeightFieldShape_IsNoCollision                          :: proc(shape: ^HeightFieldShape, x: u32, y: u32) -> bool ---
	HeightFieldShape_ProjectOntoSurface                     :: proc(shape: ^HeightFieldShape, localPosition: ^Vec3, outSurfacePosition: ^Vec3, outSubShapeID: ^SubShapeID) -> bool ---
	HeightFieldShape_GetMinHeightValue                      :: proc(shape: ^HeightFieldShape) -> f32 ---
	HeightFieldShape_GetMaxHeightValue                      :: proc(shape: ^HeightFieldShape) -> f32 ---

	/* TaperedCapsuleShape */
	TaperedCapsuleShapeSettings_Create      :: proc(halfHeightOfTaperedCylinder: f32, topRadius: f32, bottomRadius: f32) -> ^TaperedCapsuleShapeSettings ---
	TaperedCapsuleShapeSettings_CreateShape :: proc(settings: ^TaperedCapsuleShapeSettings) -> ^TaperedCapsuleShape ---
	TaperedCapsuleShape_GetTopRadius        :: proc(shape: ^TaperedCapsuleShape) -> f32 ---
	TaperedCapsuleShape_GetBottomRadius     :: proc(shape: ^TaperedCapsuleShape) -> f32 ---
	TaperedCapsuleShape_GetHalfHeight       :: proc(shape: ^TaperedCapsuleShape) -> f32 ---

	/* CompoundShape */
	CompoundShapeSettings_AddShape       :: proc(settings: ^CompoundShapeSettings, position: ^Vec3, rotation: ^Quat, shapeSettings: ^ShapeSettings, userData: u32) ---
	CompoundShapeSettings_AddShape2      :: proc(settings: ^CompoundShapeSettings, position: ^Vec3, rotation: ^Quat, shape: ^Shape, userData: u32) ---
	CompoundShape_GetNumSubShapes        :: proc(shape: ^CompoundShape) -> u32 ---
	CompoundShape_GetSubShape            :: proc(shape: ^CompoundShape, index: u32, subShape: ^^Shape, positionCOM: ^Vec3, rotation: ^Quat, userData: ^u32) ---
	CompoundShape_GetSubShapeIndexFromID :: proc(shape: ^CompoundShape, id: SubShapeID, remainder: ^SubShapeID) -> u32 ---

	/* StaticCompoundShape */
	StaticCompoundShapeSettings_Create :: proc() -> ^StaticCompoundShapeSettings ---
	StaticCompoundShape_Create         :: proc(settings: ^StaticCompoundShapeSettings) -> ^StaticCompoundShape ---

	/* MutableCompoundShape */
	MutableCompoundShapeSettings_Create     :: proc() -> ^MutableCompoundShapeSettings ---
	MutableCompoundShape_Create             :: proc(settings: ^MutableCompoundShapeSettings) -> ^MutableCompoundShape ---
	MutableCompoundShape_AddShape           :: proc(shape: ^MutableCompoundShape, position: ^Vec3, rotation: ^Quat, child: ^Shape, userData: u32, index: u32) -> u32 ---
	MutableCompoundShape_RemoveShape        :: proc(shape: ^MutableCompoundShape, index: u32) ---
	MutableCompoundShape_ModifyShape        :: proc(shape: ^MutableCompoundShape, index: u32, position: ^Vec3, rotation: ^Quat) ---
	MutableCompoundShape_ModifyShape2       :: proc(shape: ^MutableCompoundShape, index: u32, position: ^Vec3, rotation: ^Quat, newShape: ^Shape) ---
	MutableCompoundShape_AdjustCenterOfMass :: proc(shape: ^MutableCompoundShape) ---

	/* DecoratedShape */
	DecoratedShape_GetInnerShape :: proc(shape: ^DecoratedShape) -> ^Shape ---

	/* RotatedTranslatedShape */
	RotatedTranslatedShapeSettings_Create      :: proc(position: ^Vec3, rotation: ^Quat, shapeSettings: ^ShapeSettings) -> ^RotatedTranslatedShapeSettings ---
	RotatedTranslatedShapeSettings_Create2     :: proc(position: ^Vec3, rotation: ^Quat, shape: ^Shape) -> ^RotatedTranslatedShapeSettings ---
	RotatedTranslatedShapeSettings_CreateShape :: proc(settings: ^RotatedTranslatedShapeSettings) -> ^RotatedTranslatedShape ---
	RotatedTranslatedShape_Create              :: proc(position: ^Vec3, rotation: ^Quat, shape: ^Shape) -> ^RotatedTranslatedShape ---
	RotatedTranslatedShape_GetPosition         :: proc(shape: ^RotatedTranslatedShape, position: ^Vec3) ---
	RotatedTranslatedShape_GetRotation         :: proc(shape: ^RotatedTranslatedShape, rotation: ^Quat) ---

	/* ScaledShape */
	ScaledShapeSettings_Create      :: proc(shapeSettings: ^ShapeSettings, scale: ^Vec3) -> ^ScaledShapeSettings ---
	ScaledShapeSettings_Create2     :: proc(shape: ^Shape, scale: ^Vec3) -> ^ScaledShapeSettings ---
	ScaledShapeSettings_CreateShape :: proc(settings: ^ScaledShapeSettings) -> ^ScaledShape ---
	ScaledShape_Create              :: proc(shape: ^Shape, scale: ^Vec3) -> ^ScaledShape ---
	ScaledShape_GetScale            :: proc(shape: ^ScaledShape, result: ^Vec3) ---

	/* OffsetCenterOfMassShape */
	OffsetCenterOfMassShapeSettings_Create      :: proc(offset: ^Vec3, shapeSettings: ^ShapeSettings) -> ^OffsetCenterOfMassShapeSettings ---
	OffsetCenterOfMassShapeSettings_Create2     :: proc(offset: ^Vec3, shape: ^Shape) -> ^OffsetCenterOfMassShapeSettings ---
	OffsetCenterOfMassShapeSettings_CreateShape :: proc(settings: ^OffsetCenterOfMassShapeSettings) -> ^OffsetCenterOfMassShape ---
	OffsetCenterOfMassShape_Create              :: proc(offset: ^Vec3, shape: ^Shape) -> ^OffsetCenterOfMassShape ---
	OffsetCenterOfMassShape_GetOffset           :: proc(shape: ^OffsetCenterOfMassShape, result: ^Vec3) ---

	/* EmptyShape */
	EmptyShapeSettings_Create      :: proc(centerOfMass: ^Vec3) -> ^EmptyShapeSettings ---
	EmptyShapeSettings_CreateShape :: proc(settings: ^EmptyShapeSettings) -> ^EmptyShape ---

	/* BodyCreationSettings */
	BodyCreationSettings_Create                          :: proc() -> ^BodyCreationSettings ---
	BodyCreationSettings_Create2                         :: proc(settings: ^ShapeSettings, position: ^RVec3, rotation: ^Quat, motionType: MotionType, objectLayer: ObjectLayer) -> ^BodyCreationSettings ---
	BodyCreationSettings_Create3                         :: proc(shape: ^Shape, position: ^RVec3, rotation: ^Quat, motionType: MotionType, objectLayer: ObjectLayer) -> ^BodyCreationSettings ---
	BodyCreationSettings_Destroy                         :: proc(settings: ^BodyCreationSettings) ---
	BodyCreationSettings_GetPosition                     :: proc(settings: ^BodyCreationSettings, result: ^RVec3) ---
	BodyCreationSettings_SetPosition                     :: proc(settings: ^BodyCreationSettings, value: ^RVec3) ---
	BodyCreationSettings_GetRotation                     :: proc(settings: ^BodyCreationSettings, result: ^Quat) ---
	BodyCreationSettings_SetRotation                     :: proc(settings: ^BodyCreationSettings, value: ^Quat) ---
	BodyCreationSettings_GetLinearVelocity               :: proc(settings: ^BodyCreationSettings, velocity: ^Vec3) ---
	BodyCreationSettings_SetLinearVelocity               :: proc(settings: ^BodyCreationSettings, velocity: ^Vec3) ---
	BodyCreationSettings_GetAngularVelocity              :: proc(settings: ^BodyCreationSettings, velocity: ^Vec3) ---
	BodyCreationSettings_SetAngularVelocity              :: proc(settings: ^BodyCreationSettings, velocity: ^Vec3) ---
	BodyCreationSettings_GetUserData                     :: proc(settings: ^BodyCreationSettings) -> u64 ---
	BodyCreationSettings_SetUserData                     :: proc(settings: ^BodyCreationSettings, value: u64) ---
	BodyCreationSettings_GetObjectLayer                  :: proc(settings: ^BodyCreationSettings) -> ObjectLayer ---
	BodyCreationSettings_SetObjectLayer                  :: proc(settings: ^BodyCreationSettings, value: ObjectLayer) ---
	BodyCreationSettings_GetCollissionGroup              :: proc(settings: ^BodyCreationSettings, result: ^CollisionGroup) ---
	BodyCreationSettings_SetCollissionGroup              :: proc(settings: ^BodyCreationSettings, value: ^CollisionGroup) ---
	BodyCreationSettings_GetMotionType                   :: proc(settings: ^BodyCreationSettings) -> MotionType ---
	BodyCreationSettings_SetMotionType                   :: proc(settings: ^BodyCreationSettings, value: MotionType) ---
	BodyCreationSettings_GetAllowedDOFs                  :: proc(settings: ^BodyCreationSettings) -> AllowedDOFs ---
	BodyCreationSettings_SetAllowedDOFs                  :: proc(settings: ^BodyCreationSettings, value: AllowedDOFs) ---
	BodyCreationSettings_GetAllowDynamicOrKinematic      :: proc(settings: ^BodyCreationSettings) -> bool ---
	BodyCreationSettings_SetAllowDynamicOrKinematic      :: proc(settings: ^BodyCreationSettings, value: bool) ---
	BodyCreationSettings_GetIsSensor                     :: proc(settings: ^BodyCreationSettings) -> bool ---
	BodyCreationSettings_SetIsSensor                     :: proc(settings: ^BodyCreationSettings, value: bool) ---
	BodyCreationSettings_GetCollideKinematicVsNonDynamic :: proc(settings: ^BodyCreationSettings) -> bool ---
	BodyCreationSettings_SetCollideKinematicVsNonDynamic :: proc(settings: ^BodyCreationSettings, value: bool) ---
	BodyCreationSettings_GetUseManifoldReduction         :: proc(settings: ^BodyCreationSettings) -> bool ---
	BodyCreationSettings_SetUseManifoldReduction         :: proc(settings: ^BodyCreationSettings, value: bool) ---
	BodyCreationSettings_GetApplyGyroscopicForce         :: proc(settings: ^BodyCreationSettings) -> bool ---
	BodyCreationSettings_SetApplyGyroscopicForce         :: proc(settings: ^BodyCreationSettings, value: bool) ---
	BodyCreationSettings_GetMotionQuality                :: proc(settings: ^BodyCreationSettings) -> MotionQuality ---
	BodyCreationSettings_SetMotionQuality                :: proc(settings: ^BodyCreationSettings, value: MotionQuality) ---
	BodyCreationSettings_GetEnhancedInternalEdgeRemoval  :: proc(settings: ^BodyCreationSettings) -> bool ---
	BodyCreationSettings_SetEnhancedInternalEdgeRemoval  :: proc(settings: ^BodyCreationSettings, value: bool) ---
	BodyCreationSettings_GetAllowSleeping                :: proc(settings: ^BodyCreationSettings) -> bool ---
	BodyCreationSettings_SetAllowSleeping                :: proc(settings: ^BodyCreationSettings, value: bool) ---
	BodyCreationSettings_GetFriction                     :: proc(settings: ^BodyCreationSettings) -> f32 ---
	BodyCreationSettings_SetFriction                     :: proc(settings: ^BodyCreationSettings, value: f32) ---
	BodyCreationSettings_GetRestitution                  :: proc(settings: ^BodyCreationSettings) -> f32 ---
	BodyCreationSettings_SetRestitution                  :: proc(settings: ^BodyCreationSettings, value: f32) ---
	BodyCreationSettings_GetLinearDamping                :: proc(settings: ^BodyCreationSettings) -> f32 ---
	BodyCreationSettings_SetLinearDamping                :: proc(settings: ^BodyCreationSettings, value: f32) ---
	BodyCreationSettings_GetAngularDamping               :: proc(settings: ^BodyCreationSettings) -> f32 ---
	BodyCreationSettings_SetAngularDamping               :: proc(settings: ^BodyCreationSettings, value: f32) ---
	BodyCreationSettings_GetMaxLinearVelocity            :: proc(settings: ^BodyCreationSettings) -> f32 ---
	BodyCreationSettings_SetMaxLinearVelocity            :: proc(settings: ^BodyCreationSettings, value: f32) ---
	BodyCreationSettings_GetMaxAngularVelocity           :: proc(settings: ^BodyCreationSettings) -> f32 ---
	BodyCreationSettings_SetMaxAngularVelocity           :: proc(settings: ^BodyCreationSettings, value: f32) ---
	BodyCreationSettings_GetGravityFactor                :: proc(settings: ^BodyCreationSettings) -> f32 ---
	BodyCreationSettings_SetGravityFactor                :: proc(settings: ^BodyCreationSettings, value: f32) ---
	BodyCreationSettings_GetNumVelocityStepsOverride     :: proc(settings: ^BodyCreationSettings) -> u32 ---
	BodyCreationSettings_SetNumVelocityStepsOverride     :: proc(settings: ^BodyCreationSettings, value: u32) ---
	BodyCreationSettings_GetNumPositionStepsOverride     :: proc(settings: ^BodyCreationSettings) -> u32 ---
	BodyCreationSettings_SetNumPositionStepsOverride     :: proc(settings: ^BodyCreationSettings, value: u32) ---
	BodyCreationSettings_GetOverrideMassProperties       :: proc(settings: ^BodyCreationSettings) -> OverrideMassProperties ---
	BodyCreationSettings_SetOverrideMassProperties       :: proc(settings: ^BodyCreationSettings, value: OverrideMassProperties) ---
	BodyCreationSettings_GetInertiaMultiplier            :: proc(settings: ^BodyCreationSettings) -> f32 ---
	BodyCreationSettings_SetInertiaMultiplier            :: proc(settings: ^BodyCreationSettings, value: f32) ---
	BodyCreationSettings_GetMassPropertiesOverride       :: proc(settings: ^BodyCreationSettings, result: ^MassProperties) ---
	BodyCreationSettings_SetMassPropertiesOverride       :: proc(settings: ^BodyCreationSettings, massProperties: ^MassProperties) ---

	/* SoftBodyCreationSettings */
	SoftBodyCreationSettings_Create  :: proc() -> ^SoftBodyCreationSettings ---
	SoftBodyCreationSettings_Destroy :: proc(settings: ^SoftBodyCreationSettings) ---

	/* Constraint */
	Constraint_Destroy                     :: proc(constraint: ^Constraint) ---
	Constraint_GetType                     :: proc(constraint: ^Constraint) -> ConstraintType ---
	Constraint_GetSubType                  :: proc(constraint: ^Constraint) -> ConstraintSubType ---
	Constraint_GetConstraintPriority       :: proc(constraint: ^Constraint) -> u32 ---
	Constraint_SetConstraintPriority       :: proc(constraint: ^Constraint, priority: u32) ---
	Constraint_GetNumVelocityStepsOverride :: proc(constraint: ^Constraint) -> u32 ---
	Constraint_SetNumVelocityStepsOverride :: proc(constraint: ^Constraint, value: u32) ---
	Constraint_GetNumPositionStepsOverride :: proc(constraint: ^Constraint) -> u32 ---
	Constraint_SetNumPositionStepsOverride :: proc(constraint: ^Constraint, value: u32) ---
	Constraint_GetEnabled                  :: proc(constraint: ^Constraint) -> bool ---
	Constraint_SetEnabled                  :: proc(constraint: ^Constraint, enabled: bool) ---
	Constraint_GetUserData                 :: proc(constraint: ^Constraint) -> u64 ---
	Constraint_SetUserData                 :: proc(constraint: ^Constraint, userData: u64) ---
	Constraint_NotifyShapeChanged          :: proc(constraint: ^Constraint, bodyID: BodyID, deltaCOM: ^Vec3) ---
	Constraint_ResetWarmStart              :: proc(constraint: ^Constraint) ---
	Constraint_IsActive                    :: proc(constraint: ^Constraint) -> bool ---
	Constraint_SetupVelocityConstraint     :: proc(constraint: ^Constraint, deltaTime: f32) ---
	Constraint_WarmStartVelocityConstraint :: proc(constraint: ^Constraint, warmStartImpulseRatio: f32) ---
	Constraint_SolveVelocityConstraint     :: proc(constraint: ^Constraint, deltaTime: f32) -> bool ---
	Constraint_SolvePositionConstraint     :: proc(constraint: ^Constraint, deltaTime: f32, baumgarte: f32) -> bool ---

	/* TwoBodyConstraint */
	TwoBodyConstraint_GetBody1                   :: proc(constraint: ^TwoBodyConstraint) -> ^Body ---
	TwoBodyConstraint_GetBody2                   :: proc(constraint: ^TwoBodyConstraint) -> ^Body ---
	TwoBodyConstraint_GetConstraintToBody1Matrix :: proc(constraint: ^TwoBodyConstraint, result: ^Matrix4x4) ---
	TwoBodyConstraint_GetConstraintToBody2Matrix :: proc(constraint: ^TwoBodyConstraint, result: ^Matrix4x4) ---

	/* FixedConstraint */
	FixedConstraintSettings_Init           :: proc(settings: ^FixedConstraintSettings) ---
	FixedConstraint_Create                 :: proc(settings: ^FixedConstraintSettings, body1: ^Body, body2: ^Body) -> ^FixedConstraint ---
	FixedConstraint_GetSettings            :: proc(constraint: ^FixedConstraint, settings: ^FixedConstraintSettings) ---
	FixedConstraint_GetTotalLambdaPosition :: proc(constraint: ^FixedConstraint, result: ^Vec3) ---
	FixedConstraint_GetTotalLambdaRotation :: proc(constraint: ^FixedConstraint, result: ^Vec3) ---

	/* DistanceConstraint */
	DistanceConstraintSettings_Init            :: proc(settings: ^DistanceConstraintSettings) ---
	DistanceConstraint_Create                  :: proc(settings: ^DistanceConstraintSettings, body1: ^Body, body2: ^Body) -> ^DistanceConstraint ---
	DistanceConstraint_GetSettings             :: proc(constraint: ^DistanceConstraint, settings: ^DistanceConstraintSettings) ---
	DistanceConstraint_SetDistance             :: proc(constraint: ^DistanceConstraint, minDistance: f32, maxDistance: f32) ---
	DistanceConstraint_GetMinDistance          :: proc(constraint: ^DistanceConstraint) -> f32 ---
	DistanceConstraint_GetMaxDistance          :: proc(constraint: ^DistanceConstraint) -> f32 ---
	DistanceConstraint_GetLimitsSpringSettings :: proc(constraint: ^DistanceConstraint, result: ^SpringSettings) ---
	DistanceConstraint_SetLimitsSpringSettings :: proc(constraint: ^DistanceConstraint, settings: ^SpringSettings) ---
	DistanceConstraint_GetTotalLambdaPosition  :: proc(constraint: ^DistanceConstraint) -> f32 ---

	/* PointConstraint */
	PointConstraintSettings_Init           :: proc(settings: ^PointConstraintSettings) ---
	PointConstraint_Create                 :: proc(settings: ^PointConstraintSettings, body1: ^Body, body2: ^Body) -> ^PointConstraint ---
	PointConstraint_GetSettings            :: proc(constraint: ^PointConstraint, settings: ^PointConstraintSettings) ---
	PointConstraint_SetPoint1              :: proc(constraint: ^PointConstraint, space: ConstraintSpace, value: ^RVec3) ---
	PointConstraint_SetPoint2              :: proc(constraint: ^PointConstraint, space: ConstraintSpace, value: ^RVec3) ---
	PointConstraint_GetLocalSpacePoint1    :: proc(constraint: ^PointConstraint, result: ^Vec3) ---
	PointConstraint_GetLocalSpacePoint2    :: proc(constraint: ^PointConstraint, result: ^Vec3) ---
	PointConstraint_GetTotalLambdaPosition :: proc(constraint: ^PointConstraint, result: ^Vec3) ---

	/* HingeConstraint */
	HingeConstraintSettings_Init                 :: proc(settings: ^HingeConstraintSettings) ---
	HingeConstraint_Create                       :: proc(settings: ^HingeConstraintSettings, body1: ^Body, body2: ^Body) -> ^HingeConstraint ---
	HingeConstraint_GetSettings                  :: proc(constraint: ^HingeConstraint, settings: ^HingeConstraintSettings) ---
	HingeConstraint_GetLocalSpacePoint1          :: proc(constraint: ^HingeConstraint, result: ^Vec3) ---
	HingeConstraint_GetLocalSpacePoint2          :: proc(constraint: ^HingeConstraint, result: ^Vec3) ---
	HingeConstraint_GetLocalSpaceHingeAxis1      :: proc(constraint: ^HingeConstraint, result: ^Vec3) ---
	HingeConstraint_GetLocalSpaceHingeAxis2      :: proc(constraint: ^HingeConstraint, result: ^Vec3) ---
	HingeConstraint_GetLocalSpaceNormalAxis1     :: proc(constraint: ^HingeConstraint, result: ^Vec3) ---
	HingeConstraint_GetLocalSpaceNormalAxis2     :: proc(constraint: ^HingeConstraint, result: ^Vec3) ---
	HingeConstraint_GetCurrentAngle              :: proc(constraint: ^HingeConstraint) -> f32 ---
	HingeConstraint_SetMaxFrictionTorque         :: proc(constraint: ^HingeConstraint, frictionTorque: f32) ---
	HingeConstraint_GetMaxFrictionTorque         :: proc(constraint: ^HingeConstraint) -> f32 ---
	HingeConstraint_SetMotorSettings             :: proc(constraint: ^HingeConstraint, settings: ^MotorSettings) ---
	HingeConstraint_GetMotorSettings             :: proc(constraint: ^HingeConstraint, result: ^MotorSettings) ---
	HingeConstraint_SetMotorState                :: proc(constraint: ^HingeConstraint, state: MotorState) ---
	HingeConstraint_GetMotorState                :: proc(constraint: ^HingeConstraint) -> MotorState ---
	HingeConstraint_SetTargetAngularVelocity     :: proc(constraint: ^HingeConstraint, angularVelocity: f32) ---
	HingeConstraint_GetTargetAngularVelocity     :: proc(constraint: ^HingeConstraint) -> f32 ---
	HingeConstraint_SetTargetAngle               :: proc(constraint: ^HingeConstraint, angle: f32) ---
	HingeConstraint_GetTargetAngle               :: proc(constraint: ^HingeConstraint) -> f32 ---
	HingeConstraint_SetLimits                    :: proc(constraint: ^HingeConstraint, inLimitsMin: f32, inLimitsMax: f32) ---
	HingeConstraint_GetLimitsMin                 :: proc(constraint: ^HingeConstraint) -> f32 ---
	HingeConstraint_GetLimitsMax                 :: proc(constraint: ^HingeConstraint) -> f32 ---
	HingeConstraint_HasLimits                    :: proc(constraint: ^HingeConstraint) -> bool ---
	HingeConstraint_GetLimitsSpringSettings      :: proc(constraint: ^HingeConstraint, result: ^SpringSettings) ---
	HingeConstraint_SetLimitsSpringSettings      :: proc(constraint: ^HingeConstraint, settings: ^SpringSettings) ---
	HingeConstraint_GetTotalLambdaPosition       :: proc(constraint: ^HingeConstraint, result: ^Vec3) ---
	HingeConstraint_GetTotalLambdaRotation       :: proc(constraint: ^HingeConstraint, rotation: ^f32) ---
	HingeConstraint_GetTotalLambdaRotationLimits :: proc(constraint: ^HingeConstraint) -> f32 ---
	HingeConstraint_GetTotalLambdaMotor          :: proc(constraint: ^HingeConstraint) -> f32 ---

	/* SliderConstraint */
	SliderConstraintSettings_Init                 :: proc(settings: ^SliderConstraintSettings) ---
	SliderConstraintSettings_SetSliderAxis        :: proc(settings: ^SliderConstraintSettings, axis: ^Vec3) ---
	SliderConstraint_Create                       :: proc(settings: ^SliderConstraintSettings, body1: ^Body, body2: ^Body) -> ^SliderConstraint ---
	SliderConstraint_GetSettings                  :: proc(constraint: ^SliderConstraint, settings: ^SliderConstraintSettings) ---
	SliderConstraint_GetCurrentPosition           :: proc(constraint: ^SliderConstraint) -> f32 ---
	SliderConstraint_SetMaxFrictionForce          :: proc(constraint: ^SliderConstraint, frictionForce: f32) ---
	SliderConstraint_GetMaxFrictionForce          :: proc(constraint: ^SliderConstraint) -> f32 ---
	SliderConstraint_SetMotorSettings             :: proc(constraint: ^SliderConstraint, settings: ^MotorSettings) ---
	SliderConstraint_GetMotorSettings             :: proc(constraint: ^SliderConstraint, result: ^MotorSettings) ---
	SliderConstraint_SetMotorState                :: proc(constraint: ^SliderConstraint, state: MotorState) ---
	SliderConstraint_GetMotorState                :: proc(constraint: ^SliderConstraint) -> MotorState ---
	SliderConstraint_SetTargetVelocity            :: proc(constraint: ^SliderConstraint, velocity: f32) ---
	SliderConstraint_GetTargetVelocity            :: proc(constraint: ^SliderConstraint) -> f32 ---
	SliderConstraint_SetTargetPosition            :: proc(constraint: ^SliderConstraint, position: f32) ---
	SliderConstraint_GetTargetPosition            :: proc(constraint: ^SliderConstraint) -> f32 ---
	SliderConstraint_SetLimits                    :: proc(constraint: ^SliderConstraint, inLimitsMin: f32, inLimitsMax: f32) ---
	SliderConstraint_GetLimitsMin                 :: proc(constraint: ^SliderConstraint) -> f32 ---
	SliderConstraint_GetLimitsMax                 :: proc(constraint: ^SliderConstraint) -> f32 ---
	SliderConstraint_HasLimits                    :: proc(constraint: ^SliderConstraint) -> bool ---
	SliderConstraint_GetLimitsSpringSettings      :: proc(constraint: ^SliderConstraint, result: ^SpringSettings) ---
	SliderConstraint_SetLimitsSpringSettings      :: proc(constraint: ^SliderConstraint, settings: ^SpringSettings) ---
	SliderConstraint_GetTotalLambdaPosition       :: proc(constraint: ^SliderConstraint, position: ^f32) ---
	SliderConstraint_GetTotalLambdaPositionLimits :: proc(constraint: ^SliderConstraint) -> f32 ---
	SliderConstraint_GetTotalLambdaRotation       :: proc(constraint: ^SliderConstraint, result: ^Vec3) ---
	SliderConstraint_GetTotalLambdaMotor          :: proc(constraint: ^SliderConstraint) -> f32 ---

	/* ConeConstraint */
	ConeConstraintSettings_Init           :: proc(settings: ^ConeConstraintSettings) ---
	ConeConstraint_Create                 :: proc(settings: ^ConeConstraintSettings, body1: ^Body, body2: ^Body) -> ^ConeConstraint ---
	ConeConstraint_GetSettings            :: proc(constraint: ^ConeConstraint, settings: ^ConeConstraintSettings) ---
	ConeConstraint_SetHalfConeAngle       :: proc(constraint: ^ConeConstraint, halfConeAngle: f32) ---
	ConeConstraint_GetCosHalfConeAngle    :: proc(constraint: ^ConeConstraint) -> f32 ---
	ConeConstraint_GetTotalLambdaPosition :: proc(constraint: ^ConeConstraint, result: ^Vec3) ---
	ConeConstraint_GetTotalLambdaRotation :: proc(constraint: ^ConeConstraint) -> f32 ---

	/* SwingTwistConstraint */
	SwingTwistConstraintSettings_Init           :: proc(settings: ^SwingTwistConstraintSettings) ---
	SwingTwistConstraint_Create                 :: proc(settings: ^SwingTwistConstraintSettings, body1: ^Body, body2: ^Body) -> ^SwingTwistConstraint ---
	SwingTwistConstraint_GetSettings            :: proc(constraint: ^SwingTwistConstraint, settings: ^SwingTwistConstraintSettings) ---
	SwingTwistConstraint_GetNormalHalfConeAngle :: proc(constraint: ^SwingTwistConstraint) -> f32 ---
	SwingTwistConstraint_GetTotalLambdaPosition :: proc(constraint: ^SwingTwistConstraint, result: ^Vec3) ---
	SwingTwistConstraint_GetTotalLambdaTwist    :: proc(constraint: ^SwingTwistConstraint) -> f32 ---
	SwingTwistConstraint_GetTotalLambdaSwingY   :: proc(constraint: ^SwingTwistConstraint) -> f32 ---
	SwingTwistConstraint_GetTotalLambdaSwingZ   :: proc(constraint: ^SwingTwistConstraint) -> f32 ---
	SwingTwistConstraint_GetTotalLambdaMotor    :: proc(constraint: ^SwingTwistConstraint, result: ^Vec3) ---

	/* SixDOFConstraint */
	SixDOFConstraintSettings_Init                   :: proc(settings: ^SixDOFConstraintSettings) ---
	SixDOFConstraintSettings_MakeFreeAxis           :: proc(settings: ^SixDOFConstraintSettings, axis: SixDOFConstraintAxis) ---
	SixDOFConstraintSettings_IsFreeAxis             :: proc(settings: ^SixDOFConstraintSettings, axis: SixDOFConstraintAxis) -> bool ---
	SixDOFConstraintSettings_MakeFixedAxis          :: proc(settings: ^SixDOFConstraintSettings, axis: SixDOFConstraintAxis) ---
	SixDOFConstraintSettings_IsFixedAxis            :: proc(settings: ^SixDOFConstraintSettings, axis: SixDOFConstraintAxis) -> bool ---
	SixDOFConstraintSettings_SetLimitedAxis         :: proc(settings: ^SixDOFConstraintSettings, axis: SixDOFConstraintAxis, min: f32, max: f32) ---
	SixDOFConstraint_Create                         :: proc(settings: ^SixDOFConstraintSettings, body1: ^Body, body2: ^Body) -> ^SixDOFConstraint ---
	SixDOFConstraint_GetSettings                    :: proc(constraint: ^SixDOFConstraint, settings: ^SixDOFConstraintSettings) ---
	SixDOFConstraint_GetLimitsMin                   :: proc(constraint: ^SixDOFConstraint, axis: SixDOFConstraintAxis) -> f32 ---
	SixDOFConstraint_GetLimitsMax                   :: proc(constraint: ^SixDOFConstraint, axis: SixDOFConstraintAxis) -> f32 ---
	SixDOFConstraint_GetTotalLambdaPosition         :: proc(constraint: ^SixDOFConstraint, result: ^Vec3) ---
	SixDOFConstraint_GetTotalLambdaRotation         :: proc(constraint: ^SixDOFConstraint, result: ^Vec3) ---
	SixDOFConstraint_GetTotalLambdaMotorTranslation :: proc(constraint: ^SixDOFConstraint, result: ^Vec3) ---
	SixDOFConstraint_GetTotalLambdaMotorRotation    :: proc(constraint: ^SixDOFConstraint, result: ^Vec3) ---
	SixDOFConstraint_GetTranslationLimitsMin        :: proc(constraint: ^SixDOFConstraint, result: ^Vec3) ---
	SixDOFConstraint_GetTranslationLimitsMax        :: proc(constraint: ^SixDOFConstraint, result: ^Vec3) ---
	SixDOFConstraint_GetRotationLimitsMin           :: proc(constraint: ^SixDOFConstraint, result: ^Vec3) ---
	SixDOFConstraint_GetRotationLimitsMax           :: proc(constraint: ^SixDOFConstraint, result: ^Vec3) ---
	SixDOFConstraint_IsFixedAxis                    :: proc(constraint: ^SixDOFConstraint, axis: SixDOFConstraintAxis) -> bool ---
	SixDOFConstraint_IsFreeAxis                     :: proc(constraint: ^SixDOFConstraint, axis: SixDOFConstraintAxis) -> bool ---
	SixDOFConstraint_GetLimitsSpringSettings        :: proc(constraint: ^SixDOFConstraint, result: ^SpringSettings, axis: SixDOFConstraintAxis) ---
	SixDOFConstraint_SetLimitsSpringSettings        :: proc(constraint: ^SixDOFConstraint, settings: ^SpringSettings, axis: SixDOFConstraintAxis) ---
	SixDOFConstraint_SetMaxFriction                 :: proc(constraint: ^SixDOFConstraint, axis: SixDOFConstraintAxis, inFriction: f32) ---
	SixDOFConstraint_GetMaxFriction                 :: proc(constraint: ^SixDOFConstraint, axis: SixDOFConstraintAxis) -> f32 ---
	SixDOFConstraint_GetRotationInConstraintSpace   :: proc(constraint: ^SixDOFConstraint, result: ^Quat) ---
	SixDOFConstraint_GetMotorSettings               :: proc(constraint: ^SixDOFConstraint, axis: SixDOFConstraintAxis, settings: ^MotorSettings) ---
	SixDOFConstraint_SetMotorState                  :: proc(constraint: ^SixDOFConstraint, axis: SixDOFConstraintAxis, state: MotorState) ---
	SixDOFConstraint_GetMotorState                  :: proc(constraint: ^SixDOFConstraint, axis: SixDOFConstraintAxis) -> MotorState ---
	SixDOFConstraint_SetTargetVelocityCS            :: proc(constraint: ^SixDOFConstraint, inVelocity: ^Vec3) ---
	SixDOFConstraint_GetTargetVelocityCS            :: proc(constraint: ^SixDOFConstraint, result: ^Vec3) ---
	SixDOFConstraint_SetTargetAngularVelocityCS     :: proc(constraint: ^SixDOFConstraint, inAngularVelocity: ^Vec3) ---
	SixDOFConstraint_GetTargetAngularVelocityCS     :: proc(constraint: ^SixDOFConstraint, result: ^Vec3) ---
	SixDOFConstraint_SetTargetPositionCS            :: proc(constraint: ^SixDOFConstraint, inPosition: ^Vec3) ---
	SixDOFConstraint_GetTargetPositionCS            :: proc(constraint: ^SixDOFConstraint, result: ^Vec3) ---
	SixDOFConstraint_SetTargetOrientationCS         :: proc(constraint: ^SixDOFConstraint, inOrientation: ^Quat) ---
	SixDOFConstraint_GetTargetOrientationCS         :: proc(constraint: ^SixDOFConstraint, result: ^Quat) ---
	SixDOFConstraint_SetTargetOrientationBS         :: proc(constraint: ^SixDOFConstraint, inOrientation: ^Quat) ---

	/* GearConstraint */
	GearConstraintSettings_Init   :: proc(settings: ^GearConstraintSettings) ---
	GearConstraint_Create         :: proc(settings: ^GearConstraintSettings, body1: ^Body, body2: ^Body) -> ^GearConstraint ---
	GearConstraint_GetSettings    :: proc(constraint: ^GearConstraint, settings: ^GearConstraintSettings) ---
	GearConstraint_SetConstraints :: proc(constraint: ^GearConstraint, gear1: ^Constraint, gear2: ^Constraint) ---
	GearConstraint_GetTotalLambda :: proc(constraint: ^GearConstraint) -> f32 ---

	/* BodyInterface */
	BodyInterface_DestroyBody                       :: proc(interface: ^BodyInterface, bodyID: BodyID) ---
	BodyInterface_CreateAndAddBody                  :: proc(interface: ^BodyInterface, settings: ^BodyCreationSettings, activationMode: Activation) -> BodyID ---
	BodyInterface_CreateBody                        :: proc(interface: ^BodyInterface, settings: ^BodyCreationSettings) -> ^Body ---
	BodyInterface_CreateBodyWithID                  :: proc(interface: ^BodyInterface, bodyID: BodyID, settings: ^BodyCreationSettings) -> ^Body ---
	BodyInterface_CreateBodyWithoutID               :: proc(interface: ^BodyInterface, settings: ^BodyCreationSettings) -> ^Body ---
	BodyInterface_DestroyBodyWithoutID              :: proc(interface: ^BodyInterface, body: ^Body) ---
	BodyInterface_AssignBodyID                      :: proc(interface: ^BodyInterface, body: ^Body) -> bool ---
	BodyInterface_AssignBodyID2                     :: proc(interface: ^BodyInterface, body: ^Body, bodyID: BodyID) -> bool ---
	BodyInterface_UnassignBodyID                    :: proc(interface: ^BodyInterface, bodyID: BodyID) -> ^Body ---
	BodyInterface_CreateSoftBody                    :: proc(interface: ^BodyInterface, settings: ^SoftBodyCreationSettings) -> ^Body ---
	BodyInterface_CreateSoftBodyWithID              :: proc(interface: ^BodyInterface, bodyID: BodyID, settings: ^SoftBodyCreationSettings) -> ^Body ---
	BodyInterface_CreateSoftBodyWithoutID           :: proc(interface: ^BodyInterface, settings: ^SoftBodyCreationSettings) -> ^Body ---
	BodyInterface_CreateAndAddSoftBody              :: proc(interface: ^BodyInterface, settings: ^SoftBodyCreationSettings, activationMode: Activation) -> BodyID ---
	BodyInterface_AddBody                           :: proc(interface: ^BodyInterface, bodyID: BodyID, activationMode: Activation) ---
	BodyInterface_RemoveBody                        :: proc(interface: ^BodyInterface, bodyID: BodyID) ---
	BodyInterface_RemoveAndDestroyBody              :: proc(interface: ^BodyInterface, bodyID: BodyID) ---
	BodyInterface_IsActive                          :: proc(interface: ^BodyInterface, bodyID: BodyID) -> bool ---
	BodyInterface_IsAdded                           :: proc(interface: ^BodyInterface, bodyID: BodyID) -> bool ---
	BodyInterface_GetBodyType                       :: proc(interface: ^BodyInterface, bodyID: BodyID) -> BodyType ---
	BodyInterface_SetLinearVelocity                 :: proc(interface: ^BodyInterface, bodyID: BodyID, velocity: ^Vec3) ---
	BodyInterface_GetLinearVelocity                 :: proc(interface: ^BodyInterface, bodyID: BodyID, velocity: ^Vec3) ---
	BodyInterface_GetCenterOfMassPosition           :: proc(interface: ^BodyInterface, bodyID: BodyID, position: ^RVec3) ---
	BodyInterface_GetMotionType                     :: proc(interface: ^BodyInterface, bodyID: BodyID) -> MotionType ---
	BodyInterface_SetMotionType                     :: proc(interface: ^BodyInterface, bodyID: BodyID, motionType: MotionType, activationMode: Activation) ---
	BodyInterface_GetRestitution                    :: proc(interface: ^BodyInterface, bodyID: BodyID) -> f32 ---
	BodyInterface_SetRestitution                    :: proc(interface: ^BodyInterface, bodyID: BodyID, restitution: f32) ---
	BodyInterface_GetFriction                       :: proc(interface: ^BodyInterface, bodyID: BodyID) -> f32 ---
	BodyInterface_SetFriction                       :: proc(interface: ^BodyInterface, bodyID: BodyID, friction: f32) ---
	BodyInterface_SetPosition                       :: proc(interface: ^BodyInterface, bodyId: BodyID, position: ^RVec3, activationMode: Activation) ---
	BodyInterface_GetPosition                       :: proc(interface: ^BodyInterface, bodyId: BodyID, result: ^RVec3) ---
	BodyInterface_SetRotation                       :: proc(interface: ^BodyInterface, bodyId: BodyID, rotation: ^Quat, activationMode: Activation) ---
	BodyInterface_GetRotation                       :: proc(interface: ^BodyInterface, bodyId: BodyID, result: ^Quat) ---
	BodyInterface_SetPositionAndRotation            :: proc(interface: ^BodyInterface, bodyId: BodyID, position: ^RVec3, rotation: ^Quat, activationMode: Activation) ---
	BodyInterface_SetPositionAndRotationWhenChanged :: proc(interface: ^BodyInterface, bodyId: BodyID, position: ^RVec3, rotation: ^Quat, activationMode: Activation) ---
	BodyInterface_GetPositionAndRotation            :: proc(interface: ^BodyInterface, bodyId: BodyID, position: ^RVec3, rotation: ^Quat) ---
	BodyInterface_SetPositionRotationAndVelocity    :: proc(interface: ^BodyInterface, bodyId: BodyID, position: ^RVec3, rotation: ^Quat, linearVelocity: ^Vec3, angularVelocity: ^Vec3) ---
	BodyInterface_GetCollissionGroup                :: proc(interface: ^BodyInterface, bodyId: BodyID, result: ^CollisionGroup) ---
	BodyInterface_SetCollissionGroup                :: proc(interface: ^BodyInterface, bodyId: BodyID, group: ^CollisionGroup) ---
	BodyInterface_GetShape                          :: proc(interface: ^BodyInterface, bodyId: BodyID) -> ^Shape ---
	BodyInterface_SetShape                          :: proc(interface: ^BodyInterface, bodyId: BodyID, shape: ^Shape, updateMassProperties: bool, activationMode: Activation) ---
	BodyInterface_NotifyShapeChanged                :: proc(interface: ^BodyInterface, bodyId: BodyID, previousCenterOfMass: ^Vec3, updateMassProperties: bool, activationMode: Activation) ---
	BodyInterface_ActivateBody                      :: proc(interface: ^BodyInterface, bodyId: BodyID) ---
	BodyInterface_DeactivateBody                    :: proc(interface: ^BodyInterface, bodyId: BodyID) ---
	BodyInterface_GetObjectLayer                    :: proc(interface: ^BodyInterface, bodyId: BodyID) -> ObjectLayer ---
	BodyInterface_SetObjectLayer                    :: proc(interface: ^BodyInterface, bodyId: BodyID, layer: ObjectLayer) ---
	BodyInterface_GetWorldTransform                 :: proc(interface: ^BodyInterface, bodyId: BodyID, result: ^RMatrix4x4) ---
	BodyInterface_GetCenterOfMassTransform          :: proc(interface: ^BodyInterface, bodyId: BodyID, result: ^RMatrix4x4) ---
	BodyInterface_MoveKinematic                     :: proc(interface: ^BodyInterface, bodyId: BodyID, targetPosition: ^RVec3, targetRotation: ^Quat, deltaTime: f32) ---
	BodyInterface_ApplyBuoyancyImpulse              :: proc(interface: ^BodyInterface, bodyId: BodyID, surfacePosition: ^RVec3, surfaceNormal: ^Vec3, buoyancy: f32, linearDrag: f32, angularDrag: f32, fluidVelocity: ^Vec3, gravity: ^Vec3, deltaTime: f32) -> bool ---
	BodyInterface_SetLinearAndAngularVelocity       :: proc(interface: ^BodyInterface, bodyId: BodyID, linearVelocity: ^Vec3, angularVelocity: ^Vec3) ---
	BodyInterface_GetLinearAndAngularVelocity       :: proc(interface: ^BodyInterface, bodyId: BodyID, linearVelocity: ^Vec3, angularVelocity: ^Vec3) ---
	BodyInterface_AddLinearVelocity                 :: proc(interface: ^BodyInterface, bodyId: BodyID, linearVelocity: ^Vec3) ---
	BodyInterface_AddLinearAndAngularVelocity       :: proc(interface: ^BodyInterface, bodyId: BodyID, linearVelocity: ^Vec3, angularVelocity: ^Vec3) ---
	BodyInterface_SetAngularVelocity                :: proc(interface: ^BodyInterface, bodyId: BodyID, angularVelocity: ^Vec3) ---
	BodyInterface_GetAngularVelocity                :: proc(interface: ^BodyInterface, bodyId: BodyID, angularVelocity: ^Vec3) ---
	BodyInterface_GetPointVelocity                  :: proc(interface: ^BodyInterface, bodyId: BodyID, point: ^RVec3, velocity: ^Vec3) ---
	BodyInterface_AddForce                          :: proc(interface: ^BodyInterface, bodyId: BodyID, force: ^Vec3) ---
	BodyInterface_AddForce2                         :: proc(interface: ^BodyInterface, bodyId: BodyID, force: ^Vec3, point: ^RVec3) ---
	BodyInterface_AddTorque                         :: proc(interface: ^BodyInterface, bodyId: BodyID, torque: ^Vec3) ---
	BodyInterface_AddForceAndTorque                 :: proc(interface: ^BodyInterface, bodyId: BodyID, force: ^Vec3, torque: ^Vec3) ---
	BodyInterface_AddImpulse                        :: proc(interface: ^BodyInterface, bodyId: BodyID, impulse: ^Vec3) ---
	BodyInterface_AddImpulse2                       :: proc(interface: ^BodyInterface, bodyId: BodyID, impulse: ^Vec3, point: ^RVec3) ---
	BodyInterface_AddAngularImpulse                 :: proc(interface: ^BodyInterface, bodyId: BodyID, angularImpulse: ^Vec3) ---
	BodyInterface_SetMotionQuality                  :: proc(interface: ^BodyInterface, bodyId: BodyID, quality: MotionQuality) ---
	BodyInterface_GetMotionQuality                  :: proc(interface: ^BodyInterface, bodyId: BodyID) -> MotionQuality ---
	BodyInterface_GetInverseInertia                 :: proc(interface: ^BodyInterface, bodyId: BodyID, result: ^Matrix4x4) ---
	BodyInterface_SetGravityFactor                  :: proc(interface: ^BodyInterface, bodyId: BodyID, value: f32) ---
	BodyInterface_GetGravityFactor                  :: proc(interface: ^BodyInterface, bodyId: BodyID) -> f32 ---
	BodyInterface_SetUseManifoldReduction           :: proc(interface: ^BodyInterface, bodyId: BodyID, value: bool) ---
	BodyInterface_GetUseManifoldReduction           :: proc(interface: ^BodyInterface, bodyId: BodyID) -> bool ---
	BodyInterface_SetUserData                       :: proc(interface: ^BodyInterface, bodyId: BodyID, inUserData: u64) ---
	BodyInterface_GetUserData                       :: proc(interface: ^BodyInterface, bodyId: BodyID) -> u64 ---
	BodyInterface_GetMaterial                       :: proc(interface: ^BodyInterface, bodyId: BodyID, subShapeID: SubShapeID) -> ^PhysicsMaterial ---
	BodyInterface_InvalidateContactCache            :: proc(interface: ^BodyInterface, bodyId: BodyID) ---

	//--------------------------------------------------------------------------------------------------
	// BodyLockInterface
	//--------------------------------------------------------------------------------------------------
	BodyLockInterface_LockRead       :: proc(lockInterface: ^BodyLockInterface, bodyID: BodyID, outLock: ^BodyLockRead) ---
	BodyLockInterface_UnlockRead     :: proc(lockInterface: ^BodyLockInterface, ioLock: ^BodyLockRead) ---
	BodyLockInterface_LockWrite      :: proc(lockInterface: ^BodyLockInterface, bodyID: BodyID, outLock: ^BodyLockWrite) ---
	BodyLockInterface_UnlockWrite    :: proc(lockInterface: ^BodyLockInterface, ioLock: ^BodyLockWrite) ---
	BodyLockInterface_LockMultiRead  :: proc(lockInterface: ^BodyLockInterface, bodyIDs: ^BodyID, count: u32) -> ^BodyLockMultiRead ---
	BodyLockMultiRead_Destroy        :: proc(ioLock: ^BodyLockMultiRead) ---
	BodyLockMultiRead_GetBody        :: proc(ioLock: ^BodyLockMultiRead, bodyIndex: u32) -> ^Body ---
	BodyLockInterface_LockMultiWrite :: proc(lockInterface: ^BodyLockInterface, bodyIDs: ^BodyID, count: u32) -> ^BodyLockMultiWrite ---
	BodyLockMultiWrite_Destroy       :: proc(ioLock: ^BodyLockMultiWrite) ---
	BodyLockMultiWrite_GetBody       :: proc(ioLock: ^BodyLockMultiWrite, bodyIndex: u32) -> ^Body ---

	//--------------------------------------------------------------------------------------------------
	// MotionProperties
	//--------------------------------------------------------------------------------------------------
	MotionProperties_GetAllowedDOFs            :: proc(properties: ^MotionProperties) -> AllowedDOFs ---
	MotionProperties_SetLinearDamping          :: proc(properties: ^MotionProperties, damping: f32) ---
	MotionProperties_GetLinearDamping          :: proc(properties: ^MotionProperties) -> f32 ---
	MotionProperties_SetAngularDamping         :: proc(properties: ^MotionProperties, damping: f32) ---
	MotionProperties_GetAngularDamping         :: proc(properties: ^MotionProperties) -> f32 ---
	MotionProperties_SetMassProperties         :: proc(properties: ^MotionProperties, allowedDOFs: AllowedDOFs, massProperties: ^MassProperties) ---
	MotionProperties_GetInverseMassUnchecked   :: proc(properties: ^MotionProperties) -> f32 ---
	MotionProperties_SetInverseMass            :: proc(properties: ^MotionProperties, inverseMass: f32) ---
	MotionProperties_GetInverseInertiaDiagonal :: proc(properties: ^MotionProperties, result: ^Vec3) ---
	MotionProperties_GetInertiaRotation        :: proc(properties: ^MotionProperties, result: ^Quat) ---
	MotionProperties_SetInverseInertia         :: proc(properties: ^MotionProperties, diagonal: ^Vec3, rot: ^Quat) ---
	MotionProperties_ScaleToMass               :: proc(properties: ^MotionProperties, mass: f32) ---

	//--------------------------------------------------------------------------------------------------
	// RayCast
	//--------------------------------------------------------------------------------------------------
	RayCast_GetPointOnRay  :: proc(origin: ^Vec3, direction: ^Vec3, fraction: f32, result: ^Vec3) ---
	RRayCast_GetPointOnRay :: proc(origin: ^RVec3, direction: ^Vec3, fraction: f32, result: ^RVec3) ---

	//--------------------------------------------------------------------------------------------------
	// MassProperties
	//--------------------------------------------------------------------------------------------------
	MassProperties_DecomposePrincipalMomentsOfInertia :: proc(properties: ^MassProperties, rotation: ^Matrix4x4, diagonal: ^Vec3) ---
	MassProperties_ScaleToMass                        :: proc(properties: ^MassProperties, mass: f32) ---
	MassProperties_GetEquivalentSolidBoxSize          :: proc(mass: f32, inertiaDiagonal: ^Vec3, result: ^Vec3) ---

	//--------------------------------------------------------------------------------------------------
	// CollideShapeSettings
	//--------------------------------------------------------------------------------------------------
	CollideShapeSettings_Init :: proc(settings: ^CollideShapeSettings) ---

	//--------------------------------------------------------------------------------------------------
	// ShapeCastSettings
	//--------------------------------------------------------------------------------------------------
	ShapeCastSettings_Init :: proc(settings: ^ShapeCastSettings) ---

	//--------------------------------------------------------------------------------------------------
	// BroadPhaseQuery
	//--------------------------------------------------------------------------------------------------
	BroadPhaseQuery_CastRay       :: proc(query: ^BroadPhaseQuery, origin: ^Vec3, direction: ^Vec3, callback: RayCastBodyCollectorCallback, userData: rawptr, broadPhaseLayerFilter: ^BroadPhaseLayerFilter, objectLayerFilter: ^ObjectLayerFilter) -> bool ---
	BroadPhaseQuery_CastRay2      :: proc(query: ^BroadPhaseQuery, origin: ^Vec3, direction: ^Vec3, collectorType: CollisionCollectorType, callback: RayCastBodyResultCallback, userData: rawptr, broadPhaseLayerFilter: ^BroadPhaseLayerFilter, objectLayerFilter: ^ObjectLayerFilter) -> bool ---
	BroadPhaseQuery_CollideAABox  :: proc(query: ^BroadPhaseQuery, box: ^AABox, callback: CollideShapeBodyCollectorCallback, userData: rawptr, broadPhaseLayerFilter: ^BroadPhaseLayerFilter, objectLayerFilter: ^ObjectLayerFilter) -> bool ---
	BroadPhaseQuery_CollideSphere :: proc(query: ^BroadPhaseQuery, center: ^Vec3, radius: f32, callback: CollideShapeBodyCollectorCallback, userData: rawptr, broadPhaseLayerFilter: ^BroadPhaseLayerFilter, objectLayerFilter: ^ObjectLayerFilter) -> bool ---
	BroadPhaseQuery_CollidePoint  :: proc(query: ^BroadPhaseQuery, point: ^Vec3, callback: CollideShapeBodyCollectorCallback, userData: rawptr, broadPhaseLayerFilter: ^BroadPhaseLayerFilter, objectLayerFilter: ^ObjectLayerFilter) -> bool ---

	//--------------------------------------------------------------------------------------------------
	// NarrowPhaseQuery
	//--------------------------------------------------------------------------------------------------
	NarrowPhaseQuery_CastRay       :: proc(query: ^NarrowPhaseQuery, origin: ^RVec3, direction: ^Vec3, hit: ^RayCastResult, broadPhaseLayerFilter: ^BroadPhaseLayerFilter, objectLayerFilter: ^ObjectLayerFilter, bodyFilter: ^BodyFilter) -> bool ---
	NarrowPhaseQuery_CastRay2      :: proc(query: ^NarrowPhaseQuery, origin: ^RVec3, direction: ^Vec3, rayCastSettings: ^RayCastSettings, callback: CastRayCollectorCallback, userData: rawptr, broadPhaseLayerFilter: ^BroadPhaseLayerFilter, objectLayerFilter: ^ObjectLayerFilter, bodyFilter: ^BodyFilter, shapeFilter: ^ShapeFilter) -> bool ---
	NarrowPhaseQuery_CastRay3      :: proc(query: ^NarrowPhaseQuery, origin: ^RVec3, direction: ^Vec3, rayCastSettings: ^RayCastSettings, collectorType: CollisionCollectorType, callback: CastRayResultCallback, userData: rawptr, broadPhaseLayerFilter: ^BroadPhaseLayerFilter, objectLayerFilter: ^ObjectLayerFilter, bodyFilter: ^BodyFilter, shapeFilter: ^ShapeFilter) -> bool ---
	NarrowPhaseQuery_CollidePoint  :: proc(query: ^NarrowPhaseQuery, point: ^RVec3, callback: CollidePointCollectorCallback, userData: rawptr, broadPhaseLayerFilter: ^BroadPhaseLayerFilter, objectLayerFilter: ^ObjectLayerFilter, bodyFilter: ^BodyFilter, shapeFilter: ^ShapeFilter) -> bool ---
	NarrowPhaseQuery_CollidePoint2 :: proc(query: ^NarrowPhaseQuery, point: ^RVec3, collectorType: CollisionCollectorType, callback: CollidePointResultCallback, userData: rawptr, broadPhaseLayerFilter: ^BroadPhaseLayerFilter, objectLayerFilter: ^ObjectLayerFilter, bodyFilter: ^BodyFilter, shapeFilter: ^ShapeFilter) -> bool ---
	NarrowPhaseQuery_CollideShape  :: proc(query: ^NarrowPhaseQuery, shape: ^Shape, scale: ^Vec3, centerOfMassTransform: ^RMatrix4x4, settings: ^CollideShapeSettings, baseOffset: ^RVec3, callback: CollideShapeCollectorCallback, userData: rawptr, broadPhaseLayerFilter: ^BroadPhaseLayerFilter, objectLayerFilter: ^ObjectLayerFilter, bodyFilter: ^BodyFilter, shapeFilter: ^ShapeFilter) -> bool ---
	NarrowPhaseQuery_CollideShape2 :: proc(query: ^NarrowPhaseQuery, shape: ^Shape, scale: ^Vec3, centerOfMassTransform: ^RMatrix4x4, settings: ^CollideShapeSettings, baseOffset: ^RVec3, collectorType: CollisionCollectorType, callback: CollideShapeResultCallback, userData: rawptr, broadPhaseLayerFilter: ^BroadPhaseLayerFilter, objectLayerFilter: ^ObjectLayerFilter, bodyFilter: ^BodyFilter, shapeFilter: ^ShapeFilter) -> bool ---
	NarrowPhaseQuery_CastShape     :: proc(query: ^NarrowPhaseQuery, shape: ^Shape, worldTransform: ^RMatrix4x4, direction: ^Vec3, settings: ^ShapeCastSettings, baseOffset: ^RVec3, callback: CastShapeCollectorCallback, userData: rawptr, broadPhaseLayerFilter: ^BroadPhaseLayerFilter, objectLayerFilter: ^ObjectLayerFilter, bodyFilter: ^BodyFilter, shapeFilter: ^ShapeFilter) -> bool ---
	NarrowPhaseQuery_CastShape2    :: proc(query: ^NarrowPhaseQuery, shape: ^Shape, worldTransform: ^RMatrix4x4, direction: ^Vec3, settings: ^ShapeCastSettings, baseOffset: ^RVec3, collectorType: CollisionCollectorType, callback: CastShapeResultCallback, userData: rawptr, broadPhaseLayerFilter: ^BroadPhaseLayerFilter, objectLayerFilter: ^ObjectLayerFilter, bodyFilter: ^BodyFilter, shapeFilter: ^ShapeFilter) -> bool ---

	//--------------------------------------------------------------------------------------------------
	// Body
	//--------------------------------------------------------------------------------------------------
	Body_GetID                                  :: proc(body: ^Body) -> BodyID ---
	Body_GetBodyType                            :: proc(body: ^Body) -> BodyType ---
	Body_IsRigidBody                            :: proc(body: ^Body) -> bool ---
	Body_IsSoftBody                             :: proc(body: ^Body) -> bool ---
	Body_IsActive                               :: proc(body: ^Body) -> bool ---
	Body_IsStatic                               :: proc(body: ^Body) -> bool ---
	Body_IsKinematic                            :: proc(body: ^Body) -> bool ---
	Body_IsDynamic                              :: proc(body: ^Body) -> bool ---
	Body_CanBeKinematicOrDynamic                :: proc(body: ^Body) -> bool ---
	Body_SetIsSensor                            :: proc(body: ^Body, value: bool) ---
	Body_IsSensor                               :: proc(body: ^Body) -> bool ---
	Body_SetCollideKinematicVsNonDynamic        :: proc(body: ^Body, value: bool) ---
	Body_GetCollideKinematicVsNonDynamic        :: proc(body: ^Body) -> bool ---
	Body_SetUseManifoldReduction                :: proc(body: ^Body, value: bool) ---
	Body_GetUseManifoldReduction                :: proc(body: ^Body) -> bool ---
	Body_GetUseManifoldReductionWithBody        :: proc(body: ^Body, other: ^Body) -> bool ---
	Body_SetApplyGyroscopicForce                :: proc(body: ^Body, value: bool) ---
	Body_GetApplyGyroscopicForce                :: proc(body: ^Body) -> bool ---
	Body_SetEnhancedInternalEdgeRemoval         :: proc(body: ^Body, value: bool) ---
	Body_GetEnhancedInternalEdgeRemoval         :: proc(body: ^Body) -> bool ---
	Body_GetEnhancedInternalEdgeRemovalWithBody :: proc(body: ^Body, other: ^Body) -> bool ---
	Body_GetMotionType                          :: proc(body: ^Body) -> MotionType ---
	Body_SetMotionType                          :: proc(body: ^Body, motionType: MotionType) ---
	Body_GetBroadPhaseLayer                     :: proc(body: ^Body) -> BroadPhaseLayer ---
	Body_GetObjectLayer                         :: proc(body: ^Body) -> ObjectLayer ---
	Body_GetCollissionGroup                     :: proc(body: ^Body, result: ^CollisionGroup) ---
	Body_SetCollissionGroup                     :: proc(body: ^Body, value: ^CollisionGroup) ---
	Body_GetAllowSleeping                       :: proc(body: ^Body) -> bool ---
	Body_SetAllowSleeping                       :: proc(body: ^Body, allowSleeping: bool) ---
	Body_ResetSleepTimer                        :: proc(body: ^Body) ---
	Body_GetFriction                            :: proc(body: ^Body) -> f32 ---
	Body_SetFriction                            :: proc(body: ^Body, friction: f32) ---
	Body_GetRestitution                         :: proc(body: ^Body) -> f32 ---
	Body_SetRestitution                         :: proc(body: ^Body, restitution: f32) ---
	Body_GetLinearVelocity                      :: proc(body: ^Body, velocity: ^Vec3) ---
	Body_SetLinearVelocity                      :: proc(body: ^Body, velocity: ^Vec3) ---
	Body_SetLinearVelocityClamped               :: proc(body: ^Body, velocity: ^Vec3) ---
	Body_GetAngularVelocity                     :: proc(body: ^Body, velocity: ^Vec3) ---
	Body_SetAngularVelocity                     :: proc(body: ^Body, velocity: ^Vec3) ---
	Body_SetAngularVelocityClamped              :: proc(body: ^Body, velocity: ^Vec3) ---
	Body_GetPointVelocityCOM                    :: proc(body: ^Body, pointRelativeToCOM: ^Vec3, velocity: ^Vec3) ---
	Body_GetPointVelocity                       :: proc(body: ^Body, point: ^RVec3, velocity: ^Vec3) ---
	Body_AddForce                               :: proc(body: ^Body, force: ^Vec3) ---
	Body_AddForceAtPosition                     :: proc(body: ^Body, force: ^Vec3, position: ^RVec3) ---
	Body_AddTorque                              :: proc(body: ^Body, force: ^Vec3) ---
	Body_GetAccumulatedForce                    :: proc(body: ^Body, force: ^Vec3) ---
	Body_GetAccumulatedTorque                   :: proc(body: ^Body, force: ^Vec3) ---
	Body_ResetForce                             :: proc(body: ^Body) ---
	Body_ResetTorque                            :: proc(body: ^Body) ---
	Body_ResetMotion                            :: proc(body: ^Body) ---
	Body_GetInverseInertia                      :: proc(body: ^Body, result: ^Matrix4x4) ---
	Body_AddImpulse                             :: proc(body: ^Body, impulse: ^Vec3) ---
	Body_AddImpulseAtPosition                   :: proc(body: ^Body, impulse: ^Vec3, position: ^RVec3) ---
	Body_AddAngularImpulse                      :: proc(body: ^Body, angularImpulse: ^Vec3) ---
	Body_MoveKinematic                          :: proc(body: ^Body, targetPosition: ^RVec3, targetRotation: ^Quat, deltaTime: f32) ---
	Body_ApplyBuoyancyImpulse                   :: proc(body: ^Body, surfacePosition: ^RVec3, surfaceNormal: ^Vec3, buoyancy: f32, linearDrag: f32, angularDrag: f32, fluidVelocity: ^Vec3, gravity: ^Vec3, deltaTime: f32) -> bool ---
	Body_IsInBroadPhase                         :: proc(body: ^Body) -> bool ---
	Body_IsCollisionCacheInvalid                :: proc(body: ^Body) -> bool ---
	Body_GetShape                               :: proc(body: ^Body) -> ^Shape ---
	Body_GetPosition                            :: proc(body: ^Body, result: ^RVec3) ---
	Body_GetRotation                            :: proc(body: ^Body, result: ^Quat) ---
	Body_GetWorldTransform                      :: proc(body: ^Body, result: ^RMatrix4x4) ---
	Body_GetCenterOfMassPosition                :: proc(body: ^Body, result: ^RVec3) ---
	Body_GetCenterOfMassTransform               :: proc(body: ^Body, result: ^RMatrix4x4) ---
	Body_GetInverseCenterOfMassTransform        :: proc(body: ^Body, result: ^RMatrix4x4) ---
	Body_GetWorldSpaceBounds                    :: proc(body: ^Body, result: ^AABox) ---
	Body_GetWorldSpaceSurfaceNormal             :: proc(body: ^Body, subShapeID: SubShapeID, position: ^RVec3, normal: ^Vec3) ---
	Body_GetMotionProperties                    :: proc(body: ^Body) -> ^MotionProperties ---
	Body_GetMotionPropertiesUnchecked           :: proc(body: ^Body) -> ^MotionProperties ---
	Body_SetUserData                            :: proc(body: ^Body, userData: u64) ---
	Body_GetUserData                            :: proc(body: ^Body) -> u64 ---
	Body_GetFixedToWorldBody                    :: proc() -> ^Body ---
	BroadPhaseLayerFilter_SetProcs              :: proc(procs: ^BroadPhaseLayerFilter_Procs) ---
	BroadPhaseLayerFilter_Create                :: proc(userData: rawptr) -> ^BroadPhaseLayerFilter ---
	BroadPhaseLayerFilter_Destroy               :: proc(filter: ^BroadPhaseLayerFilter) ---
	ObjectLayerFilter_SetProcs                  :: proc(procs: ^ObjectLayerFilter_Procs) ---
	ObjectLayerFilter_Create                    :: proc(userData: rawptr) -> ^ObjectLayerFilter ---
	ObjectLayerFilter_Destroy                   :: proc(filter: ^ObjectLayerFilter) ---
	BodyFilter_SetProcs                         :: proc(procs: ^BodyFilter_Procs) ---
	BodyFilter_Create                           :: proc(userData: rawptr) -> ^BodyFilter ---
	BodyFilter_Destroy                          :: proc(filter: ^BodyFilter) ---
	ShapeFilter_SetProcs                        :: proc(procs: ^ShapeFilter_Procs) ---
	ShapeFilter_Create                          :: proc(userData: rawptr) -> ^ShapeFilter ---
	ShapeFilter_Destroy                         :: proc(filter: ^ShapeFilter) ---
	ShapeFilter_GetBodyID2                      :: proc(filter: ^ShapeFilter) -> BodyID ---
	ShapeFilter_SetBodyID2                      :: proc(filter: ^ShapeFilter, id: BodyID) ---
	SimShapeFilter_SetProcs                     :: proc(procs: ^SimShapeFilter_Procs) ---
	SimShapeFilter_Create                       :: proc(userData: rawptr) -> ^SimShapeFilter ---
	SimShapeFilter_Destroy                      :: proc(filter: ^SimShapeFilter) ---
	ContactListener_SetProcs                    :: proc(procs: ^ContactListener_Procs) ---
	ContactListener_Create                      :: proc(userData: rawptr) -> ^ContactListener ---
	ContactListener_Destroy                     :: proc(listener: ^ContactListener) ---
	BodyActivationListener_SetProcs             :: proc(procs: ^BodyActivationListener_Procs) ---
	BodyActivationListener_Create               :: proc(userData: rawptr) -> ^BodyActivationListener ---
	BodyActivationListener_Destroy              :: proc(listener: ^BodyActivationListener) ---
	BodyDrawFilter_SetProcs                     :: proc(procs: ^BodyDrawFilter_Procs) ---
	BodyDrawFilter_Create                       :: proc(userData: rawptr) -> ^BodyDrawFilter ---
	BodyDrawFilter_Destroy                      :: proc(filter: ^BodyDrawFilter) ---

	/* ContactManifold */
	ContactManifold_GetWorldSpaceNormal          :: proc(manifold: ^ContactManifold, result: ^Vec3) ---
	ContactManifold_GetPenetrationDepth          :: proc(manifold: ^ContactManifold) -> f32 ---
	ContactManifold_GetSubShapeID1               :: proc(manifold: ^ContactManifold) -> SubShapeID ---
	ContactManifold_GetSubShapeID2               :: proc(manifold: ^ContactManifold) -> SubShapeID ---
	ContactManifold_GetPointCount                :: proc(manifold: ^ContactManifold) -> u32 ---
	ContactManifold_GetWorldSpaceContactPointOn1 :: proc(manifold: ^ContactManifold, index: u32, result: ^RVec3) ---
	ContactManifold_GetWorldSpaceContactPointOn2 :: proc(manifold: ^ContactManifold, index: u32, result: ^RVec3) ---

	/* ContactSettings */
	ContactSettings_GetFriction                       :: proc(settings: ^ContactSettings) -> f32 ---
	ContactSettings_SetFriction                       :: proc(settings: ^ContactSettings, friction: f32) ---
	ContactSettings_GetRestitution                    :: proc(settings: ^ContactSettings) -> f32 ---
	ContactSettings_SetRestitution                    :: proc(settings: ^ContactSettings, restitution: f32) ---
	ContactSettings_GetInvMassScale1                  :: proc(settings: ^ContactSettings) -> f32 ---
	ContactSettings_SetInvMassScale1                  :: proc(settings: ^ContactSettings, scale: f32) ---
	ContactSettings_GetInvInertiaScale1               :: proc(settings: ^ContactSettings) -> f32 ---
	ContactSettings_SetInvInertiaScale1               :: proc(settings: ^ContactSettings, scale: f32) ---
	ContactSettings_GetInvMassScale2                  :: proc(settings: ^ContactSettings) -> f32 ---
	ContactSettings_SetInvMassScale2                  :: proc(settings: ^ContactSettings, scale: f32) ---
	ContactSettings_GetInvInertiaScale2               :: proc(settings: ^ContactSettings) -> f32 ---
	ContactSettings_SetInvInertiaScale2               :: proc(settings: ^ContactSettings, scale: f32) ---
	ContactSettings_GetIsSensor                       :: proc(settings: ^ContactSettings) -> bool ---
	ContactSettings_SetIsSensor                       :: proc(settings: ^ContactSettings, sensor: bool) ---
	ContactSettings_GetRelativeLinearSurfaceVelocity  :: proc(settings: ^ContactSettings, result: ^Vec3) ---
	ContactSettings_SetRelativeLinearSurfaceVelocity  :: proc(settings: ^ContactSettings, velocity: ^Vec3) ---
	ContactSettings_GetRelativeAngularSurfaceVelocity :: proc(settings: ^ContactSettings, result: ^Vec3) ---
	ContactSettings_SetRelativeAngularSurfaceVelocity :: proc(settings: ^ContactSettings, velocity: ^Vec3) ---

	/* CharacterBase */
	CharacterBase_Destroy             :: proc(character: ^CharacterBase) ---
	CharacterBase_GetCosMaxSlopeAngle :: proc(character: ^CharacterBase) -> f32 ---
	CharacterBase_SetMaxSlopeAngle    :: proc(character: ^CharacterBase, maxSlopeAngle: f32) ---
	CharacterBase_GetUp               :: proc(character: ^CharacterBase, result: ^Vec3) ---
	CharacterBase_SetUp               :: proc(character: ^CharacterBase, value: ^Vec3) ---
	CharacterBase_IsSlopeTooSteep     :: proc(character: ^CharacterBase, value: ^Vec3) -> bool ---
	CharacterBase_GetShape            :: proc(character: ^CharacterBase) -> ^Shape ---
	CharacterBase_GetGroundState      :: proc(character: ^CharacterBase) -> GroundState ---
	CharacterBase_IsSupported         :: proc(character: ^CharacterBase) -> bool ---
	CharacterBase_GetGroundPosition   :: proc(character: ^CharacterBase, position: ^RVec3) ---
	CharacterBase_GetGroundNormal     :: proc(character: ^CharacterBase, normal: ^Vec3) ---
	CharacterBase_GetGroundVelocity   :: proc(character: ^CharacterBase, velocity: ^Vec3) ---
	CharacterBase_GetGroundMaterial   :: proc(character: ^CharacterBase) -> ^PhysicsMaterial ---
	CharacterBase_GetGroundBodyId     :: proc(character: ^CharacterBase) -> BodyID ---
	CharacterBase_GetGroundSubShapeId :: proc(character: ^CharacterBase) -> SubShapeID ---
	CharacterBase_GetGroundUserData   :: proc(character: ^CharacterBase) -> u64 ---

	/* CharacterSettings */
	CharacterSettings_Init :: proc(settings: ^CharacterSettings) ---

	/* Character */
	Character_Create                      :: proc(settings: ^CharacterSettings, position: ^RVec3, rotation: ^Quat, userData: u64, system: ^PhysicsSystem) -> ^Character ---
	Character_AddToPhysicsSystem          :: proc(character: ^Character, activationMode: Activation, lockBodies: bool) ---
	Character_RemoveFromPhysicsSystem     :: proc(character: ^Character, lockBodies: bool) ---
	Character_Activate                    :: proc(character: ^Character, lockBodies: bool) ---
	Character_PostSimulation              :: proc(character: ^Character, maxSeparationDistance: f32, lockBodies: bool) ---
	Character_SetLinearAndAngularVelocity :: proc(character: ^Character, linearVelocity: ^Vec3, angularVelocity: ^Vec3, lockBodies: bool) ---
	Character_GetLinearVelocity           :: proc(character: ^Character, result: ^Vec3) ---
	Character_SetLinearVelocity           :: proc(character: ^Character, value: ^Vec3, lockBodies: bool) ---
	Character_AddLinearVelocity           :: proc(character: ^Character, value: ^Vec3, lockBodies: bool) ---
	Character_AddImpulse                  :: proc(character: ^Character, value: ^Vec3, lockBodies: bool) ---
	Character_GetBodyID                   :: proc(character: ^Character) -> BodyID ---
	Character_GetPositionAndRotation      :: proc(character: ^Character, position: ^RVec3, rotation: ^Quat, lockBodies: bool) ---
	Character_SetPositionAndRotation      :: proc(character: ^Character, position: ^RVec3, rotation: ^Quat, activationMode: Activation, lockBodies: bool) ---
	Character_GetPosition                 :: proc(character: ^Character, position: ^RVec3, lockBodies: bool) ---
	Character_SetPosition                 :: proc(character: ^Character, position: ^RVec3, activationMode: Activation, lockBodies: bool) ---
	Character_GetRotation                 :: proc(character: ^Character, rotation: ^Quat, lockBodies: bool) ---
	Character_SetRotation                 :: proc(character: ^Character, rotation: ^Quat, activationMode: Activation, lockBodies: bool) ---
	Character_GetCenterOfMassPosition     :: proc(character: ^Character, result: ^RVec3, lockBodies: bool) ---
	Character_GetWorldTransform           :: proc(character: ^Character, result: ^RMatrix4x4, lockBodies: bool) ---
	Character_GetLayer                    :: proc(character: ^Character) -> ObjectLayer ---
	Character_SetLayer                    :: proc(character: ^Character, value: ObjectLayer, lockBodies: bool) ---
	Character_SetShape                    :: proc(character: ^Character, shape: ^Shape, maxPenetrationDepth: f32, lockBodies: bool) ---

	/* CharacterVirtualSettings */
	CharacterVirtualSettings_Init :: proc(settings: ^CharacterVirtualSettings) ---

	/* CharacterVirtual */
	CharacterVirtual_Create                             :: proc(settings: ^CharacterVirtualSettings, position: ^RVec3, rotation: ^Quat, userData: u64, system: ^PhysicsSystem) -> ^CharacterVirtual ---
	CharacterVirtual_GetID                              :: proc(character: ^CharacterVirtual) -> CharacterID ---
	CharacterVirtual_SetListener                        :: proc(character: ^CharacterVirtual, listener: ^CharacterContactListener) ---
	CharacterVirtual_SetCharacterVsCharacterCollision   :: proc(character: ^CharacterVirtual, characterVsCharacterCollision: ^CharacterVsCharacterCollision) ---
	CharacterVirtual_GetLinearVelocity                  :: proc(character: ^CharacterVirtual, velocity: ^Vec3) ---
	CharacterVirtual_SetLinearVelocity                  :: proc(character: ^CharacterVirtual, velocity: ^Vec3) ---
	CharacterVirtual_GetPosition                        :: proc(character: ^CharacterVirtual, position: ^RVec3) ---
	CharacterVirtual_SetPosition                        :: proc(character: ^CharacterVirtual, position: ^RVec3) ---
	CharacterVirtual_GetRotation                        :: proc(character: ^CharacterVirtual, rotation: ^Quat) ---
	CharacterVirtual_SetRotation                        :: proc(character: ^CharacterVirtual, rotation: ^Quat) ---
	CharacterVirtual_GetWorldTransform                  :: proc(character: ^CharacterVirtual, result: ^RMatrix4x4) ---
	CharacterVirtual_GetCenterOfMassTransform           :: proc(character: ^CharacterVirtual, result: ^RMatrix4x4) ---
	CharacterVirtual_GetMass                            :: proc(character: ^CharacterVirtual) -> f32 ---
	CharacterVirtual_SetMass                            :: proc(character: ^CharacterVirtual, value: f32) ---
	CharacterVirtual_GetMaxStrength                     :: proc(character: ^CharacterVirtual) -> f32 ---
	CharacterVirtual_SetMaxStrength                     :: proc(character: ^CharacterVirtual, value: f32) ---
	CharacterVirtual_GetPenetrationRecoverySpeed        :: proc(character: ^CharacterVirtual) -> f32 ---
	CharacterVirtual_SetPenetrationRecoverySpeed        :: proc(character: ^CharacterVirtual, value: f32) ---
	CharacterVirtual_GetEnhancedInternalEdgeRemoval     :: proc(character: ^CharacterVirtual) -> bool ---
	CharacterVirtual_SetEnhancedInternalEdgeRemoval     :: proc(character: ^CharacterVirtual, value: bool) ---
	CharacterVirtual_GetCharacterPadding                :: proc(character: ^CharacterVirtual) -> f32 ---
	CharacterVirtual_GetMaxNumHits                      :: proc(character: ^CharacterVirtual) -> u32 ---
	CharacterVirtual_SetMaxNumHits                      :: proc(character: ^CharacterVirtual, value: u32) ---
	CharacterVirtual_GetHitReductionCosMaxAngle         :: proc(character: ^CharacterVirtual) -> f32 ---
	CharacterVirtual_SetHitReductionCosMaxAngle         :: proc(character: ^CharacterVirtual, value: f32) ---
	CharacterVirtual_GetMaxHitsExceeded                 :: proc(character: ^CharacterVirtual) -> bool ---
	CharacterVirtual_GetShapeOffset                     :: proc(character: ^CharacterVirtual, result: ^Vec3) ---
	CharacterVirtual_SetShapeOffset                     :: proc(character: ^CharacterVirtual, value: ^Vec3) ---
	CharacterVirtual_GetUserData                        :: proc(character: ^CharacterVirtual) -> u64 ---
	CharacterVirtual_SetUserData                        :: proc(character: ^CharacterVirtual, value: u64) ---
	CharacterVirtual_GetInnerBodyID                     :: proc(character: ^CharacterVirtual) -> BodyID ---
	CharacterVirtual_CancelVelocityTowardsSteepSlopes   :: proc(character: ^CharacterVirtual, desiredVelocity: ^Vec3, velocity: ^Vec3) ---
	CharacterVirtual_StartTrackingContactChanges        :: proc(character: ^CharacterVirtual) ---
	CharacterVirtual_FinishTrackingContactChanges       :: proc(character: ^CharacterVirtual) ---
	CharacterVirtual_Update                             :: proc(character: ^CharacterVirtual, deltaTime: f32, layer: ObjectLayer, system: ^PhysicsSystem, bodyFilter: ^BodyFilter, shapeFilter: ^ShapeFilter) ---
	CharacterVirtual_ExtendedUpdate                     :: proc(character: ^CharacterVirtual, deltaTime: f32, settings: ^ExtendedUpdateSettings, layer: ObjectLayer, system: ^PhysicsSystem, bodyFilter: ^BodyFilter, shapeFilter: ^ShapeFilter) ---
	CharacterVirtual_RefreshContacts                    :: proc(character: ^CharacterVirtual, layer: ObjectLayer, system: ^PhysicsSystem, bodyFilter: ^BodyFilter, shapeFilter: ^ShapeFilter) ---
	CharacterVirtual_CanWalkStairs                      :: proc(character: ^CharacterVirtual, linearVelocity: ^Vec3) -> bool ---
	CharacterVirtual_WalkStairs                         :: proc(character: ^CharacterVirtual, deltaTime: f32, stepUp: ^Vec3, stepForward: ^Vec3, stepForwardTest: ^Vec3, stepDownExtra: ^Vec3, layer: ObjectLayer, system: ^PhysicsSystem, bodyFilter: ^BodyFilter, shapeFilter: ^ShapeFilter) -> bool ---
	CharacterVirtual_StickToFloor                       :: proc(character: ^CharacterVirtual, stepDown: ^Vec3, layer: ObjectLayer, system: ^PhysicsSystem, bodyFilter: ^BodyFilter, shapeFilter: ^ShapeFilter) -> bool ---
	CharacterVirtual_UpdateGroundVelocity               :: proc(character: ^CharacterVirtual) ---
	CharacterVirtual_SetShape                           :: proc(character: ^CharacterVirtual, shape: ^Shape, maxPenetrationDepth: f32, layer: ObjectLayer, system: ^PhysicsSystem, bodyFilter: ^BodyFilter, shapeFilter: ^ShapeFilter) -> bool ---
	CharacterVirtual_SetInnerBodyShape                  :: proc(character: ^CharacterVirtual, shape: ^Shape) ---
	CharacterVirtual_GetNumActiveContacts               :: proc(character: ^CharacterVirtual) -> u32 ---
	CharacterVirtual_GetActiveContact                   :: proc(character: ^CharacterVirtual, index: u32, result: ^CharacterVirtualContact)	 ---
	CharacterVirtual_HasCollidedWithBody                :: proc(character: ^CharacterVirtual, body: BodyID) -> bool ---
	CharacterVirtual_HasCollidedWith                    :: proc(character: ^CharacterVirtual, other: CharacterID) -> bool ---
	CharacterVirtual_HasCollidedWithCharacter           :: proc(character: ^CharacterVirtual, other: ^CharacterVirtual) -> bool ---
	CharacterContactListener_SetProcs                   :: proc(procs: ^CharacterContactListener_Procs) ---
	CharacterContactListener_Create                     :: proc(userData: rawptr) -> ^CharacterContactListener ---
	CharacterContactListener_Destroy                    :: proc(listener: ^CharacterContactListener) ---
	CharacterVsCharacterCollision_SetProcs              :: proc(procs: ^CharacterVsCharacterCollision_Procs) ---
	CharacterVsCharacterCollision_Create                :: proc(userData: rawptr) -> ^CharacterVsCharacterCollision ---
	CharacterVsCharacterCollision_CreateSimple          :: proc() -> ^CharacterVsCharacterCollision ---
	CharacterVsCharacterCollisionSimple_AddCharacter    :: proc(characterVsCharacter: ^CharacterVsCharacterCollision, character: ^CharacterVirtual) ---
	CharacterVsCharacterCollisionSimple_RemoveCharacter :: proc(characterVsCharacter: ^CharacterVsCharacterCollision, character: ^CharacterVirtual) ---
	CharacterVsCharacterCollision_Destroy               :: proc(listener: ^CharacterVsCharacterCollision) ---

	/* CollisionDispatch */
	CollisionDispatch_CollideShapeVsShape        :: proc(shape1: ^Shape, shape2: ^Shape, scale1: ^Vec3, scale2: ^Vec3, centerOfMassTransform1: ^Matrix4x4, centerOfMassTransform2: ^Matrix4x4, collideShapeSettings: ^CollideShapeSettings, callback: CollideShapeCollectorCallback, userData: rawptr, shapeFilter: ^ShapeFilter) -> bool ---
	CollisionDispatch_CastShapeVsShapeLocalSpace :: proc(direction: ^Vec3, shape1: ^Shape, shape2: ^Shape, scale1InShape2LocalSpace: ^Vec3, scale2: ^Vec3, centerOfMassTransform1InShape2LocalSpace: ^Matrix4x4, centerOfMassWorldTransform2: ^Matrix4x4, shapeCastSettings: ^ShapeCastSettings, callback: CastShapeCollectorCallback, userData: rawptr, shapeFilter: ^ShapeFilter) -> bool ---
	CollisionDispatch_CastShapeVsShapeWorldSpace :: proc(direction: ^Vec3, shape1: ^Shape, shape2: ^Shape, scale1: ^Vec3, inScale2: ^Vec3, centerOfMassWorldTransform1: ^Matrix4x4, centerOfMassWorldTransform2: ^Matrix4x4, shapeCastSettings: ^ShapeCastSettings, callback: CastShapeCollectorCallback, userData: rawptr, shapeFilter: ^ShapeFilter) -> bool ---
	DebugRenderer_SetProcs                       :: proc(procs: ^DebugRenderer_Procs) ---
	DebugRenderer_Create                         :: proc(userData: rawptr) -> ^DebugRenderer ---
	DebugRenderer_Destroy                        :: proc(renderer: ^DebugRenderer) ---
	DebugRenderer_NextFrame                      :: proc(renderer: ^DebugRenderer) ---
	DebugRenderer_SetCameraPos                   :: proc(renderer: ^DebugRenderer, position: ^RVec3) ---
	DebugRenderer_DrawLine                       :: proc(renderer: ^DebugRenderer, from: ^RVec3, to: ^RVec3, color: Color) ---
	DebugRenderer_DrawWireBox                    :: proc(renderer: ^DebugRenderer, box: ^AABox, color: Color) ---
	DebugRenderer_DrawWireBox2                   :: proc(renderer: ^DebugRenderer, _matrix: ^RMatrix4x4, box: ^AABox, color: Color) ---
	DebugRenderer_DrawMarker                     :: proc(renderer: ^DebugRenderer, position: ^RVec3, color: Color, size: f32) ---
	DebugRenderer_DrawArrow                      :: proc(renderer: ^DebugRenderer, from: ^RVec3, to: ^RVec3, color: Color, size: f32) ---
	DebugRenderer_DrawCoordinateSystem           :: proc(renderer: ^DebugRenderer, _matrix: ^RMatrix4x4, size: f32) ---
	DebugRenderer_DrawPlane                      :: proc(renderer: ^DebugRenderer, point: ^RVec3, normal: ^Vec3, color: Color, size: f32) ---
	DebugRenderer_DrawWireTriangle               :: proc(renderer: ^DebugRenderer, v1: ^RVec3, v2: ^RVec3, v3: ^RVec3, color: Color) ---
	DebugRenderer_DrawWireSphere                 :: proc(renderer: ^DebugRenderer, center: ^RVec3, radius: f32, color: Color, level: c.int) ---
	DebugRenderer_DrawWireUnitSphere             :: proc(renderer: ^DebugRenderer, _matrix: ^RMatrix4x4, color: Color, level: c.int) ---
	DebugRenderer_DrawTriangle                   :: proc(renderer: ^DebugRenderer, v1: ^RVec3, v2: ^RVec3, v3: ^RVec3, color: Color, castShadow: DebugRenderer_CastShadow) ---
	DebugRenderer_DrawBox                        :: proc(renderer: ^DebugRenderer, box: ^AABox, color: Color, castShadow: DebugRenderer_CastShadow, drawMode: DebugRenderer_DrawMode) ---
	DebugRenderer_DrawBox2                       :: proc(renderer: ^DebugRenderer, _matrix: ^RMatrix4x4, box: ^AABox, color: Color, castShadow: DebugRenderer_CastShadow, drawMode: DebugRenderer_DrawMode) ---
	DebugRenderer_DrawSphere                     :: proc(renderer: ^DebugRenderer, center: ^RVec3, radius: f32, color: Color, castShadow: DebugRenderer_CastShadow, drawMode: DebugRenderer_DrawMode) ---
	DebugRenderer_DrawUnitSphere                 :: proc(renderer: ^DebugRenderer, _matrix: RMatrix4x4, color: Color, castShadow: DebugRenderer_CastShadow, drawMode: DebugRenderer_DrawMode) ---
	DebugRenderer_DrawCapsule                    :: proc(renderer: ^DebugRenderer, _matrix: ^RMatrix4x4, halfHeightOfCylinder: f32, radius: f32, color: Color, castShadow: DebugRenderer_CastShadow, drawMode: DebugRenderer_DrawMode) ---
	DebugRenderer_DrawCylinder                   :: proc(renderer: ^DebugRenderer, _matrix: ^RMatrix4x4, halfHeight: f32, radius: f32, color: Color, castShadow: DebugRenderer_CastShadow, drawMode: DebugRenderer_DrawMode) ---
	DebugRenderer_DrawOpenCone                   :: proc(renderer: ^DebugRenderer, top: ^RVec3, axis: ^Vec3, perpendicular: ^Vec3, halfAngle: f32, length: f32, color: Color, castShadow: DebugRenderer_CastShadow, drawMode: DebugRenderer_DrawMode) ---
	DebugRenderer_DrawSwingConeLimits            :: proc(renderer: ^DebugRenderer, _matrix: ^RMatrix4x4, swingYHalfAngle: f32, swingZHalfAngle: f32, edgeLength: f32, color: Color, castShadow: DebugRenderer_CastShadow, drawMode: DebugRenderer_DrawMode) ---
	DebugRenderer_DrawSwingPyramidLimits         :: proc(renderer: ^DebugRenderer, _matrix: ^RMatrix4x4, minSwingYAngle: f32, maxSwingYAngle: f32, minSwingZAngle: f32, maxSwingZAngle: f32, edgeLength: f32, color: Color, castShadow: DebugRenderer_CastShadow, drawMode: DebugRenderer_DrawMode) ---
	DebugRenderer_DrawPie                        :: proc(renderer: ^DebugRenderer, center: ^RVec3, radius: f32, normal: ^Vec3, axis: ^Vec3, minAngle: f32, maxAngle: f32, color: Color, castShadow: DebugRenderer_CastShadow, drawMode: DebugRenderer_DrawMode) ---
	DebugRenderer_DrawTaperedCylinder            :: proc(renderer: ^DebugRenderer, inMatrix: ^RMatrix4x4, top: f32, bottom: f32, topRadius: f32, bottomRadius: f32, color: Color, castShadow: DebugRenderer_CastShadow, drawMode: DebugRenderer_DrawMode) ---
	Skeleton_Create                              :: proc() -> ^Skeleton ---
	Skeleton_Destroy                             :: proc(skeleton: ^Skeleton) ---
	Skeleton_AddJoint                            :: proc(skeleton: ^Skeleton, name: cstring) -> u32 ---
	Skeleton_AddJoint2                           :: proc(skeleton: ^Skeleton, name: cstring, parentIndex: c.int) -> u32 ---
	Skeleton_AddJoint3                           :: proc(skeleton: ^Skeleton, name: cstring, parentName: cstring) -> u32 ---
	Skeleton_GetJointCount                       :: proc(skeleton: ^Skeleton) -> c.int ---
	Skeleton_GetJoint                            :: proc(skeleton: ^Skeleton, index: c.int, joint: ^SkeletonJoint) ---
	Skeleton_GetJointIndex                       :: proc(skeleton: ^Skeleton, name: cstring) -> c.int ---
	Skeleton_CalculateParentJointIndices         :: proc(skeleton: ^Skeleton) ---
	Skeleton_AreJointsCorrectlyOrdered           :: proc(skeleton: ^Skeleton) -> bool ---

	/* Ragdoll */
	RagdollSettings_Create                                :: proc() -> ^RagdollSettings ---
	RagdollSettings_Destroy                               :: proc(settings: ^RagdollSettings) ---
	RagdollSettings_GetSkeleton                           :: proc(character: ^RagdollSettings) -> ^Skeleton ---
	RagdollSettings_SetSkeleton                           :: proc(character: ^RagdollSettings, skeleton: ^Skeleton) ---
	RagdollSettings_Stabilize                             :: proc(settings: ^RagdollSettings) -> bool ---
	RagdollSettings_DisableParentChildCollisions          :: proc(settings: ^RagdollSettings, jointMatrices: ^Matrix4x4, minSeparationDistance: f32) ---
	RagdollSettings_CalculateBodyIndexToConstraintIndex   :: proc(settings: ^RagdollSettings) ---
	RagdollSettings_GetConstraintIndexForBodyIndex        :: proc(settings: ^RagdollSettings, bodyIndex: c.int) -> c.int ---
	RagdollSettings_CalculateConstraintIndexToBodyIdxPair :: proc(settings: ^RagdollSettings) ---
	RagdollSettings_CreateRagdoll                         :: proc(settings: ^RagdollSettings, system: ^PhysicsSystem, collisionGroup: CollisionGroupID, userData: u64) -> ^Ragdoll ---
	Ragdoll_Destroy                                       :: proc(ragdoll: ^Ragdoll) ---
	Ragdoll_AddToPhysicsSystem                            :: proc(ragdoll: ^Ragdoll, activationMode: Activation, lockBodies: bool) ---
	Ragdoll_RemoveFromPhysicsSystem                       :: proc(ragdoll: ^Ragdoll, lockBodies: bool) ---
	Ragdoll_Activate                                      :: proc(ragdoll: ^Ragdoll, lockBodies: bool) ---
	Ragdoll_IsActive                                      :: proc(ragdoll: ^Ragdoll, lockBodies: bool) -> bool ---
	Ragdoll_ResetWarmStart                                :: proc(ragdoll: ^Ragdoll) ---

	/* EstimateCollisionResponse */
	EstimateCollisionResponse                     :: proc(body1: ^Body, body2: ^Body, manifold: ^ContactManifold, combinedFriction: f32, combinedRestitution: f32, minVelocityForRestitution: f32, numIterations: u32, result: ^CollisionEstimationResult) ---
	WheelSettings_Init                            :: proc(settings: ^WheelSettings) ---
	WheelSettingsWV_Init                          :: proc(settings: ^WheelSettingsWV) ---
	VehicleConstraintSettings_Init                :: proc(settings: ^VehicleConstraintSettings) ---
	VehicleConstraint_Create                      :: proc(body: ^Body, settings: ^VehicleConstraintSettings) -> ^VehicleConstraint ---
	VehicleConstraint_Destroy                     :: proc(constraint: ^VehicleConstraint) ---
	VehicleConstraint_AsPhysicsStepListener       :: proc(constraint: ^VehicleConstraint) -> ^PhysicsStepListener ---
	VehicleConstraint_GetWheeledVehicleController :: proc(constraint: ^VehicleConstraint) -> ^WheeledVehicleController ---
	VehicleConstraint_SetVehicleCollisionTester   :: proc(constraint: ^VehicleConstraint, tester: ^VehicleCollisionTester) ---

	/* Wheel */
	Wheel_Create          :: proc(settings: ^WheelSettings) -> ^Wheel ---
	Wheel_Destroy         :: proc(wheel: ^Wheel) ---
	Wheel_HasContact      :: proc(wheel: ^Wheel) -> bool ---
	Wheel_HasHitHardPoint :: proc(wheel: ^Wheel) -> bool ---

	/* WheelWV */
	WheelWV_Create :: proc(settings: ^WheelSettingsWV) -> ^WheelWV ---

	/* VehicleEngine */
	VehicleEngineSettings_Init :: proc(settings: ^VehicleEngineSettings) ---

	/* VehicleTransmission */
	VehicleTransmissionSettings_Create  :: proc(mode: TransmissionMode, switchTime: f32, clutchReleaseTime: f32, switchLatency: f32, shiftUpRPM: f32, shiftDownRPM: f32, clutchStrength: f32) -> ^VehicleTransmissionSettings ---
	VehicleTransmissionSettings_Destroy :: proc(settings: ^VehicleTransmissionSettings) ---

	/* VehicleColliionTester */
	VehicleCollisionTesterRay_Create           :: proc(layer: ObjectLayer, up: ^Vec3, maxSlopeAngle: f32) -> ^VehicleCollisionTesterRay ---
	VehicleCollisionTesterRay_Destroy          :: proc(tester: ^VehicleCollisionTesterRay) ---
	VehicleCollisionTesterCastSphere_Create    :: proc(layer: ObjectLayer, radius: f32, up: ^Vec3, maxSlopeAngle: f32) -> ^VehicleCollisionTesterCastSphere ---
	VehicleCollisionTesterCastSphere_Destroy   :: proc(tester: ^VehicleCollisionTesterCastSphere) ---
	VehicleCollisionTesterCastCylinder_Create  :: proc(layer: ObjectLayer, convexRadiusFraction: f32) -> ^VehicleCollisionTesterCastCylinder ---
	VehicleCollisionTesterCastCylinder_Destroy :: proc(tester: ^VehicleCollisionTesterCastCylinder) ---

	/* WheeledVehicleController */
	WheeledVehicleControllerSettings_Create    :: proc(engine: ^VehicleEngineSettings, transmission: ^VehicleTransmissionSettings, differentialLimitedSlipRatio: f32) -> ^WheeledVehicleControllerSettings ---
	WheeledVehicleControllerSettings_Destroy   :: proc(settings: ^WheeledVehicleControllerSettings) ---
	WheeledVehicleController_GetConstraint     :: proc(vehicle: ^WheeledVehicleController) -> ^Constraint ---
	WheeledVehicleController_SetForwardInput   :: proc(vehicle: ^WheeledVehicleController, forward: f32) ---
	WheeledVehicleController_GetForwardInput   :: proc(vehicle: ^WheeledVehicleController) -> f32 ---
	WheeledVehicleController_SetRightInput     :: proc(vehicle: ^WheeledVehicleController, rightRatio: f32) ---
	WheeledVehicleController_GetRightInput     :: proc(vehicle: ^WheeledVehicleController) -> f32 ---
	WheeledVehicleController_SetBrakeInput     :: proc(vehicle: ^WheeledVehicleController, brakeInput: f32) ---
	WheeledVehicleController_GetBrakeInput     :: proc(vehicle: ^WheeledVehicleController) -> f32 ---
	WheeledVehicleController_SetHandBrakeInput :: proc(vehicle: ^WheeledVehicleController, handBrakeInput: f32) ---
	WheeledVehicleController_GetHandBrakeInput :: proc(vehicle: ^WheeledVehicleController) -> f32 ---
}
