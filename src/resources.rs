use bevy_ecs::resource::Resource;
use crossbeam::channel::Receiver;
use rand::{rngs::StdRng, SeedableRng};
use rapier2d::{
    dynamics::{
        CCDSolver, ImpulseJointSet, IntegrationParameters, IslandManager, MultibodyJointSet,
        RigidBodySet,
    },
    geometry::{BroadPhaseMultiSap, ColliderSet, CollisionEvent, NarrowPhase},
    na::{vector, Vector2},
    pipeline::{ChannelEventCollector, PhysicsPipeline, QueryPipeline},
    prelude::nalgebra,
};

#[derive(Resource, Debug, Clone, Default, PartialEq, Eq, Hash)]
pub enum SimulationMode {
    #[default]
    Running,

    Paused,
}

#[derive(Debug, Default, Resource)]
pub struct SimulationState {
    pub mode: SimulationMode,
}

#[derive(Resource)]
pub struct NormalDistribution(pub StdRng);

impl Default for NormalDistribution {
    fn default() -> Self {
        NormalDistribution(StdRng::from_os_rng())
    }
}

#[derive(Resource)]
pub struct PhysicsEngine {
    pub collider_set: ColliderSet,
    pub rigid_body_set: RigidBodySet,
    pub gravity: Vector2<f32>,
    pub integration_parameters: IntegrationParameters,
    pub physics_pipeline: PhysicsPipeline,
    pub island_manager: IslandManager,
    pub broad_phase: BroadPhaseMultiSap,
    pub narrow_phase: NarrowPhase,
    pub impulse_joint_set: ImpulseJointSet,
    pub multibody_joint_set: MultibodyJointSet,
    pub ccd_solver: CCDSolver,
    pub query_pipeline: QueryPipeline,
    //pub physics_hooks:
    pub event_handler: ChannelEventCollector,
    pub collision_recv: Receiver<CollisionEvent>,
}

impl Default for PhysicsEngine {
    fn default() -> Self {
        let (collision_send, collision_recv) = crossbeam::channel::unbounded();
        let (contact_force_send, _contact_force_recv) = crossbeam::channel::unbounded();
        Self {
            rigid_body_set: RigidBodySet::new(),
            collider_set: ColliderSet::new(),
            gravity: vector![0.0, 1.0],
            integration_parameters: IntegrationParameters::default(),
            island_manager: IslandManager::new(),
            broad_phase: BroadPhaseMultiSap::new(),
            narrow_phase: NarrowPhase::new(),
            impulse_joint_set: ImpulseJointSet::new(),
            multibody_joint_set: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            query_pipeline: QueryPipeline::new(),
            // physics_hooks: (),
            event_handler: ChannelEventCollector::new(collision_send, contact_force_send),
            physics_pipeline: PhysicsPipeline::new(),
            collision_recv,
        }
    }
}
