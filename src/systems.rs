use crate::component::{CircleCollider, CircleMesh, CuboidCollider, Position, Sensor, Velocity};
use crate::resources::{NormalDistribution, PhysicsEngine, SimulationMode, SimulationState};
use crate::{BALL_RADIUS, WINDOW_HEIGHT, WINDOW_WIDTH, pixel};
use crate::{colours::BALL_COLOURS, component::HeightFieldCollider};
use bevy_ecs::{
    query::{With, Without},
    system::{Commands, Query, Res, ResMut},
};
use macroquad::{
    rand::{self as macroquad_rand},
    shapes::draw_circle,
};
use rand::{Rng, rngs::StdRng};
use rand_distr::StandardNormal;
use rapier2d::{
    dynamics::RigidBodyBuilder,
    geometry::{ColliderBuilder, CollisionEvent, CollisionEventFlags},
    math::Isometry,
    na::{DVector, Isometry2, Vector2, vector},
    pipeline::ActiveEvents,
    prelude::nalgebra,
};
use uom::si::{
    f32::{Length, Velocity as VelocityUnit},
    length, velocity,
};

pub fn get_random_ball_velocity(normal_distribution: &mut StdRng) -> Vector2<VelocityUnit> {
    // Standard generates values in the [0,1) range
    let pseudo_random_value: f32 = normal_distribution.sample(StandardNormal);
    let x_velocity: VelocityUnit =
        VelocityUnit::new::<velocity::meter_per_second>((2.0 * pseudo_random_value) - 1.0);
    let y_velocity: VelocityUnit = VelocityUnit::new::<velocity::meter_per_second>(1.0);
    vector![x_velocity, y_velocity]
}

pub fn create_ball_physics_system(
    mut query: Query<(&mut Position, &mut Velocity, &mut CircleCollider)>,
    physics_engine: ResMut<PhysicsEngine>,
) {
    let PhysicsEngine {
        collider_set,
        rigid_body_set,
        ..
    } = physics_engine.into_inner();
    for (position, velocity, mut circle_collider) in &mut query {
        if circle_collider.physics_handle.is_none() {
            let rigid_body = RigidBodyBuilder::dynamic()
                .translation(vector![position.0.x.value, position.0.y.value])
                .linvel(vector![velocity.0.x.value, velocity.0.y.value])
                .build();
            let collider = ColliderBuilder::ball(circle_collider.radius.value)
                .restitution(0.0)
                .density(0.001)
                .active_events(ActiveEvents::COLLISION_EVENTS)
                .build();
            let ball_body_handle = rigid_body_set.insert(rigid_body);
            collider_set.insert_with_parent(collider, ball_body_handle, rigid_body_set);
            circle_collider.physics_handle = Some(ball_body_handle);
        }
    }
}

#[allow(clippy::needless_pass_by_value)]
pub fn create_cuboid_colliders_system(
    query: Query<&CuboidCollider, Without<Sensor>>,
    mut physics_engine: ResMut<PhysicsEngine>,
) {
    for collider in query.iter() {
        let new_collider =
            ColliderBuilder::cuboid(collider.half_extents.x, collider.half_extents.y)
                .position(collider.position)
                .build();
        physics_engine.collider_set.insert(new_collider);
    }
}

#[allow(clippy::needless_pass_by_value)]
pub fn create_height_field_colliders_system(
    query: Query<&HeightFieldCollider>,
    mut physics_engine: ResMut<PhysicsEngine>,
) {
    for collider in query.iter() {
        let new_collider = ColliderBuilder::heightfield(collider.heights.clone(), collider.scale)
            .translation(collider.translation)
            .friction(1.0)
            .restitution(0.0)
            .build();
        physics_engine.collider_set.insert(new_collider);
    }
}

#[allow(clippy::needless_pass_by_value)]
pub fn create_cuboid_sensors_system(
    query: Query<&CuboidCollider, With<Sensor>>,
    mut physics_engine: ResMut<PhysicsEngine>,
) {
    for collider in query.iter() {
        let new_collider =
            ColliderBuilder::cuboid(collider.half_extents.x, collider.half_extents.y)
                .position(collider.position)
                .translation(collider.translation)
                .sensor(true)
                .build();
        physics_engine.collider_set.insert(new_collider);
    }
}

#[allow(clippy::needless_pass_by_value)]
pub fn draw_balls_system(query: Query<(&Position, &CircleMesh)>) {
    for (position, mesh) in query.iter() {
        draw_circle(
            position.0.x.get::<pixel>(),
            -position.0.y.get::<pixel>(),
            mesh.radius.get::<pixel>(),
            mesh.colour,
        );
    }
}

#[allow(clippy::needless_pass_by_value)]
pub fn update_balls_system(
    mut query: Query<(&mut Position, &mut Velocity, &CircleCollider)>,
    physics_engine: Res<PhysicsEngine>,
) {
    for (mut position, mut velocity, circle_collider) in &mut query {
        if let Some(handle) = circle_collider.physics_handle {
            let ball_body = &physics_engine.rigid_body_set[handle];
            position.0 = vector![
                Length::new::<length::meter>(ball_body.translation().x),
                Length::new::<length::meter>(ball_body.translation().y)
            ];
            velocity.0 = vector![
                VelocityUnit::new::<velocity::meter_per_second>(ball_body.linvel().x),
                VelocityUnit::new::<velocity::meter_per_second>(ball_body.linvel().y)
            ];
        }
    }
}

pub fn spawn_fixed_colliders_system(mut commands: Commands) {
    let window_width = Length::new::<pixel>(WINDOW_WIDTH);
    let collider_half_thickness = Length::new::<length::meter>(0.05);
    let max_balls: u32 = get_max_balls();
    let nsubdivs: usize = (max_balls * 2)
        .try_into()
        .expect("Expected fewer subdivisions");
    let ceiling_width: Length;
    #[allow(clippy::cast_precision_loss)]
    {
        ceiling_width = Length::new::<length::meter>(max_balls as f32 * BALL_RADIUS * 2.0);
    }
    let _ceiling_entity = commands.spawn(HeightFieldCollider {
        heights: DVector::from_fn(nsubdivs + 1, |i, _| if i % 2 == 0 { -1.2 } else { 0.0 }),
        scale: vector![ceiling_width.value, collider_half_thickness.value],
        translation: vector![
            0.5 * window_width.get::<length::meter>(),
            -collider_half_thickness.value
        ],
    });

    let window_height = Length::new::<pixel>(WINDOW_HEIGHT);
    let window_width = Length::new::<pixel>(WINDOW_WIDTH);
    let gap: Length = 0.5 * (window_width - ceiling_width);
    let _left_wall = commands.spawn(CuboidCollider {
        half_extents: vector![
            0.5 * window_height.get::<length::meter>(),
            collider_half_thickness.value
        ],
        position: Isometry::new(
            vector![
                (gap - collider_half_thickness).get::<length::meter>(),
                (window_height / -2.0).get::<length::meter>()
            ],
            std::f32::consts::FRAC_PI_2,
        ),
        translation: vector![0.0, 0.0],
    });

    let _right_wall = commands.spawn(CuboidCollider {
        half_extents: vector![
            0.5 * window_height.get::<length::meter>(),
            collider_half_thickness.value
        ],
        position: Isometry::new(
            vector![
                (window_width + collider_half_thickness - gap).get::<length::meter>(),
                (window_height / -2.0).get::<length::meter>()
            ],
            std::f32::consts::FRAC_PI_2,
        ),
        translation: vector![0.0, 0.0],
    });
}

pub fn spawn_ground_sensor_system(mut commands: Commands) {
    let collider_half_thickness = Length::new::<length::meter>(0.05);
    let window_width = Length::new::<pixel>(WINDOW_WIDTH);
    let window_height = Length::new::<pixel>(WINDOW_HEIGHT);
    commands.spawn((
        CuboidCollider {
            half_extents: vector![
                0.5 * window_width.get::<length::meter>(),
                collider_half_thickness.value
            ],
            translation: vector![
                0.0,
                (-window_height - collider_half_thickness).get::<length::meter>()
            ],
            position: Isometry2::identity(),
        },
        Sensor,
    ));
}

#[allow(clippy::needless_pass_by_value)]
pub fn spawn_new_ball_system(
    mut commands: Commands,
    physics_engine: ResMut<PhysicsEngine>,
    mut normal_distribution: ResMut<NormalDistribution>,
) {
    if physics_engine
        .island_manager
        .active_dynamic_bodies()
        .is_empty()
    {
        let window_width = Length::new::<pixel>(WINDOW_WIDTH);
        let window_height = Length::new::<pixel>(WINDOW_HEIGHT);
        let new_ball_position = vector![
            window_width / 2.0,
            Length::new::<length::meter>(2.0 * BALL_RADIUS) - window_height
        ];
        let new_ball_velocity = get_random_ball_velocity(&mut normal_distribution.0);
        let _ball_entity = commands.spawn((
            Position(new_ball_position),
            CircleMesh {
                colour: BALL_COLOURS[macroquad_rand::gen_range(0, BALL_COLOURS.len())],
                radius: Length::new::<length::meter>(BALL_RADIUS),
            },
            CircleCollider {
                radius: Length::new::<length::meter>(BALL_RADIUS),
                physics_handle: None,
            },
            Velocity(new_ball_velocity),
        ));
    }
}

#[allow(clippy::needless_pass_by_value)]
pub fn end_simulation_system(
    physics_engine: ResMut<PhysicsEngine>,
    mut simulation_state: ResMut<SimulationState>,
) {
    while let Ok(collision_event) = physics_engine.collision_recv.try_recv() {
        if let CollisionEvent::Started(
            _collider_handle_1,
            _collider_handle_2,
            CollisionEventFlags::SENSOR,
        ) = collision_event
        {
            simulation_state.mode = SimulationMode::Paused;
        }
    }
}

pub fn step_physics_engine_system(physics_engine: ResMut<PhysicsEngine>) {
    let PhysicsEngine {
        gravity,
        integration_parameters,
        physics_pipeline,
        island_manager,
        broad_phase,
        narrow_phase,
        rigid_body_set,
        collider_set,
        impulse_joint_set,
        multibody_joint_set,
        ccd_solver,
        query_pipeline,
        event_handler,
        ..
    } = physics_engine.into_inner();
    let physics_hooks = ();
    physics_pipeline.step(
        gravity,
        integration_parameters,
        island_manager,
        broad_phase,
        narrow_phase,
        rigid_body_set,
        collider_set,
        impulse_joint_set,
        multibody_joint_set,
        ccd_solver,
        Some(query_pipeline),
        &physics_hooks,
        event_handler,
    );
}

/// Maximum number of ball which can fit across the window
#[allow(
    clippy::cast_precision_loss,
    clippy::cast_sign_loss,
    clippy::cast_possible_truncation
)]
fn get_max_balls() -> u32 {
    let window_width = Length::new::<pixel>(WINDOW_WIDTH);
    let ball_radius = Length::new::<length::meter>(BALL_RADIUS);

    (window_width / (2.0 * ball_radius)).value.floor() as u32
}
