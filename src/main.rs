#![warn(clippy::all, clippy::pedantic)]

mod colours;
mod component;
mod resources;
mod systems;

use crate::colours::{BALL_COLOURS, GUNMETAL};
use bevy_ecs::{
    schedule::{IntoScheduleConfigs, Schedule, SystemSet},
    world::World,
};
use component::{CircleCollider, CircleMesh, Position, Velocity};
use macroquad::{
    input::{KeyCode, is_key_released},
    miniquad::date,
    rand::{self as macroquad_rand, srand},
    window::{Conf, clear_background, next_frame},
};
use rapier2d::{na::vector, prelude::nalgebra};
use resources::{NormalDistribution, PhysicsEngine, SimulationMode, SimulationState};
use systems::{
    create_ball_physics_system, create_cuboid_colliders_system, create_cuboid_sensors_system,
    create_height_field_colliders_system, draw_balls_system, end_simulation_system,
    get_random_ball_velocity, spawn_fixed_colliders_system, spawn_ground_sensor_system,
    spawn_new_ball_system, step_physics_engine_system, update_balls_system,
};
use uom::{
    si::{f32::Length, length},
    unit,
};

#[macro_use]
extern crate uom;

const WINDOW_WIDTH: f32 = 1366.0;
const WINDOW_HEIGHT: f32 = 768.0;
const BALL_RADIUS: f32 = 0.6;

unit! {
    system: uom::si;
    quantity: uom::si::length;

    // 1 metre is 50 px
    @pixel: 0.02; "px", "pixel", "pixels";
}

fn conf() -> Conf {
    #[allow(clippy::cast_possible_truncation)]
    Conf {
        window_title: String::from("Macroquad Rapier Bubbles Bevy ECS"),
        window_width: WINDOW_WIDTH as i32,
        window_height: WINDOW_HEIGHT as i32,
        high_dpi: true,
        ..Default::default()
    }
}

#[derive(SystemSet, Debug, Hash, PartialEq, Eq, Clone)]
enum ScheduleSet {
    BeforeSimulation,
    Simulation,
    AfterSimulation,
}

#[macroquad::main(conf)]
async fn main() {
    // seed macroquad random number generator
    #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
    {
        srand(date::now().floor() as u64);
    }

    let mut world = World::new();
    world.init_resource::<PhysicsEngine>();
    world.init_resource::<NormalDistribution>();
    world.init_resource::<SimulationState>();

    // Create ball
    let window_width = Length::new::<pixel>(WINDOW_WIDTH);
    let window_height = Length::new::<pixel>(WINDOW_HEIGHT);
    let new_ball_position = vector![
        window_width / 2.0,
        Length::new::<length::meter>(2.0 * BALL_RADIUS) - window_height
    ];

    let mut normal_distribution = world
        .get_resource_mut::<NormalDistribution>()
        .expect("Expected NormalDistribution to have been initialised");
    let new_ball_velocity = get_random_ball_velocity(&mut normal_distribution.0);
    let _ball_entity = world.spawn((
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

    // Startup schedule
    Schedule::default()
        .add_systems(
            (
                (spawn_fixed_colliders_system, spawn_ground_sensor_system),
                (
                    create_cuboid_colliders_system,
                    create_height_field_colliders_system,
                    create_cuboid_sensors_system,
                ),
            )
                .chain(),
        )
        .run(&mut world);

    let mut paused_schedule = Schedule::default();
    paused_schedule.add_systems(draw_balls_system);

    let mut playing_schedule = Schedule::default();
    playing_schedule
        .configure_sets(
            (
                ScheduleSet::BeforeSimulation,
                ScheduleSet::Simulation,
                ScheduleSet::AfterSimulation,
            )
                .chain(),
        )
        .add_systems(
            (create_ball_physics_system, draw_balls_system)
                .chain()
                .in_set(ScheduleSet::BeforeSimulation),
        )
        .add_systems(step_physics_engine_system.in_set(ScheduleSet::Simulation))
        .add_systems(
            (
                update_balls_system,
                spawn_new_ball_system,
                end_simulation_system,
            )
                .in_set(ScheduleSet::AfterSimulation),
        );

    // run the game loop, stepping the simulation once per frame.
    loop {
        if is_key_released(KeyCode::Escape) {
            break;
        }

        clear_background(GUNMETAL);

        let simulation_state = world
            .get_resource::<SimulationState>()
            .expect("Expected simulation state to have been initialised");

        match &simulation_state.mode {
            SimulationMode::Running => {
                playing_schedule.run(&mut world);
            }
            SimulationMode::Paused => paused_schedule.run(&mut world),
        }

        next_frame().await;
    }
}
