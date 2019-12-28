extern crate three;

use three::light::Light;
use three::Object;

extern crate nalgebra as na;

use na::{Point, Point3, RealField, Vector3};
use ncollide3d::shape::{ConvexHull, Cuboid, Shape, ShapeHandle};
use ncollide3d::transformation::{ToTriMesh};
use nphysics3d::force_generator::DefaultForceGeneratorSet;
use nphysics3d::joint::DefaultJointConstraintSet;
use nphysics3d::object::{
    BodyPartHandle, BodySet, ColliderDesc, ColliderHandle, DefaultBodySet, DefaultColliderHandle,
    DefaultColliderSet, Ground, RigidBodyDesc,
};
use nphysics3d::world::{DefaultGeometricalWorld, DefaultMechanicalWorld};

use rand::distributions::{Distribution, Standard};
use rand::{rngs::StdRng, SeedableRng};
use three::camera::Camera;

struct SimulatorWorld {
    mechanical_world: DefaultMechanicalWorld<f32>,
    geometrical_world: DefaultGeometricalWorld<f32>,
    bodies: DefaultBodySet<f32>,
    colliders: DefaultColliderSet<f32>,
    joint_constraints: DefaultJointConstraintSet<f32>,
    force_generators: DefaultForceGeneratorSet<f32>,
}

fn physics_tick(world: &mut SimulatorWorld) {}

fn init_world(world: &mut SimulatorWorld) {
    /*
     * Ground.
     */
    let ground_thickness = 0.2;
    let ground_shape = ShapeHandle::new(Cuboid::new(Vector3::new(3.0, ground_thickness, 3.0)));

    let ground_handle = world.bodies.insert(Ground::new());
    let co = ColliderDesc::new(ground_shape)
        .translation(Vector3::y() * -ground_thickness)
        .build(BodyPartHandle(ground_handle, 0));

    world.colliders.insert(co);

    /*
     * Create the boxes.
     */
    let num = 6;
    let rad = 0.1;

    let cuboid = ShapeHandle::new(Cuboid::new(Vector3::repeat(rad)));

    let shift = (rad + ColliderDesc::<f32>::default_margin()) * 2.0;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift / 2.0;
    let centerz = shift * (num as f32) / 2.0;
    let height = 3.0;

    for i in 0usize..num {
        for j in 0usize..num {
            for k in 0usize..num {
                let x = i as f32 * shift - centerx;
                let y = j as f32 * shift + centery + height;
                let z = k as f32 * shift - centerz;

                // Build the rigid body.
                let rb = RigidBodyDesc::new()
                    .translation(Vector3::new(x, y, z))
                    .build();
                let rb_handle = world.bodies.insert(rb);

                // Build the collider.
                let co = ColliderDesc::new(cuboid.clone())
                    .density(1.0)
                    .build(BodyPartHandle(rb_handle, 0));
                world.colliders.insert(co);
            }
        }
    }
}

fn main() {
    // nphysics' world init

    let mut world = SimulatorWorld {
        mechanical_world: DefaultMechanicalWorld::new(Vector3::new(0.0, -9.81, 0.0)), // Remake G-Force impl!
        geometrical_world: DefaultGeometricalWorld::new(),
        bodies: DefaultBodySet::new(),
        colliders: DefaultColliderSet::new(),
        joint_constraints: DefaultJointConstraintSet::new(),
        force_generators: DefaultForceGeneratorSet::new(),
    };

    world
        .geometrical_world
        .maintain(&mut world.bodies, &mut world.colliders); // TODO: Move to SimulatorWorld impl

    world.mechanical_world.maintain(
        &mut world.geometrical_world,
        &mut world.bodies,
        &mut world.colliders,
        &mut world.joint_constraints,
    );

    // world.mechanical_world.counters.enable();

    let mut win = three::Window::new("Helios Engine");
    let cam = win.factory.perspective_camera(45.0, 1.0..150.0);

    /*
     * TODO: Make lighting work
     */

    // let ambient_light = win.factory.ambient_light(0xffffffff, 0.5);
    // win.scene.add(&ambient_light);

    let mut cam_x: f32 = 0.;
    let mut cam_y: f32 = 0.;
    let mut cam_z: f32 = 0.;

    let mut is_fullscreen = false;
    let mut is_camera_moved = true;

    init_world(&mut world);

    while win.update() && !win.input.hit(three::KEY_ESCAPE) {
        if is_camera_moved {
            cam.look_at(
                [cam_x.clone(), cam_y.clone(), cam_z.clone()],
                [5.0, 0.0, 1.0], //TODO: Attach camera rotation to user's mouse
                None,
            );
            is_camera_moved = false;
        }

        if win.input.hit(three::KEY_SPACE) {
            // TODO: Change button; FIXME: Fix flickering
            if is_fullscreen {
                is_fullscreen = false
            } else {
                is_fullscreen = true
            }
            println!("Fullscreen status: {}", is_fullscreen);
            win.set_fullscreen(is_fullscreen)
        }
        true.to_string().len();
        world.mechanical_world.step(
            &mut world.geometrical_world,
            &mut world.bodies,
            &mut world.colliders,
            &mut world.joint_constraints,
            &mut world.force_generators,
        );

        physics_tick(&mut world);

        for collider_o in world.colliders.iter() {
            let cs = collider_o.1.shape();
            cs.to_tr
            break;
            // println!("Collider is in render!");
        }
        break;
        win.render(&cam);
    }
    println!("Exiting now.");
    println!(
        "Amount of bodies simulated this time: {}",
        world.colliders.iter().count()
    );
}
