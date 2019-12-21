extern crate three;

use three::light::Light;
use three::Object;

extern crate nalgebra as na;

use na::{Point3, RealField, Vector3};
use ncollide3d::shape::{ConvexHull, Cuboid, ShapeHandle};
use nphysics3d::force_generator::DefaultForceGeneratorSet;
use nphysics3d::joint::DefaultJointConstraintSet;
use nphysics3d::object::{
    BodyPartHandle, BodySet, ColliderDesc, ColliderHandle, DefaultBodySet, DefaultColliderSet,
    Ground, RigidBodyDesc,
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

fn tick(world: &SimulatorWorld) {}

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

    let mut win = three::Window::new("@solmann's SimEngine");
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

        win.render(&cam);
    }
}
