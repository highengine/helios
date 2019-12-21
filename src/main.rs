extern crate three;

use three::{Object};
use three::light::Light;

extern crate nalgebra as na;

use na::{Point3, Vector3, RealField};
use ncollide3d::shape::{ConvexHull, Cuboid, ShapeHandle};
use nphysics3d::force_generator::DefaultForceGeneratorSet;
use nphysics3d::joint::DefaultJointConstraintSet;
use nphysics3d::object::{BodyPartHandle, ColliderDesc, DefaultBodySet, DefaultColliderSet, Ground, RigidBodyDesc, BodySet, ColliderHandle};
use nphysics3d::world::{DefaultGeometricalWorld, DefaultMechanicalWorld};

use rand::distributions::{Distribution, Standard};
use rand::{rngs::StdRng, SeedableRng};

pub fn init_world(world: &mut three::Window,
                  camera: &mut three::camera::Camera) {
    /*
     * World
     */
    let mechanical_world: DefaultMechanicalWorld<f32> = DefaultMechanicalWorld::new(Vector3::new(0.0, -9.81, 0.0));
    let geometrical_world: nphysics3d::world::DefaultGeometricalWorld<f32> = DefaultGeometricalWorld::new();
    let mut bodies: DefaultBodySet<f32> = DefaultBodySet::new();
    let mut colliders: DefaultColliderSet<f32> = DefaultColliderSet::new();
    let joint_constraints: DefaultJointConstraintSet<f32> = DefaultJointConstraintSet::new();
    let force_generators: DefaultForceGeneratorSet<f32> = DefaultForceGeneratorSet::new();

    /*
     * Ground
     */
    let ground_thickness = 0.2;
    let ground_shape = ShapeHandle::new(Cuboid::new(Vector3::new(3.0, ground_thickness, 3.0)));

    let ground_handle = bodies.insert(Ground::new());
    let co = ColliderDesc::new(ground_shape)
        .translation(Vector3::y() * -ground_thickness)
        .build(BodyPartHandle(ground_handle, 0));
    colliders.insert(co);

}

fn main() {
    let mut win = three::Window::new("@solmann's SimEngine");
    let cam = win.factory.perspective_camera(45.0, 1.0 .. 150.0);
    cam.look_at([-4.0, 25.0, 10.0], [5.0, 0.0, 1.0], None);

    let mut cam_x: f32 = 0.;
    let mut cam_y: f32 = 0.;
    let mut cam_z: f32 = 0.;


    while win.update() && !win.input.hit(three::KEY_ESCAPE) {
        cam.look_at([cam_x.clone(), cam_y.clone(), cam_z.clone()],
                    [5.0, 0.0, 1.0], None);

        win.render(&cam);
    }
}