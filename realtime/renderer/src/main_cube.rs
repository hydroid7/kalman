extern crate kiss3d;
extern crate nalgebra as na;

use na::{Vector3, UnitQuaternion, Translation3, Point3};
use kiss3d::window::Window;
use kiss3d::light::Light;

fn main() {
    let mut window = Window::new("Smartphone position");
    let mut c      = window.add_cube(1.0, 1.0, 1.0);

    c.set_color(1.0, 0.0, 0.0);

    window.set_light(Light::StickToCamera);

    const AXIS_LENGTH: f32 = -100000.0;
    let rot = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.01);
    let trans = Translation3::new(0.0, 0.0, 0.01);

    while window.render() {
        c.prepend_to_local_rotation(&rot);
        c.append_translation(&trans);

        window.draw_line(&Point3::new(-1.0 * AXIS_LENGTH, 0.0, 0.0), &Point3::new(AXIS_LENGTH, 0.0, 0.0), &Point3::new(0.5, 0.5, 0.5));
        window.draw_line(&Point3::new(0.0, -1.0 * AXIS_LENGTH, 0.0), &Point3::new(0.0, AXIS_LENGTH, 0.0), &Point3::new(0.5, 0.5, 0.5));
        window.draw_line(&Point3::new(0.0, 0.0, -1.0 * AXIS_LENGTH), &Point3::new(0.0, 0.0, AXIS_LENGTH), &Point3::new(0.5, 0.5, 0.5));
    }
}
