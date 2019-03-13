extern crate nalgebra as na;
extern crate kiss3d;

use std::net::UdpSocket;
use std::str;
use serde::{Serialize, Deserialize};
use serde_json::{Value};
use std::sync::mpsc::{Sender, channel, Receiver};
use std::{thread, time};
use na::{Vector3, UnitQuaternion, Translation3, Point3};
use kiss3d::window::Window;
use kiss3d::light::Light;

#[derive(Serialize, Deserialize, Debug)]
struct Point {
    x: f32,
    y: f32,
    z: f32
}
#[derive(Serialize, Deserialize, Debug)]
struct SolidState {
    timestamp: f32,
    position: Point
}

impl SolidState {
    fn to_translation(&self) -> Translation3<f32> {
        Translation3::new(self.position.x, self.position.y, self.position.z)
    }
}


fn main() {

    let mut current_state: SolidState;

    let mut window = Window::new("Smartphone position");
    let mut c      = window.add_cube(1.0, 1.0, 1.0);
    c.set_color(1.0, 0.0, 0.0);

    window.set_light(Light::StickToCamera);

    const AXIS_LENGTH: f32 = -100000.0;
    let rot = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.01);
    let trans = Translation3::new(0.0, 0.0, 0.01);

    let (sender, receiver) = channel();
    let socket_receiver = thread::spawn(move || {
        connect_socket(sender);
    });

    while window.render() {
        let message = receiver.try_recv();
        match message {
            Ok(message) => {
                c.set_local_translation(message.to_translation());
                println!("{:?}", message)
            },
            Err(reason) => println!("{:?}", reason)
        }

        // c.prepend_to_local_rotation(&rot);
        // c.append_translation(&trans);

        window.draw_line(&Point3::new(-1.0 * AXIS_LENGTH, 0.0, 0.0), &Point3::new(AXIS_LENGTH, 0.0, 0.0), &Point3::new(0.5, 0.5, 0.5));
        window.draw_line(&Point3::new(0.0, -1.0 * AXIS_LENGTH, 0.0), &Point3::new(0.0, AXIS_LENGTH, 0.0), &Point3::new(0.5, 0.5, 0.5));
        window.draw_line(&Point3::new(0.0, 0.0, -1.0 * AXIS_LENGTH), &Point3::new(0.0, 0.0, AXIS_LENGTH), &Point3::new(0.5, 0.5, 0.5));
    }
    socket_receiver.join();
}

fn setup() {

}


fn connect_socket(sender: Sender<SolidState>) {
    let socket = UdpSocket::bind("127.0.0.1:2000").expect("couldn't bind to address");
    let mut buf = [0; 1000];
    loop {
        let r = socket.recv_from(&mut buf);
        match r {
            Ok(bytes) => { sender.send(serde_json::from_slice(&mut buf[..bytes.0]).unwrap()); }
            Err(reason) => { println!("{:?}", reason) }
        }
        // let filled_buf = &mut buf[..number_of_bytes];
        // let v: SolidState = serde_json::from_slice(filled_buf).unwrap();
        // sender.send(v).unwrap();
        // println!("{:}", v["timestamp"]);
        // println!("Position {:} {:} {:}", v["position"]["x"], v["position"]["y"], v["position"]["z"]);
    };
}
