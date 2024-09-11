use controllers::p9n_interface;
use safe_drive::{
    context::Context, error::DynError, logger::Logger, msg::common_interfaces::geometry_msgs::msg,
    msg::common_interfaces::sensor_msgs,
};
use std::{cell::RefCell, rc::Rc};

#[allow(unused_imports)]
use safe_drive::pr_info;

use differential_two_wheel_control::{Chassis, DtwcSetting, Tire};
use motor_controller::udp_communication;
const CHASSIS: Chassis = Chassis {
    l: Tire { id: 0, raito: 1. },
    r: Tire { id: 1, raito: 1. },
};
const MAX_PAWER_INPUT: f64 = 160.;
const MAX_PAWER_OUTPUT: f64 = 1.;
const MAX_REVOLUTION: f64 = 5400.;

const OWN_PORT: &str = "50006";
const BROADCAST_ADDR: &str = "192.168.1.6:60000";

fn main() -> Result<(), DynError> {
    let mut dtwc_setting = DtwcSetting {
        chassis: CHASSIS,
        max_pawer_input: MAX_PAWER_INPUT,
        max_pawer_output: MAX_PAWER_OUTPUT,
        max_revolution: MAX_REVOLUTION,
    };

    let _logger = Logger::new("robo2_2_2_2024_a");
    let ctx = Context::new()?;
    let mut selector = ctx.create_selector()?;
    let node = ctx.create_node("robo2_2_2_2024_a", None, Default::default())?;

    let subscriber_cmd = node.create_subscriber::<msg::Twist>("cmd_vel2_2_2", None)?;
    let subscriber_joy = node.create_subscriber::<sensor_msgs::msg::Joy>("rjoy2_2_2", None)?;

    let required_max_pawer_output = Rc::new(RefCell::new(MAX_PAWER_OUTPUT / 2.));

    let required_max_pawer_output_cmd = Rc::clone(&required_max_pawer_output);
    selector.add_subscriber(
        subscriber_cmd,
        Box::new(move |msg| {
            dtwc_setting.max_pawer_output = *required_max_pawer_output_cmd.borrow();
            let motor_power = dtwc_setting.move_chassis(msg.linear.x, msg.linear.y, msg.angular.z);

            for i in motor_power.keys() {
                udp_communication::send_pwm_udp(OWN_PORT, BROADCAST_ADDR, *i, motor_power[i]);
            }
        }),
    );

    let required_max_pawer_output_joy = Rc::clone(&required_max_pawer_output);
    selector.add_subscriber(
        subscriber_joy,
        Box::new(move |msg| {
            let binding = sensor_msgs::msg::Joy::new().unwrap();
            let mut joy_c = p9n_interface::DualShock4Interface::new(&binding);
            joy_c.set_joy_msg(&msg);

            if joy_c.pressed_r2() {
                *required_max_pawer_output_joy.borrow_mut() = MAX_PAWER_OUTPUT;
            } else {
                *required_max_pawer_output_joy.borrow_mut() = MAX_PAWER_OUTPUT / 2.;
            }

            if joy_c.pressed_cross() {
                udp_communication::send_pwm_udp(OWN_PORT, BROADCAST_ADDR, 2, 1.);
            } else if joy_c.pressed_circle() {
                udp_communication::send_pwm_udp(OWN_PORT, BROADCAST_ADDR, 2, -1.);
            } else {
                udp_communication::send_pwm_udp(OWN_PORT, BROADCAST_ADDR, 2, 0.);
            }
        }),
    );

    loop {
        selector.wait()?;
    }
}
