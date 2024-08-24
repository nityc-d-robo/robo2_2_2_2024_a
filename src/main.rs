use safe_drive::{
    context::Context, error::DynError, logger::Logger, msg::common_interfaces::geometry_msgs::msg,
    msg::common_interfaces::sensor_msgs,
};

use differential_two_wheel_control::DtwcSetting;
use motor_controller::udp_communication;

const ROBOT_CENTER_TO_WHEEL_DISTANCE: f64 = 0.37;

fn main() -> Result<(), DynError> {
    let dtwc_setting = DtwcSetting {
        l_id: 0,
        r_id: 1,
        robot_center_to_wheel_distance: ROBOT_CENTER_TO_WHEEL_DISTANCE,
    };

    // for debug
    let _logger = Logger::new("robo2_2_2_2024_a");
    let ctx = Context::new()?;
    let mut selector = ctx.create_selector()?;
    let node = ctx.create_node("robo2_2_2_2024_a", None, Default::default())?;

    let subscriber_cmd = node.create_subscriber::<msg::Twist>("cmd_vel2_2_2", None)?;
    let subscriber_joy = node.create_subscriber::<sensor_msgs::msg::Joy>("rjoy2_2_2", None)?;

    selector.add_subscriber(
        subscriber_cmd,
        Box::new(move |msg| {
            let motor_power = dtwc_setting.move_chassis(msg.linear.x, msg.linear.y, msg.angular.z);

            for i in motor_power.keys() {
                udp_communication::send_pwm_udp("50006", "192.168.4:60000", *i, motor_power[i]);
            }
        }),
    );

    selector.add_subscriber(
        subscriber_joy,
        Box::new(move |_msg| {
            todo!();
        }),
    );

    loop {
        selector.wait()?;
    }
}
