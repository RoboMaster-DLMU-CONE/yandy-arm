mod packet;
mod robot_view;

use kiss3d::event::{Action, Key, MouseButton, WindowEvent};
use kiss3d::light::Light;
use kiss3d::window::Canvas;
use kiss3d::window::Window;
use nalgebra as na;
use packet::{YandyControlCmd, YandyControlPack};
use robot_view::RobotView;
use std::net::UdpSocket;
use std::time::Instant;

struct FilteringCamera {
    inner: kiss3d::camera::ArcBall,
    right_mouse_down: bool,
}

impl kiss3d::camera::Camera for FilteringCamera {
    fn clip_planes(&self) -> (f32, f32) {
        self.inner.clip_planes()
    }

    fn view_transform(&self) -> na::Isometry3<f32> {
        self.inner.view_transform()
    }

    fn handle_event(&mut self, canvas: &Canvas, event: &WindowEvent) {
        // Track state
        match event {
            WindowEvent::MouseButton(MouseButton::Button2, action, _) => {
                self.right_mouse_down = *action == Action::Press;
                // Block this event from reaching ArcBall (it would trigger panning state change)
                return;
            }
            _ => {}
        }

        // If right mouse is down, we want to block CursorPos events to stop panning
        if self.right_mouse_down {
            if let WindowEvent::CursorPos(_, _, _) = event {
                return;
            }
            if let WindowEvent::Scroll(_, _, _) = event {
                return;
            }
        }

        // Also block Scroll anyway if we use it for Roll?
        // Let's assume Scroll for Roll is always active or at least we don't want Zoom conflict.
        if let WindowEvent::Scroll(_, _, _) = event {
            // Block scroll from camera
            return;
        }

        self.inner.handle_event(canvas, event);
    }

    fn eye(&self) -> na::Point3<f32> {
        self.inner.eye()
    }

    fn transformation(&self) -> na::Matrix4<f32> {
        self.inner.transformation()
    }

    fn inverse_transformation(&self) -> na::Matrix4<f32> {
        self.inner.inverse_transformation()
    }

    fn update(&mut self, canvas: &Canvas) {
        self.inner.update(canvas)
    }

    fn upload(
        &self,
        pass: usize,
        proj: &mut kiss3d::resource::ShaderUniform<na::Matrix4<f32>>,
        view: &mut kiss3d::resource::ShaderUniform<na::Matrix4<f32>>,
    ) {
        self.inner.upload(pass, proj, view)
    }
}

fn main() {
    let mut window = Window::new("Yandy Teleop Client");
    window.set_light(Light::StickToCamera);
    window.set_background_color(0.1, 0.1, 0.1);

    // Load URDF
    let relative_path = "../../../config/urdf/yandy_urdf.urdf";
    let absolute_path = std::fs::canonicalize(relative_path)
        .expect(
            "Failed to resolve URDF path - check if file exists relative to execution directory",
        )
        .to_string_lossy()
        .into_owned();

    println!("Loading URDF from: {}", absolute_path);
    // Simple load attempt
    let _robot_view = RobotView::new(&mut window, &absolute_path);

    // Target Visual (Coordinate Axes)
    let mut target_node = window.add_group();
    // X Axis (Red)
    let mut axis_x = target_node.add_cube(0.2, 0.02, 0.02);
    axis_x.set_color(1.0, 0.0, 0.0);
    axis_x.set_local_translation(na::Translation3::new(0.1, 0.0, 0.0));

    // Y Axis (Green)
    let mut axis_y = target_node.add_cube(0.02, 0.2, 0.02);
    axis_y.set_color(0.0, 1.0, 0.0);
    axis_y.set_local_translation(na::Translation3::new(0.0, 0.1, 0.0));

    // Z Axis (Blue)
    let mut axis_z = target_node.add_cube(0.02, 0.02, 0.2);
    axis_z.set_color(0.0, 0.0, 1.0);
    axis_z.set_local_translation(na::Translation3::new(0.0, 0.0, 0.1));

    // State
    let mut desired_pos = na::Point3::new(0.2, 0.0, 0.2);
    let mut desired_roll: f32 = 0.0;
    let mut desired_pitch: f32 = 0.0;
    let mut desired_yaw: f32 = 0.0;

    let mut current_pos = desired_pos;
    let mut current_roll = desired_roll;
    let mut current_pitch = desired_pitch;
    let mut current_yaw = desired_yaw;

    // Config
    let speed_pos = 1.0; // m/s
    let mouse_sens = 0.005;
    let damping = 10.0;

    // UDP
    let socket = UdpSocket::bind("0.0.0.0:0").expect("Failed to bind UDP");
    socket
        .connect("127.0.0.1:11451")
        .expect("Failed to connect UDP");
    socket.set_nonblocking(true).ok();

    let mut serializer = packet::PacketSerializer::new();

    let mut last_frame = Instant::now();
    let mut mouse_control_active = false;
    let mut cmd_state = YandyControlCmd::None;

    // Camera
    let mut camera = FilteringCamera {
        inner: kiss3d::camera::ArcBall::new(na::Point3::new(1.0, 1.0, 1.0), na::Point3::origin()),
        right_mouse_down: false,
    };

    // Set some defaults on inner camera
    camera.inner.set_dist_step(0.5);

    let mut last_cursor = window.cursor_pos();

    while window.render_with_camera(&mut camera) {
        let now = Instant::now();
        let dt = now.duration_since(last_frame).as_secs_f32();
        last_frame = now;

        let curr_cursor = window.cursor_pos();

        // Mouse Logic
        if mouse_control_active {
            if let (Some(curr), Some(last)) = (curr_cursor, last_cursor) {
                let dx = (curr.0 - last.0) as f32;
                let dy = (curr.1 - last.1) as f32;

                // X move -> Yaw
                desired_yaw -= dx * mouse_sens;
                // Y move -> Pitch
                desired_pitch -= dy * mouse_sens;
            }
        }
        last_cursor = curr_cursor;

        // Event Polling
        for event in window.events().iter() {
            match event.value {
                WindowEvent::MouseButton(btn, action, _) => {
                    if btn == MouseButton::Button2 {
                        mouse_control_active = action == Action::Press;
                    }
                }
                WindowEvent::Scroll(_, y, _) => {
                    let is_ctrl = window.get_key(Key::LControl) == Action::Press;
                    let factor = if is_ctrl { 0.1 } else { 0.5 }; // Reduced base speed, keep ctrl factor
                    desired_roll += y as f32 * 0.1 * factor; // Reduced multiplier from 0.2 to 0.1
                }
                WindowEvent::Key(key, action, _) => {
                    if action == Action::Press {
                        let is_ctrl = window.get_key(Key::LControl) == Action::Press
                            || window.get_key(Key::RControl) == Action::Press;
                        match key {
                            Key::F if is_ctrl => cmd_state = YandyControlCmd::ToggleHeld,
                            Key::E if is_ctrl => cmd_state = YandyControlCmd::IncStore,
                            Key::Q if is_ctrl => cmd_state = YandyControlCmd::DecStore,

                            Key::F => cmd_state = YandyControlCmd::SwitchFetch,
                            Key::R => cmd_state = YandyControlCmd::SwitchStore,
                            Key::G => cmd_state = YandyControlCmd::SwitchEnable,
                            Key::C => cmd_state = YandyControlCmd::SwitchGrip,
                            Key::Z => cmd_state = YandyControlCmd::Reset,
                            Key::B => cmd_state = YandyControlCmd::Error,
                            _ => {}
                        }
                    } else if action == Action::Release {
                        cmd_state = YandyControlCmd::None;
                    }
                }
                _ => {}
            }
        }

        // Keyboard Movement
        let mut move_vec = na::Vector3::zeros();
        if window.get_key(Key::W) == Action::Press {
            move_vec.x += 1.0;
        }
        if window.get_key(Key::S) == Action::Press {
            move_vec.x -= 1.0;
        }
        if window.get_key(Key::A) == Action::Press {
            move_vec.y += 1.0;
        }
        if window.get_key(Key::D) == Action::Press {
            move_vec.y -= 1.0;
        }

        if window.get_key(Key::Q) == Action::Press {
            move_vec.z += 1.0;
        }
        if window.get_key(Key::E) == Action::Press {
            move_vec.z -= 1.0;
        }
        if window.get_key(Key::Space) == Action::Press {
            move_vec.z += 1.0;
        }
        if window.get_key(Key::LShift) == Action::Press {
            move_vec.z -= 1.0;
        }

        if move_vec.norm() > 0.001 {
            let is_ctrl = window.get_key(Key::LControl) == Action::Press;
            let speed_factor = if is_ctrl { 0.1 } else { 1.0 };
            move_vec = move_vec.normalize() * speed_pos * speed_factor * dt;
            desired_pos += move_vec;
        }

        // Damping
        let alpha = (damping * dt).min(1.0);
        current_pos.coords = current_pos.coords.lerp(&desired_pos.coords, alpha);

        current_roll += (desired_roll - current_roll) * alpha;
        current_pitch += (desired_pitch - current_pitch) * alpha;
        current_yaw += (desired_yaw - current_yaw) * alpha;

        // Update Visuals
        let rotation =
            na::UnitQuaternion::from_euler_angles(current_roll, current_pitch, current_yaw);
        target_node.set_local_translation(na::Translation3::from(current_pos.coords));
        target_node.set_local_rotation(rotation);

        // Draw Text
        window.draw_text(
            &format!(
                "Pos: {:.2} {:.2} {:.2}",
                current_pos.x, current_pos.y, current_pos.z
            ),
            &na::Point2::new(10.0, 10.0),
            40.0,
            &kiss3d::text::Font::default(),
            &na::Point3::new(1.0, 1.0, 1.0),
        );
        window.draw_text(
            &format!(
                "RPY: {:.2} {:.2} {:.2}",
                current_roll, current_pitch, current_yaw
            ),
            &na::Point2::new(10.0, 50.0),
            40.0,
            &kiss3d::text::Font::default(),
            &na::Point3::new(1.0, 1.0, 1.0),
        );
        window.draw_text(
            &format!("CMD: {:?}", cmd_state),
            &na::Point2::new(10.0, 90.0),
            40.0,
            &kiss3d::text::Font::default(),
            &na::Point3::new(1.0, 1.0, 0.0),
        );

        // UDP Send
        let packet = YandyControlPack {
            x: current_pos.x,
            y: current_pos.y,
            z: current_pos.z,
            roll: current_roll,
            pitch: current_pitch,
            yaw: current_yaw,
            cmd: cmd_state.to_u8(),
        };

        let bytes = serializer.serialize(&packet);
        socket.send(&bytes).ok();
    }
}
