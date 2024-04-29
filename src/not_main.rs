// use std::cell::RefCell;

// use iced::alignment::Alignment;
// use iced::executor;
// use iced::mouse;
// use iced::theme::Theme;
// use iced::widget::canvas::{Cache, Geometry, LineCap, Path, Stroke, Style};
// use iced::widget::Column;
// use iced::widget::{button, canvas, column, container, slider, text, Row};
// use iced::{
//     Application, Color, Command, Element, Length, Point, Rectangle, Settings, Size, Subscription,
// };
// use rapier2d::na::OPoint;
// use rapier2d::prelude::*;

// const GROUND_WIDTH: f32 = 100.0;
// const GROUND_HEIGHT: f32 = 0.1;
// const GROUND_COLOR: Color = Color::from_rgb(0.5, 0.5, 0.5);
// const GROUND_RESTITUTION: f32 = 0.7;

// const GRAVITY: Vector<f32> = vector![0.0, -9.81];
// // const GRAVITY: Vector<f32> = vector![0.0, 0.0];

// const SCALE_FACTOR: f32 = 50.0;
// const BACKGROUND_COLOR: Color = Color::WHITE;

// const TIME_STEP: u64 = 16;

// const MOTOR_DAMPING: f32 = 10.0;

// pub fn main() -> iced::Result {
//     PhysicsSimulation::run(Settings::default())
// }

// struct Capsule {
//     body_handle: RigidBodyHandle,
//     collider_handle: ColliderHandle,
//     length: f32,
//     radius: f32,
//     angle: f32,
//     color: Color,
// }

// struct Hinge {
//     capsules: [Capsule; 2],
//     joint_handle: ImpulseJointHandle,
//     origin: Point,
//     min_angle: f32,
//     max_angle: f32,
//     motor_target_velocity: f32,
// }

// struct PhysicsSimulation {
//     hinges: Vec<Hinge>,
//     selected_hinge_index: usize,
//     rigid_body_set: RigidBodySet,
//     collider_set: ColliderSet,
//     integration_parameters: IntegrationParameters,
//     physics_pipeline: PhysicsPipeline,
//     island_manager: IslandManager,
//     broad_phase: BroadPhase,
//     narrow_phase: NarrowPhase,
//     impulse_joint_set: ImpulseJointSet,
//     multibody_joint_set: MultibodyJointSet,
//     ccd_solver: CCDSolver,
//     query_pipeline: QueryPipeline,
//     physics_hooks: (),
//     event_handler: (),
//     cache: Cache,
//     debug_text: RefCell<String>,
//     is_playing: bool,
// }

// #[derive(Debug, Clone, Copy)]
// enum Message {
//     Tick,
//     TogglePlayback,
//     ResetSimulation,
//     StepSimulation,
//     HingeSelected(usize),
//     CapsuleLengthChanged(usize, f32),
//     CapsuleRadiusChanged(usize, f32),
//     CapsuleAngleChanged(usize, f32),
//     JointMinAngleChanged(f32),
//     JointMaxAngleChanged(f32),
//     MotorTargetVelocityChanged(f32),
// }

// fn create_capsule(
//     hinge: &mut Hinge,
//     capsule_index: usize,
//     group: u32,
//     rigid_body_set: &mut RigidBodySet,
//     collider_set: &mut ColliderSet,
// ) {
//     let body = RigidBodyBuilder::dynamic()
//         .translation(vector![hinge.origin.x, hinge.origin.y])
//         .build();
//     let offset_x: f32 =
//         -hinge.capsules[capsule_index].angle.sin() * hinge.capsules[capsule_index].length / 2.0;
//     let offset_y: f32 =
//         hinge.capsules[capsule_index].angle.cos() * hinge.capsules[capsule_index].length / 2.0;
//     let pt_a: OPoint<f32, nalgebra::Const<2>> = point!(offset_x, offset_y);
//     let pt_b: OPoint<f32, nalgebra::Const<2>> = point!(-offset_x, -offset_y);
//     let collider = ColliderBuilder::new(SharedShape::capsule(
//         pt_a,
//         pt_b,
//         hinge.capsules[capsule_index].radius / 2.0,
//     ))
//     .collision_groups(InteractionGroups::new(group.into(), 0b0001.into()))
//     .build();
//     hinge.capsules[capsule_index].body_handle = rigid_body_set.insert(body);
//     hinge.capsules[capsule_index].collider_handle = collider_set.insert_with_parent(
//         collider,
//         hinge.capsules[capsule_index].body_handle,
//         rigid_body_set,
//     );
// }

// impl PhysicsSimulation {
//     fn init_world(&mut self) {
//         // Iterate over all the rigid bodies and remove them
//         let rigid_body_handles: Vec<_> = self
//             .rigid_body_set
//             .iter()
//             .map(|(handle, _)| handle)
//             .collect();
//         for rigid_body_handle in rigid_body_handles {
//             self.rigid_body_set.remove(
//                 rigid_body_handle,
//                 &mut self.island_manager,
//                 &mut self.collider_set,
//                 &mut self.impulse_joint_set,
//                 &mut self.multibody_joint_set,
//                 true,
//             );
//         }

//         // Iterate over all the joints and remove them
//         let joint_handles: Vec<_> = self
//             .impulse_joint_set
//             .iter()
//             .map(|(handle, _)| handle)
//             .collect();
//         for joint_handle in joint_handles {
//             self.impulse_joint_set.remove(joint_handle, true);
//         }

//         // Create the ground
//         let ground_collider = ColliderBuilder::cuboid(GROUND_WIDTH, GROUND_HEIGHT)
//             .restitution(GROUND_RESTITUTION)
//             .collision_groups(InteractionGroups::new(0b0001.into(), 0b1111.into()))
//             .build();
//         self.collider_set.insert(ground_collider);

//         // Create the hinges
//         for hinge in &mut self.hinges {
//             create_capsule(hinge, 0, 0b0010, &mut self.rigid_body_set, &mut self.collider_set);
//             create_capsule(hinge, 1, 0b0100, &mut self.rigid_body_set, &mut self.collider_set);

//             // Create the revolute joint
//             let revolute_joint: RevoluteJoint = RevoluteJointBuilder::new()
//                 .local_anchor1(point![0.0, 0.0])
//                 .local_anchor2(point![0.0, 0.0])
//                 .limits([hinge.min_angle, hinge.max_angle])
//                 // Factor is the damping coefficient of the motor’s spring-like equation.
//                 .motor_velocity(hinge.motor_target_velocity, MOTOR_DAMPING)
//                 .motor_max_force(f32::MAX)
//                 .motor_model(MotorModel::ForceBased)
//                 .build();
//             hinge.joint_handle = self.impulse_joint_set.insert(
//                 hinge.capsules[0].body_handle,
//                 hinge.capsules[1].body_handle,
//                 revolute_joint,
//                 true,
//             );
//         }
//     }

//     fn step(&mut self) {
//         self.physics_pipeline.step(
//             &GRAVITY,
//             &self.integration_parameters,
//             &mut self.island_manager,
//             &mut self.broad_phase,
//             &mut self.narrow_phase,
//             &mut self.rigid_body_set,
//             &mut self.collider_set,
//             &mut self.impulse_joint_set,
//             &mut self.multibody_joint_set,
//             &mut self.ccd_solver,
//             Some(&mut self.query_pipeline),
//             &self.physics_hooks,
//             &self.event_handler,
//         );
//     }
// }

// impl Application for PhysicsSimulation {
//     type Executor = executor::Default;
//     type Message = Message;
//     type Theme = Theme;
//     type Flags = ();

//     fn new(_flags: ()) -> (Self, Command<Message>) {
//         let mut app = PhysicsSimulation {
//             hinges: vec![
//                 Hinge {
//                     capsules: [
//                         Capsule {
//                             body_handle: RigidBodyHandle::invalid(),
//                             collider_handle: ColliderHandle::invalid(),
//                             length: 2.0,
//                             radius: 0.5,
//                             angle: 0.0,
//                             color: Color {
//                                 r: 0.0,
//                                 g: 0.5,
//                                 b: 0.8,
//                                 a: 0.5,
//                             },
//                         },
//                         Capsule {
//                             body_handle: RigidBodyHandle::invalid(),
//                             collider_handle: ColliderHandle::invalid(),
//                             length: 2.0,
//                             radius: 0.5,
//                             angle: 0.0,
//                             color: Color {
//                                 r: 0.8,
//                                 g: 0.5,
//                                 b: 0.0,
//                                 a: 0.5,
//                             },
//                         },
//                     ],
//                     joint_handle: ImpulseJointHandle::invalid(),
//                     origin: Point::new(0.0, 5.0),
//                     min_angle: -std::f32::consts::FRAC_PI_2,
//                     max_angle: std::f32::consts::FRAC_PI_2,
//                     motor_target_velocity: 0.0,
//                 },
//                 // Add more hinges here...
//             ],
//             selected_hinge_index: 0,
//             rigid_body_set: RigidBodySet::new(),
//             collider_set: ColliderSet::new(),
//             integration_parameters: IntegrationParameters::default(),
//             physics_pipeline: PhysicsPipeline::new(),
//             island_manager: IslandManager::new(),
//             broad_phase: BroadPhase::new(),
//             narrow_phase: NarrowPhase::new(),
//             impulse_joint_set: ImpulseJointSet::new(),
//             multibody_joint_set: MultibodyJointSet::new(),
//             ccd_solver: CCDSolver::new(),
//             query_pipeline: QueryPipeline::new(),
//             physics_hooks: (),
//             event_handler: (),
//             cache: Cache::default(),
//             debug_text: "".to_string().into(),
//             is_playing: false,
//         };

//         app.init_world();

//         (app, Command::none())
//     }

//     fn title(&self) -> String {
//         String::from("Revolute Joint Limits - Iced")
//     }

//     fn update(&mut self, message: Message) -> Command<Message> {
//         match message {
//             Message::Tick => {
//                 if self.is_playing {
//                     self.step();
//                     self.cache.clear();
//                 }
//             }
//             Message::TogglePlayback => {
//                 self.is_playing = !self.is_playing;
//             }
//             Message::ResetSimulation => {
//                 self.init_world();
//                 self.cache.clear();
//             }
//             Message::StepSimulation => {
//                 if !self.is_playing {
//                     self.step();
//                     self.cache.clear();
//                 }
//             }
//             Message::HingeSelected(index) => {
//                 self.selected_hinge_index = index;
//             }
//             Message::CapsuleLengthChanged(capsule_index, length) => {
//                 self.hinges[self.selected_hinge_index].capsules[capsule_index].length = length;
//                 self.init_world();
//                 self.cache.clear();
//             }
//             Message::CapsuleRadiusChanged(capsule_index, radius) => {
//                 self.hinges[self.selected_hinge_index].capsules[capsule_index].radius = radius;
//                 self.init_world();
//                 self.cache.clear();
//             }
//             Message::CapsuleAngleChanged(capsule_index, angle) => {
//                 self.hinges[self.selected_hinge_index].capsules[capsule_index].angle = angle;
//                 self.init_world();
//                 self.cache.clear();
//             }
//             Message::JointMinAngleChanged(min_angle) => {
//                 self.hinges[self.selected_hinge_index].min_angle = min_angle;
//                 self.init_world();
//                 self.cache.clear();
//             }
//             Message::JointMaxAngleChanged(max_angle) => {
//                 self.hinges[self.selected_hinge_index].max_angle = max_angle;
//                 self.init_world();
//                 self.cache.clear();
//             }
//             Message::MotorTargetVelocityChanged(target_velocity) => {
//                 // This updates the motor's target velocity in during the simulation
//                 // TODO: find a better way to change target velocity
//                 // This does not appear to cause a memory leak but it seems awful
//                 self.hinges[self.selected_hinge_index].motor_target_velocity = target_velocity;
//                 let joint_handle: ImpulseJointHandle =
//                     self.hinges[self.selected_hinge_index].joint_handle;
//                 let joint: ImpulseJoint =
//                     self.impulse_joint_set.remove(joint_handle, true).unwrap();

//                 let mut revolute_joint: RevoluteJoint = joint.data.as_revolute().unwrap().clone();
//                 revolute_joint.set_motor_velocity(target_velocity, MOTOR_DAMPING);

//                 let new_joint_handle: ImpulseJointHandle =
//                     self.impulse_joint_set
//                         .insert(joint.body1, joint.body2, revolute_joint, true);
//                 self.hinges[self.selected_hinge_index].joint_handle = new_joint_handle;
//             }
//         }

//         Command::none()
//     }

//     fn view(&self) -> Element<Message> {
//         let controls = view_controls(&self.hinges, self.selected_hinge_index, self.is_playing);

//         let canvas = canvas(self as &Self)
//             .width(Length::Fill)
//             .height(Length::Fill);

//         let debug_text = text(&*self.debug_text.borrow()).size(16);

//         let content = column![canvas, debug_text, controls].height(Length::Fill);

//         container(content)
//             .width(Length::Fill)
//             .height(Length::Fill)
//             .into()
//     }

//     fn subscription(&self) -> Subscription<Message> {
//         iced::time::every(std::time::Duration::from_millis(TIME_STEP)).map(|_| Message::Tick)
//     }
// }

// impl<Message> canvas::Program<Message> for PhysicsSimulation {
//     type State = ();

//     fn draw(
//         &self,
//         _state: &Self::State,
//         renderer: &iced::Renderer,
//         _theme: &Theme,
//         bounds: Rectangle,
//         _cursor: mouse::Cursor,
//     ) -> Vec<Geometry> {
//         let selected_hinge = &self.hinges[self.selected_hinge_index];
//         let joint = self
//             .impulse_joint_set
//             .get(selected_hinge.joint_handle)
//             .unwrap();
//         let rotation = joint.data.local_frame1.rotation;

//         let mut debug_text_vec: Vec<String> = Vec::new();
//         debug_text_vec.push(format!("Joint rotation: {:.2}", rotation.angle()));

//         let offset_factor = Vector::new(
//             bounds.width / 2.0,
//             bounds.height - (GROUND_HEIGHT * SCALE_FACTOR),
//         );

//         let physics_geometry: Geometry = self.cache.draw(renderer, bounds.size(), |frame| {
//             // Clear the frame with the background color
//             frame.fill_rectangle(Point::ORIGIN, bounds.size(), BACKGROUND_COLOR);

//             // Draw the hinges
//             for (i_hinge, hinge) in self.hinges.iter().enumerate() {
//                 for capsule in &hinge.capsules {
//                     let capsule_body: &RigidBody = &self.rigid_body_set[capsule.body_handle];
//                     let capsule_collider: &Collider = &self.collider_set[capsule.collider_handle];
//                     let capsule_position = capsule_body.translation();
//                     let capsule_rotation: f32 = capsule_collider.rotation().angle() + capsule.angle;
//                     // let capsule_rotation = capsule_body.rotation().angle();

//                     debug_text_vec.push(format!(
//                         "Capsule {} rotation: {:.2}",
//                         2 * i_hinge + 1,
//                         capsule_rotation
//                     ));

//                     // Scale and offset the capsule's position
//                     let scaled_capsule_position: Point = Point::new(
//                         (-capsule_position.x * SCALE_FACTOR) + offset_factor.x,
//                         offset_factor.y - (capsule_position.y * SCALE_FACTOR),
//                     );
//                     let scaled_capsule_radius: f32 = capsule.radius * SCALE_FACTOR;
//                     let scaled_capsule_half_length: f32 = capsule.length * SCALE_FACTOR / 2.0;

//                     // Draw the capsule
//                     let capsule_path = Path::line(
//                         Point::new(0.0, -scaled_capsule_half_length),
//                         Point::new(0.0, scaled_capsule_half_length),
//                     );
//                     let capsule_stroke = Stroke {
//                         style: Style::Solid(capsule.color),
//                         width: scaled_capsule_radius,
//                         line_cap: LineCap::Round,
//                         ..Stroke::default()
//                     };

//                     frame.with_save(|frame| {
//                         frame.translate(iced::Vector::new(
//                             scaled_capsule_position.x,
//                             scaled_capsule_position.y,
//                         ));
//                         frame.rotate(capsule_rotation);
//                         frame.stroke(&capsule_path, capsule_stroke)
//                     });
//                 }
//             }

//             *self.debug_text.borrow_mut() = debug_text_vec.join("\n");

//             // Draw the ground
//             let ground_rect = Rectangle::new(
//                 Point::new(0.0, bounds.height - (GROUND_HEIGHT * SCALE_FACTOR)),
//                 Size::new(bounds.width, GROUND_HEIGHT * SCALE_FACTOR),
//             );
//             frame.fill_rectangle(ground_rect.position(), ground_rect.size(), GROUND_COLOR);
//         });
//         vec![physics_geometry]
//     }
// }

// fn view_controls<'a>(
//     hinges: &[Hinge],
//     selected_hinge_index: usize,
//     is_playing: bool,
// ) -> Element<'a, Message> {
//     let mut playback_controls = Row::new().spacing(10);
//     playback_controls = playback_controls
//         .push(button(if is_playing { "Pause" } else { "Play" }).on_press(Message::TogglePlayback));
//     if !is_playing {
//         playback_controls =
//             playback_controls.push(button("Step").on_press(Message::StepSimulation));
//     }
//     playback_controls = playback_controls.push(button("Reset").on_press(Message::ResetSimulation));
//     let hinge_selector = hinges
//         .iter()
//         .enumerate()
//         .fold(Row::new(), |row, (index, _hinge)| {
//             row.push(
//                 button(text(format!("Hinge {}", index + 1)))
//                     .style(if index == selected_hinge_index {
//                         iced::theme::Button::Primary
//                     } else {
//                         iced::theme::Button::Secondary
//                     })
//                     .on_press(Message::HingeSelected(index)),
//             )
//         });

//     let selected_hinge = &hinges[selected_hinge_index];

//     let capsule_controls = selected_hinge.capsules.iter().enumerate().fold(
//         Column::new().spacing(10),
//         |column, (capsule_index, capsule)| {
//             let length_slider = slider(1.0..=10.0, capsule.length, move |value| {
//                 Message::CapsuleLengthChanged(capsule_index, value)
//             })
//             .step(0.1);

//             let radius_slider = slider(0.1..=5.0, capsule.radius, move |value| {
//                 Message::CapsuleRadiusChanged(capsule_index, value)
//             })
//             .step(0.1);

//             let angle_slider = slider(
//                 -std::f32::consts::PI..=std::f32::consts::PI,
//                 capsule.angle,
//                 move |value| Message::CapsuleAngleChanged(capsule_index, value),
//             )
//             .step(0.1);

//             column
//                 .push(text(format!("Capsule {}", capsule_index + 1)))
//                 .push(length_slider)
//                 .push(radius_slider)
//                 .push(angle_slider)
//         },
//     );

//     let joint_controls = Column::new()
//         .push(text("Joint Limits"))
//         .push(
//             slider(
//                 -std::f32::consts::PI..=std::f32::consts::PI,
//                 selected_hinge.min_angle,
//                 Message::JointMinAngleChanged,
//             )
//             .step(0.1),
//         )
//         .push(
//             slider(
//                 -std::f32::consts::PI..=std::f32::consts::PI,
//                 selected_hinge.max_angle,
//                 Message::JointMaxAngleChanged,
//             )
//             .step(0.1),
//         );

//     let motor_controls = Column::new().push(text("Motor Target Velocity")).push(
//         slider(
//             -10.0..=10.0,
//             selected_hinge.motor_target_velocity,
//             Message::MotorTargetVelocityChanged,
//         )
//         .step(0.1),
//     );

//     column![
//         playback_controls,
//         hinge_selector,
//         capsule_controls,
//         joint_controls,
//         motor_controls
//     ]
//     .padding(10)
//     .spacing(20)
//     .align_items(Alignment::Center)
//     .into()
// }