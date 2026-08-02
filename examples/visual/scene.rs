use glam::Vec3;
use walker2::{Fidelity, GroundQuery, Walker, WalkerCommand, WalkerSpec};

use crate::camera::ViewMode;
use crate::shared::scenarios::{
    self, ScenarioId, BRAKE_AT, CROWD_TIERS, DT, FIDELITIES, IMPULSE_1, IMPULSE_1_AT, IMPULSE_2,
    IMPULSE_2_AT, IMPULSE_3, IMPULSE_3_AT, ROLLING_AMPLITUDE, ROLLING_SECONDS, ROLLING_WAVELENGTH,
    SCALES, SLOPE_GRADE,
};

#[derive(Clone, Copy)]
pub enum DemoGround {
    Flat,
    Rolling { amplitude: f32, wavelength: f32 },
    Slope { grade: f32 },
}

impl GroundQuery for DemoGround {
    fn height_at(&self, x: f32, z: f32) -> f32 {
        match *self {
            Self::Flat => 0.0,
            Self::Rolling {
                amplitude,
                wavelength,
            } => {
                let w = wavelength.max(0.01);
                amplitude * ((x / w).sin() + (z / (w * 1.37)).cos())
            }
            Self::Slope { grade } => z * grade,
        }
    }
}

pub struct Actor {
    pub label: String,
    pub walker: Walker,
    pub ground: DemoGround,
    driver_index: usize,
}

#[derive(Clone, Copy)]
pub struct Flash {
    pub pos: Vec3,
    pub strength: f32,
    pub remaining: f32,
}

#[derive(Clone, Copy)]
pub struct ImpulseFlash {
    pub origin: Vec3,
    pub impulse: Vec3,
    pub remaining: f32,
}

pub struct Scene {
    pub id: ScenarioId,
    pub actors: Vec<Actor>,
    pub footfalls: Vec<Flash>,
    pub impulses: Vec<ImpulseFlash>,
    pub tick: u64,
}

impl Scene {
    pub fn new(id: ScenarioId) -> Self {
        let mut scene = Self {
            id,
            actors: Vec::new(),
            footfalls: Vec::new(),
            impulses: Vec::new(),
            tick: 0,
        };
        scene.populate();
        scene
    }

    pub fn reset(&mut self) {
        self.actors.clear();
        self.footfalls.clear();
        self.impulses.clear();
        self.tick = 0;
        self.populate();
    }

    pub fn set_scenario(&mut self, id: ScenarioId) {
        self.id = id;
        self.reset();
    }

    pub fn time(&self) -> f32 {
        self.tick as f32 * DT
    }

    pub fn duration_ticks(&self) -> u64 {
        (self.id.duration() / DT).round() as u64
    }

    pub fn finished(&self) -> bool {
        self.tick >= self.duration_ticks()
    }

    pub fn phase_label(&self) -> &'static str {
        let t = self.time();
        match self.id {
            ScenarioId::FlatWalk => {
                if t < scenarios::FLAT_PHASE_SECONDS {
                    "walk"
                } else {
                    "sprint"
                }
            }
            ScenarioId::TurnInPlace => {
                if t < scenarios::TURN_IDLE_SECONDS {
                    "settle"
                } else if t < scenarios::TURN_IDLE_SECONDS + scenarios::TURN_PHASE_SECONDS {
                    "face 150 degrees"
                } else {
                    "face 30 degrees"
                }
            }
            ScenarioId::StrafeBackpedal => {
                if t < scenarios::MOVE_PHASE_SECONDS {
                    "strafe right"
                } else if t < scenarios::MOVE_PHASE_SECONDS * 2.0 {
                    "backpedal"
                } else {
                    "forward-left diagonal"
                }
            }
            ScenarioId::StartStop => {
                if t < scenarios::START_SECONDS {
                    "settle"
                } else if t < BRAKE_AT {
                    "sprint launch and cruise"
                } else {
                    "hard brake"
                }
            }
            ScenarioId::Terrain => {
                if t < ROLLING_SECONDS {
                    "rolling hills"
                } else {
                    "14% uphill sprint"
                }
            }
            ScenarioId::ImpulseRecovery => {
                if t < IMPULSE_1_AT {
                    "settle"
                } else if t < IMPULSE_2_AT {
                    "recover lateral hit"
                } else if t < scenarios::IMPULSE_SPRINT_AT {
                    "recover diagonal hit"
                } else if t < IMPULSE_3_AT {
                    "sprint"
                } else {
                    "recover while sprinting"
                }
            }
            ScenarioId::ScaleFroude => "synchronized sprint",
            ScenarioId::FidelityTiers => {
                if t < 4.0 {
                    "walk; switching actor is Full"
                } else if t < 5.0 {
                    "sprint; switching actor is Reduced"
                } else if t < 6.0 {
                    "sprint; switching actor is Kinematic"
                } else if t < 8.0 {
                    "sprint; switching actor is Full"
                } else if t < 10.0 {
                    "turn in place"
                } else {
                    "strafe"
                }
            }
            ScenarioId::Crowd => "deterministic wandering",
        }
    }

    pub fn default_view(&self) -> ViewMode {
        match self.id {
            ScenarioId::TurnInPlace | ScenarioId::StrafeBackpedal | ScenarioId::Crowd => {
                ViewMode::Top
            }
            ScenarioId::StartStop | ScenarioId::Terrain => ViewMode::Side,
            _ => ViewMode::Iso,
        }
    }

    pub fn default_zoom(&self) -> f32 {
        match self.id {
            ScenarioId::Crowd => 4.5,
            ScenarioId::ScaleFroude => 8.0,
            ScenarioId::FlatWalk | ScenarioId::FidelityTiers => 18.0,
            _ => 28.0,
        }
    }

    pub fn step(&mut self) {
        self.apply_tick_events();
        let t = self.time();
        let id = self.id;

        for actor in &mut self.actors {
            let command = match id {
                ScenarioId::FlatWalk => scenarios::flat_walk_command(t),
                ScenarioId::TurnInPlace => scenarios::turn_command(t),
                ScenarioId::StrafeBackpedal => scenarios::movement_command(t),
                ScenarioId::StartStop => scenarios::start_stop_command(t),
                ScenarioId::Terrain => {
                    if t < ROLLING_SECONDS {
                        WalkerCommand::walk(Vec3::Z, 0.0)
                    } else {
                        WalkerCommand::sprint(Vec3::Z, 0.0)
                    }
                }
                ScenarioId::ImpulseRecovery => scenarios::impulse_command(t),
                ScenarioId::ScaleFroude => WalkerCommand::sprint(Vec3::Z, 0.0),
                ScenarioId::FidelityTiers => scenarios::fidelity_command(t),
                ScenarioId::Crowd => scenarios::crowd_command(actor.driver_index, t),
            };
            actor.walker.step(&actor.ground, command, DT);
            for event in actor.walker.take_footfalls() {
                self.footfalls.push(Flash {
                    pos: event.pos,
                    strength: event.strength,
                    remaining: 0.45,
                });
            }
        }

        self.tick += 1;
        for flash in &mut self.footfalls {
            flash.remaining -= DT;
        }
        self.footfalls.retain(|flash| flash.remaining > 0.0);
        for flash in &mut self.impulses {
            flash.remaining -= DT;
        }
        self.impulses.retain(|flash| flash.remaining > 0.0);
    }

    fn apply_tick_events(&mut self) {
        let at = |seconds: f32| (seconds / DT).round() as u64;
        match self.id {
            ScenarioId::Terrain if self.tick == at(ROLLING_SECONDS) => {
                self.actors.clear();
                self.push_actor(
                    "biped / slope",
                    WalkerSpec::biped(),
                    DemoGround::Slope { grade: SLOPE_GRADE },
                    0.0,
                    0.0,
                    1.0,
                    Fidelity::Full,
                    0,
                );
            }
            ScenarioId::ImpulseRecovery => {
                let event = if self.tick == at(IMPULSE_1_AT) {
                    Some(IMPULSE_1)
                } else if self.tick == at(IMPULSE_2_AT) {
                    Some(IMPULSE_2)
                } else if self.tick == at(IMPULSE_3_AT) {
                    Some(IMPULSE_3)
                } else {
                    None
                };
                if let Some(impulse) = event {
                    if let Some(actor) = self.actors.first_mut() {
                        let origin = actor.walker.position();
                        actor.walker.apply_impulse(impulse);
                        self.impulses.push(ImpulseFlash {
                            origin,
                            impulse,
                            remaining: 0.9,
                        });
                    }
                }
            }
            ScenarioId::FidelityTiers => {
                let fidelity = if self.tick == at(4.0) {
                    Some(Fidelity::Reduced)
                } else if self.tick == at(5.0) {
                    Some(Fidelity::Kinematic)
                } else if self.tick == at(6.0) {
                    Some(Fidelity::Full)
                } else {
                    None
                };
                if let (Some(fidelity), Some(actor)) = (fidelity, self.actors.get_mut(3)) {
                    actor.walker.set_fidelity(fidelity);
                }
            }
            _ => {}
        }
    }

    fn populate(&mut self) {
        match self.id {
            ScenarioId::FlatWalk => {
                for (index, (label, spec, x)) in [
                    ("biped", WalkerSpec::biped(), -8.0),
                    ("longstrider", WalkerSpec::longstrider(), 0.0),
                    ("humanoid", WalkerSpec::humanoid(), 8.0),
                ]
                .into_iter()
                .enumerate()
                {
                    self.push_actor(
                        label,
                        spec,
                        DemoGround::Flat,
                        x,
                        0.0,
                        1.0,
                        Fidelity::Full,
                        index,
                    );
                }
            }
            ScenarioId::TurnInPlace
            | ScenarioId::StrafeBackpedal
            | ScenarioId::StartStop
            | ScenarioId::ImpulseRecovery => {
                self.push_actor(
                    "biped",
                    WalkerSpec::biped(),
                    DemoGround::Flat,
                    0.0,
                    0.0,
                    1.0,
                    Fidelity::Full,
                    0,
                );
            }
            ScenarioId::Terrain => {
                self.push_actor(
                    "biped / rolling",
                    WalkerSpec::biped(),
                    DemoGround::Rolling {
                        amplitude: ROLLING_AMPLITUDE,
                        wavelength: ROLLING_WAVELENGTH,
                    },
                    0.0,
                    0.0,
                    1.0,
                    Fidelity::Full,
                    0,
                );
            }
            ScenarioId::ScaleFroude => {
                for (index, (&scale, x)) in SCALES.iter().zip([-9.0, 0.0, 14.0]).enumerate() {
                    self.push_actor(
                        &format!("scale {scale:.2}"),
                        WalkerSpec::biped(),
                        DemoGround::Flat,
                        x,
                        0.0,
                        scale,
                        Fidelity::Full,
                        index,
                    );
                }
            }
            ScenarioId::FidelityTiers => {
                for (index, ((label, fidelity), x)) in
                    FIDELITIES.into_iter().zip([-10.0, -3.5, 3.5]).enumerate()
                {
                    self.push_actor(
                        label,
                        WalkerSpec::biped(),
                        DemoGround::Flat,
                        x,
                        0.0,
                        1.0,
                        fidelity,
                        index,
                    );
                }
                self.push_actor(
                    "Switching",
                    WalkerSpec::biped(),
                    DemoGround::Flat,
                    10.0,
                    0.0,
                    1.0,
                    Fidelity::Full,
                    3,
                );
            }
            ScenarioId::Crowd => {
                let mut driver_index = 0;
                let mut x_origin = -46.0;
                for (label, fidelity, count) in CROWD_TIERS {
                    for i in 0..count {
                        let x = x_origin + (i % 10) as f32 * 4.2;
                        let z = (i / 10) as f32 * 4.2 - 18.0;
                        self.push_actor(
                            label,
                            WalkerSpec::biped(),
                            DemoGround::Rolling {
                                amplitude: 0.5,
                                wavelength: 11.0,
                            },
                            x,
                            z,
                            1.0,
                            fidelity,
                            driver_index,
                        );
                        driver_index += 1;
                    }
                    x_origin += 48.0;
                }
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn push_actor(
        &mut self,
        label: &str,
        spec: WalkerSpec,
        ground: DemoGround,
        x: f32,
        z: f32,
        scale: f32,
        fidelity: Fidelity,
        driver_index: usize,
    ) {
        let mut walker = Walker::new_at(spec, &ground, x, z, 0.0, scale);
        walker.set_fidelity(fidelity);
        self.actors.push(Actor {
            label: label.to_owned(),
            walker,
            ground,
            driver_index,
        });
    }
}
