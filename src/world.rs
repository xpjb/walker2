use crate::{
    command::WalkerCommand,
    events::WalkerEvent,
    ground::GroundQuery,
    handles::WalkerHandle,
    rig::PartPose,
    spec::WalkerSpec,
    walker::{Fidelity, Walker},
};

/// Spawn description for `WalkerWorld::spawn`.
#[derive(Clone, Copy)]
pub struct WalkerSpawnDesc {
    pub spec: WalkerSpec,
    pub x: f32,
    pub z: f32,
    pub yaw: f32,
    /// Uniform world size multiplier (1.0 = spec baseline); Froude-scales
    /// speed and cadence automatically.
    pub scale: f32,
    pub fidelity: Fidelity,
}

impl WalkerSpawnDesc {
    pub fn new(spec: WalkerSpec, x: f32, z: f32, yaw: f32) -> Self {
        Self { spec, x, z, yaw, scale: 1.0, fidelity: Fidelity::Full }
    }

    pub fn with_scale(mut self, scale: f32) -> Self {
        self.scale = scale;
        self
    }

    pub fn with_fidelity(mut self, fidelity: Fidelity) -> Self {
        self.fidelity = fidelity;
        self
    }
}

struct Slot {
    walker: Walker,
    command: WalkerCommand,
}

pub struct StepCtx<'a, G: GroundQuery> {
    pub ground: &'a G,
    pub dt: f32,
}

/// Slot/generation container stepping many walkers with per-walker
/// commands and a drained event stream.
#[derive(Default)]
pub struct WalkerWorld {
    slots: Vec<Option<Slot>>,
    generations: Vec<u32>,
    free: Vec<u32>,
    events: Vec<WalkerEvent>,
}

impl WalkerWorld {
    pub fn spawn<G: GroundQuery>(&mut self, ground: &G, desc: WalkerSpawnDesc) -> WalkerHandle {
        let mut walker = Walker::new_at(desc.spec, ground, desc.x, desc.z, desc.yaw, desc.scale);
        walker.set_fidelity(desc.fidelity);
        let slot = Slot { walker, command: WalkerCommand::IDLE };
        if let Some(index) = self.free.pop() {
            let generation = self.generations[index as usize].wrapping_add(1).max(1);
            self.generations[index as usize] = generation;
            self.slots[index as usize] = Some(slot);
            return WalkerHandle { index, generation };
        }
        let index = self.slots.len() as u32;
        self.slots.push(Some(slot));
        self.generations.push(1);
        WalkerHandle { index, generation: 1 }
    }

    pub fn despawn(&mut self, handle: WalkerHandle) -> bool {
        if !self.contains(handle) {
            return false;
        }
        self.slots[handle.index as usize] = None;
        self.free.push(handle.index);
        true
    }

    pub fn contains(&self, handle: WalkerHandle) -> bool {
        self.slots
            .get(handle.index as usize)
            .is_some_and(Option::is_some)
            && self.generations.get(handle.index as usize).copied() == Some(handle.generation)
    }

    pub fn walker(&self, handle: WalkerHandle) -> Option<&Walker> {
        if self.generations.get(handle.index as usize).copied() != Some(handle.generation) {
            return None;
        }
        self.slots
            .get(handle.index as usize)
            .and_then(Option::as_ref)
            .map(|slot| &slot.walker)
    }

    pub fn walker_mut(&mut self, handle: WalkerHandle) -> Option<&mut Walker> {
        if self.generations.get(handle.index as usize).copied() != Some(handle.generation) {
            return None;
        }
        self.slots
            .get_mut(handle.index as usize)
            .and_then(Option::as_mut)
            .map(|slot| &mut slot.walker)
    }

    pub fn set_command(&mut self, handle: WalkerHandle, command: WalkerCommand) -> bool {
        if self.generations.get(handle.index as usize).copied() != Some(handle.generation) {
            return false;
        }
        let Some(slot) = self
            .slots
            .get_mut(handle.index as usize)
            .and_then(Option::as_mut)
        else {
            return false;
        };
        slot.command = command;
        true
    }

    pub fn step<G: GroundQuery>(&mut self, ctx: StepCtx<G>) {
        for (index, slot) in self
            .slots
            .iter_mut()
            .enumerate()
            .filter_map(|(index, slot)| slot.as_mut().map(|slot| (index, slot)))
        {
            slot.walker.step(ctx.ground, slot.command, ctx.dt);
            let handle = WalkerHandle {
                index: index as u32,
                generation: self.generations[index],
            };
            for footfall in slot.walker.take_footfalls() {
                self.events.push(WalkerEvent::Footfall {
                    walker: handle,
                    pos: footfall.pos.to_array(),
                    strength: footfall.strength,
                });
            }
            self.events.push(WalkerEvent::ActuatorLoad {
                walker: handle,
                load: slot.walker.hydraulic_load(),
                motion: slot.walker.hydraulic_motion(),
            });
            self.events.push(WalkerEvent::PowerDemand {
                walker: handle,
                value: slot.walker.power_demand(),
            });
        }
    }

    pub fn drain_events(&mut self, out: &mut Vec<WalkerEvent>) {
        out.append(&mut self.events);
    }

    pub fn snapshot(&self, handle: WalkerHandle, out: &mut Vec<PartPose>) -> bool {
        let Some(walker) = self.walker(handle) else {
            return false;
        };
        walker.part_poses(out);
        true
    }
}
