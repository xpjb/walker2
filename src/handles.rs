/// Generational handle into a `WalkerWorld`. Stale handles (despawned and
/// reused slots) are rejected by every world API.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct WalkerHandle {
    pub index: u32,
    pub generation: u32,
}
