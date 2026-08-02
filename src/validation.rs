/// Rolling gait self-check. Counts anomalies (same-foot double steps,
/// double-swing frames, overlean, reach violations) and tracks quality
/// stats, emitting a compact report string every 2 seconds of sim time.
/// Cheap enough to leave on; poll with `Walker::take_gait_report`.
#[derive(Default)]
pub struct GaitValidation {
    report_timer: f32,
    pub step_count: u32,
    pub same_foot_steps: u32,
    pub double_swing_frames: u32,
    pub overlean_frames: u32,
    pub reach_violations: u32,
    last_step_leg: Option<usize>,
    max_pitch: f32,
    max_roll: f32,
    max_capture_error: f32,
    max_balance_error: f32,
    max_speed: f32,
    max_vertical_speed: f32,
    max_cabin_jerk: f32,
    max_leg_use: f32,
    tilt_sum_sq: f32,
    tilt_samples: u32,
    speed_sum: f32,
    speed_samples: u32,
    last_step_len: f32,
    pub(crate) pending_report: Option<String>,
    // Cumulative counters that survive window resets — for tests/examples.
    pub total_steps: u32,
    pub total_same_foot: u32,
    pub total_double_swing_frames: u32,
    pub total_reach_violations: u32,
}

impl GaitValidation {
    pub(crate) fn record_step(&mut self, leg: usize, step_len: f32) {
        if self.last_step_leg == Some(leg) {
            self.same_foot_steps += 1;
            self.total_same_foot += 1;
        }
        self.last_step_leg = Some(leg);
        self.step_count += 1;
        self.total_steps += 1;
        self.last_step_len = step_len;
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn update_window(
        &mut self,
        dt: f32,
        pitch: f32,
        roll: f32,
        capture_error: f32,
        balance_error: f32,
        speed: f32,
        vertical_speed: f32,
        cabin_jerk: f32,
        leg_use: f32,
        swing_count: usize,
    ) {
        if swing_count > 1 {
            self.double_swing_frames += 1;
            self.total_double_swing_frames += 1;
        }
        if pitch.abs() > 0.20 {
            self.overlean_frames += 1;
        }
        self.max_capture_error = self.max_capture_error.max(capture_error);
        self.max_balance_error = self.max_balance_error.max(balance_error);
        self.max_leg_use = self.max_leg_use.max(leg_use);
        self.max_pitch = self.max_pitch.max(pitch.abs());
        self.max_roll = self.max_roll.max(roll.abs());
        self.tilt_sum_sq += pitch * pitch + roll * roll;
        self.tilt_samples += 1;
        self.max_speed = self.max_speed.max(speed);
        self.max_vertical_speed = self.max_vertical_speed.max(vertical_speed.abs());
        self.max_cabin_jerk = self.max_cabin_jerk.max(cabin_jerk);
        self.speed_sum += speed;
        self.speed_samples += 1;
        self.report_timer += dt;
        if self.report_timer >= 2.0 {
            let mut report = None;
            if self.step_count > 0
                || self.same_foot_steps > 0
                || self.double_swing_frames > 0
                || self.overlean_frames > 0
                || self.reach_violations > 0
            {
                let avg_speed = if self.speed_samples > 0 {
                    self.speed_sum / self.speed_samples as f32
                } else {
                    0.0
                };
                let tilt_rms = if self.tilt_samples > 0 {
                    (self.tilt_sum_sq / self.tilt_samples as f32).sqrt()
                } else {
                    0.0
                };
                report = Some(format!(
                    "gait: steps={} same_foot={} double_swing_frames={} overlean_frames={} reach={} last_step={:.2} avg_speed={:.2} max_speed={:.2} max_y_speed={:.2} max_jerk={:.2} max_pitch={:.2} max_roll={:.2} tilt_rms={:.3} max_capture={:.2} max_balance={:.2} max_leg_use={:.2}",
                    self.step_count,
                    self.same_foot_steps,
                    self.double_swing_frames,
                    self.overlean_frames,
                    self.reach_violations,
                    self.last_step_len,
                    avg_speed,
                    self.max_speed,
                    self.max_vertical_speed,
                    self.max_cabin_jerk,
                    self.max_pitch,
                    self.max_roll,
                    tilt_rms,
                    self.max_capture_error,
                    self.max_balance_error,
                    self.max_leg_use,
                ));
            }
            let last = self.last_step_leg;
            *self = Self {
                last_step_leg: last,
                pending_report: report,
                total_steps: self.total_steps,
                total_same_foot: self.total_same_foot,
                total_double_swing_frames: self.total_double_swing_frames,
                total_reach_violations: self.total_reach_violations,
                ..Default::default()
            };
        }
    }
}
