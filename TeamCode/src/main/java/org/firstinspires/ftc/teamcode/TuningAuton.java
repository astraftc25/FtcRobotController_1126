


/*
 * =================================================================================================
 * TUNING & CALIBRATION GUIDE
 * =================================================================================================
 * * 1. DRIVE DISTANCE (The "Tape Measure" Test)
 * Mecanum wheel diameter effectively changes based on "squish" into foam tiles.
 * - TEST: Run driveForward(48, 0.5, false).
 * - FIX: If the robot actually went 46.5 inches instead of 48:
 * Update TICKS_PER_INCH constant to: (560 / (Math.PI * 2.97)) * (48.0 / 46.5).
 * - CHECK: Re-run. If within 0.25", move on.
 *
 *
 * * 2. TURRET PID (The "Slap & Track" Test)
 * Stability is key for wide goals.
 * - STEP A (Kp): Set Kd to 0. Set Kp to 0.01. Increase Kp until the turret reaches the target
 * but starts to "hunt" (bounce back and forth). Then back it off by 10%.
 * - STEP B (Kd): Increase Kd in increments of 0.002.
 * THE SLAP TEST: Gently flick the turret. If it wobbles 3+ times, add Kd.
 * If it snaps back and stops instantly, it is perfect.
 * - STEP C (Deadband): If the motor "hums" when stationary, ensure a ~0.5 degree deadband
 * is active in the code.
 *
 *
 * * 3. TURNING (The "IMU Drift" Test)
 * Mecanum rollers cause over-rotation after motors stop.
 * - TEST: Run turnLeft(90, 0.4).
 * - FIX: If it stops at 92°, decrease turn power or increase the "tolerance"
 * (e.g., change 2.0 to 3.0 in the error loop).
 * - ORIENTATION: Ensure RevHubOrientationOnRobot matches your physical Hub mount.
 *
 *
 * * 4. LIMELIGHT "LOCK-ON" (Web Dashboard: http://10.TE.AM.11:5801)
 * - EXPOSURE: Turn down until the screen is black except for the target.
 * Prevents interference from stadium lights.
 * - TARGETING: If shots consistently hit the left/right of the goal, use
 * "Crosshair Offset" in the Limelight settings to re-center.
 *
 *
 * * 5. HARDWARE "HEALTH" CHECK
 * - SET SCREWS: Check turret/intake gears. If these slip, PID tuning is impossible.
 * - BATTERY: Tune with battery at 13.0V+. A 12V battery will underperform.
 * - LENS: Clean the Limelight lens with microfiber. Fingerprints = jittery tracking.
 * =================================================================================================
 */
