package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * Turret -- an Axon MAX servo (servo mode) through a 1:3.2 overdrive into a 2.75:1 turret.
 * 350 deg x 3.2 / 2.75 = ~407 deg at the servo's full signal; the Control Hub's default
 * signal reaches ~90% of that, so the full 0..1 position range is about 367 deg of turret.
 */
public class Turret implements Subsystem {

    private static final double DEGREES_PER_FULL_RANGE = 367.0;  // turret degrees across position 0..1
    private static final double CENTER     = 0.5;    // servo position that points the turret forward
    private static final double MIN_POS    = 0.15;   // software limits (about +/-128 deg) so the wires can't wrap
    private static final double MAX_POS    = 0.85;
    private static final double NUDGE_STEP = 0.002;  // position change per loop while a D-pad button is held

    private final Servo servo;
    private double position = CENTER;

    public Turret(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "turret");
        servo.setPosition(position);   // the turret moves to center as soon as INIT runs -- hands clear!
    }

    /** Swing a little to the left (counter-clockwise). Call every loop while D-pad left is held. */
    public void nudgeLeft() {
        setPosition(position + NUDGE_STEP);
    }

    /** Swing a little to the right (clockwise). Call every loop while D-pad right is held. */
    public void nudgeRight() {
        setPosition(position - NUDGE_STEP);
    }

    /** Point at an angle: 0 = forward, + = left. Auto-aim (Limelight or odometry) will call this. */
    public void setAngle(double degrees) {
        setPosition(CENTER + degrees / DEGREES_PER_FULL_RANGE);
    }

    /** The angle we're aiming at, in degrees (0 = forward, + = left). */
    public double getAngle() {
        return (position - CENTER) * DEGREES_PER_FULL_RANGE;
    }

    private void setPosition(double p) {
        position = Math.max(MIN_POS, Math.min(MAX_POS, p));   // never past the safe limits
    }

    @Override
    public void update() {
        servo.setPosition(position);
    }
}