package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.Servo;
/*
 * Turret -- an Axon MAX servo (servo mode) through a 1:3.2 overdrive into a 2.75:1 turret.
 * 350 deg x 3.2 / 2.75 = ~407 deg of turret across the servo's FULL signal (500..2500 us).
 * We ask the Control Hub for that full signal, then allow +/-200 deg: more than a full circle,
 * so the turret can point anywhere, even straight behind the robot.
 */
public class Turret implements Subsystem {

    private static final double DEGREES_PER_FULL_RANGE = 407.0;  // turret degrees across position 0..1
    public  static final double MAX_ANGLE  = 90.0;  // farthest the turret may turn each way (check the wires!)
    private static final double CENTER     = 0.5;    // servo position that points the turret forward
    private static final double MIN_POS    = CENTER - MAX_ANGLE / DEGREES_PER_FULL_RANGE;
    private static final double MAX_POS    = CENTER + MAX_ANGLE / DEGREES_PER_FULL_RANGE;
    private static final double NUDGE_STEP = 0.002;  // position change per loop while a D-pad button is held

    private final ServoImplEx servo;   // null if this robot has no "turret" configured
    private double position = CENTER;

    public Turret(HardwareMap hardwareMap) {
        ServoImplEx found = null;
        try {
            found = hardwareMap.get(ServoImplEx.class, "turret");
        } catch (IllegalArgumentException e) {
            found = null;   // no "turret" in the configuration -- run without it instead of crashing
        }
        servo = found;
        if (servo != null) {
            servo.setDirection(Servo.Direction.REVERSE);
            servo.setPwmRange(new PwmControl.PwmRange(500, 2500));   // the servo's full travel, not the hub's default
            servo.setPosition(position);   // the turret moves to center as soon as INIT runs -- hands clear!
        }
    }

    /** False on a robot without a turret: everything else still works, the turret just does nothing. */
    public boolean hasTurret() {
        return servo != null;
    }

    /** Swing a little to the left (counter-clockwise). For testing by hand. */
    public void nudgeLeft() {
        setPosition(position + NUDGE_STEP);
    }

    /** Swing a little to the right (clockwise). For testing by hand. */
    public void nudgeRight() {
        setPosition(position - NUDGE_STEP);
    }

    /** Point at an angle: 0 = forward, + = left. Auto-aim calls this every loop. */
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
        if (servo != null) {
            servo.setPosition(position);
        }
    }
}