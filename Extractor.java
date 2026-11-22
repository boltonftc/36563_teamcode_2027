package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * Extractor -- the pollen extractor arm. A positional servo with just TWO places to be:
 *   STOWED   = up, tucked in (where it starts every match)
 *   DEPLOYED = lowered in front of the robot
 * The driver decides when: D-pad down deploys, D-pad up stows.
 */
public class Extractor implements Subsystem {

    /** The two places the arm can be. public so TeleOp can show it on telemetry. */
    public enum State {
        STOWED,     // up, tucked in -- where it starts
        DEPLOYED    // lowered in front of the robot
    }

    // Servo numbers (0.0 .. 1.0) for each state. FIND YOURS with Extractor Calibrate!
    private static final double STOWED_POS   = 0;
    private static final double DEPLOYED_POS = 0.6;

    private final Servo servo;                 // null if this robot has no "extractor" configured
    private State state = State.STOWED;        // where we WANT the arm to be

    public Extractor(HardwareMap hardwareMap) {
        Servo found = null;
        try {
            found = hardwareMap.get(Servo.class, "extractor");
        } catch (IllegalArgumentException e) {
            found = null;   // no "extractor" in the configuration -- run without it
        }
        servo = found;
        if (servo != null) {
            servo.setPosition(STOWED_POS);   // moves as soon as INIT runs -- hands clear!
        }
    }

    /** Lower the arm in front of the robot. */
    public void deploy() {
        state = State.DEPLOYED;
    }

    /** Raise the arm back up, out of the way. */
    public void stow() {
        state = State.STOWED;
    }

    /** Where we're sending the arm right now. */
    public State getState() {
        return state;
    }

    public boolean isDeployed() {
        return state == State.DEPLOYED;
    }

    @Override
    public void update() {
        if (servo == null) {
            return;                              // no extractor on this robot
        }
        switch (state) {
            case STOWED:
                servo.setPosition(STOWED_POS);
                break;
            case DEPLOYED:
                servo.setPosition(DEPLOYED_POS);
                break;
        }
    }
}