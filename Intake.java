package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
 * ============================================================================
 *  Intake  --  the mechanism that pulls game pieces in (and spits them out)
 * ============================================================================
 *
 *  The intake is a single motor. At any moment it is doing exactly ONE of
 *  three things: running FORWARD to collect, sitting OFF, or running in
 *  REVERSE to spit a stuck piece back out. Three neat choices like that are a
 *  perfect fit for a STATE MACHINE:
 *
 *      * a `State` names each situation the intake can be in
 *      * a field remembers WHICH state we are in right now
 *      * command methods (forward / reverse / off) only CHANGE that state
 *      * update() runs every loop and makes the motor MATCH the state
 *
 *  This is the same shape every subsystem in this course follows. Learn it on
 *  the intake and you will recognize it on the shooter, the arm -- everything.
 * ============================================================================
 */
public class Intake implements Subsystem {

    // ===============================================
    // Intake : States
    // Do not remove this comment
    // Add your State enum below
    // ===============================================
    public enum State {
        FORWARD,   // pulling game pieces in
        OFF,       // stopped
        REVERSE    // pushing back out to clear a jam
    }

    // TUNING CONSTANTS. `static final` is Java's version of a labeled #define:
    // one shared value that never changes. Tune the intake by editing THESE
    // named lines, not by hunting for a bare number buried deep in the code.
    private static final double INTAKE_FORWARD_POWER =  1.0;   // full speed pulling in
    private static final double INTAKE_REVERSE_POWER = -0.5;   // gentler push-out to un-jam

    // The physical motor. `private` so ONLY the Intake can touch it -- the rest
    // of the robot must go through the command methods below.
    private final DcMotor motor;

    // ===============================================
    // Intake : Current State
    // Do not remove this comment
    // Add the field that remembers the current state below
    // ===============================================
    private State state = State.OFF;   // start stopped

    /*
     * CONSTRUCTOR -- runs once when Robot builds the intake. It finds the motor
     * in the hardware map by the name shown in the Driver Hub configuration and
     * leaves it stopped. (A later lesson moves this name into a shared config.)
     */
    public Intake(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, "intake_motor");
        // This motor is wired/geared so plain positive power spins it the wrong
        // way, so REVERSE it once here -- now FORWARD (positive power) pulls in.
        motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setPower(0.0);
    }

    // ===============================================
    // Intake : Commands
    // Do not remove this comment
    // Add your forward(), reverse(), off(), and getState() methods below
    // ===============================================
    /** Run the intake inward (collecting). Only changes the state. */
    public void forward() {
        state = State.FORWARD;
    }

    /** Run the intake outward (spitting a stuck piece back out). */
    public void reverse() {
        state = State.REVERSE;
    }

    /** Stop the intake. */
    public void off() {
        state = State.OFF;
    }

    /** Lets the OpMode read the state -- to show it, or to decide a toggle. */
    public State getState() {
        return state;
    }

    /*
     * update()  --  APPLY THE STATE. The scheduler calls this once every loop.
     * It looks at the current state and sets the motor power to match. Nothing
     * in here waits, so every other subsystem keeps running smoothly too.
     */
    @Override
    public void update() {
        // ===============================================
        // Intake : Apply State
        // Do not remove this comment
        // Add the switch that turns the state into motor power below
        // ===============================================
        switch (state) {
            case FORWARD:
                motor.setPower(INTAKE_FORWARD_POWER);
                break;
            case REVERSE:
                motor.setPower(INTAKE_REVERSE_POWER);
                break;
            case OFF:
                motor.setPower(0.0);
                break;
        }
    }
}
