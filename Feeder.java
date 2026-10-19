package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
 * ============================================================================
 *  Feeder  --  2nd intake stage used as the indexer (hold X to feed)
 * ============================================================================
 *
 *  WHAT THIS IS
 *  ------------
 *  Our robot has a 2-stage intake driven by two goBILDA Yellow Jacket motors
 *  and NO indexer servo. This class runs the SECOND stage as the indexer:
 *  while the driver HOLDS X it runs at 100% and pushes stored pollen into the
 *  flywheel; let go and it stops. No reverse, no states -- just on or off.
 *
 *  It has the same commands as the lesson's Indexer -- feed(), stop() and
 *  isFeeding() -- so swapping it in is mostly renaming `indexer` to `feeder`.
 *
 *  Heads up: this will likely shoot everything at once. Try tapping X to pulse
 *  it. The motor is set to BRAKE so it stops as soon as X is released.
 *
 * ----------------------------------------------------------------------------
 *  OTHER CHANGES NEEDED (copy this file into your package as Feeder.java)
 * ----------------------------------------------------------------------------
 *
 *  1. DRIVER HUB / CONTROL HUB CONFIGURATION
 *     Add the 2nd-stage Yellow Jacket on a free motor port as a goBILDA motor
 *     named exactly:   feeder_motor
 *     (Stage 1 stays "intake_motor", driven by Intake.java as before.)
 *
 *  2. Robot.java  --  build the Feeder, and take the Indexer OUT
 *     Add a field next to the others:
 *
 *         public final Feeder feeder;
 *
 *     And in the constructor:
 *
 *         feeder = new Feeder(hardwareMap);
 *         subsystems.add(feeder);
 *
 *     REMOVE (or comment out) the Indexer field and its two construction lines
 *     (`indexer = new Indexer(hardwareMap);` and `subsystems.add(indexer);`).
 *     WHY: Indexer looks for a CRServo named "feeder". Our robot has none, so on
 *     the real robot it throws "No CRServo named feeder is found" and the whole
 *     OpMode fails at INIT.
 *
 *  3. MecanumDrive.java (TeleOp)  --  point X at the feeder
 *         robot.indexer.feed()      ->  robot.feeder.feed()
 *         robot.indexer.stop()      ->  robot.feeder.stop()
 *         robot.indexer.isFeeding() ->  robot.feeder.isFeeding()   (telemetry)
 *
 *  4. Any AUTO that shoots (e.g. SimpleAuto.java, or action code in a
 *     Path Capture auto) -- same rename: robot.indexer.* -> robot.feeder.*
 *     Tip: search the package for "indexer" -- after step 2, anything left
 *     will be a compile error pointing you to the line.
 *
 *  5. Spinning the wrong way? Change FORWARD to REVERSE in the constructor.
 *
 *  ABOUT THE SIMULATOR
 *  The sim has no 2nd intake motor, so in the sim this Feeder simply does
 *  nothing (it won't crash). With the Indexer removed, X won't shoot in the
 *  sim -- that's expected. Everything else (driving, intake, flywheel, autos
 *  that move) still works there.
 * ============================================================================
 */
public class Feeder implements Subsystem {

    private static final double FEED_POWER = 1.0;   // full speed while X is held

    private DcMotor motor;            // stays null in the sim (no such motor there)
    private boolean feeding = false;

    public Feeder(HardwareMap hardwareMap) {
        try {
            motor = hardwareMap.get(DcMotor.class, "feeder_motor");
            motor.setDirection(DcMotor.Direction.FORWARD);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setPower(0.0);
        } catch (IllegalArgumentException e) {
            motor = null;   // not configured (e.g. in the simulator) -- feeder does nothing
        }
    }

    /** Start feeding. Only changes the flag -- update() moves the motor. */
    public void feed() {
        feeding = true;
    }

    /** Stop feeding. */
    public void stop() {
        feeding = false;
    }

    /** True while feeding -- handy for telemetry. */
    public boolean isFeeding() {
        return feeding;
    }

    @Override
    public void update() {
        if (motor != null) {
            motor.setPower(feeding ? FEED_POWER : 0.0);
        }
    }
}
