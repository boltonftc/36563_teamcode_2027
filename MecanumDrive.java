package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * ============================================================================
 *  MecanumDrive  --  your TeleOp (the program a driver runs)
 * ============================================================================
 *
 *  Every FTC program is called an "OpMode". This one is a LinearOpMode: it
 *  runs top-to-bottom like a story. Notice the THREE parts you will see in
 *  every OpMode you ever write:
 *
 *     1. INIT  -- setup that runs ONCE, right after you press the INIT button.
 *     2. START -- the single moment the driver presses the play button.
 *                 waitForStart() pauses the program there until they do.
 *     3. LOOP  -- code that repeats about 50 times every second, from START
 *                 until the match ends.
 *
 *  Right now this program does almost nothing. You will grow it, one lesson at
 *  a time, until it drives a full robot.
 * ============================================================================
 */
@TeleOp(name = "MecanumDrive", group = "BIOBUZZ")
public class MecanumDrive extends LinearOpMode {

    // Edge-detection memory for the buttons (added in the subsystem lessons).
    // ===============================================
    // Intake : Variables
    // Do not remove this comment
    // Add your intake button-edge variables below
    // ===============================================
    private boolean previousA = false;
    private boolean previousB = false;

    // ===============================================
    // Flywheel : Variables
    // Do not remove this comment
    // Add your flywheel button-edge variable below
    // ===============================================
    private boolean previousY = false;

    @Override
    public void runOpMode() {

        // ===============================================
        // Drive : Setup
        // Do not remove this comment
        // Your four drive motors are created here, in INIT.
        // ===============================================
        // Build the whole robot. Every subsystem's constructor runs here (finding its motors and
        // servos), so it must happen during INIT, before the match starts.
        Robot robot = new Robot(hardwareMap);

        // ===============================================
        // IMU : Setup
        // Do not remove this comment
        // Get the IMU from the hardware map and initialize it below
        // ===============================================
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));

        // ─── INIT ──────────────────────────────────────────────────────
        // Everything here runs ONCE, after you press INIT. Show a message so the driver knows
        // the robot is ready.
        telemetry.addLine("Initialized. Press START.");
        telemetry.update();

        // START: pause here until the driver presses the play button.
        waitForStart();

        // ===============================================
        // Telemetry : Loop Counter
        // Do not remove this comment
        // Type your loop-counter variable below
        // ===============================================
        int loops = 0;

        // ─── THE RUN LOOP ──────────────────────────────────────────────
        // Repeats about 50 times a second until the match ends. Everything a driver does happens
        // in here, over and over.
        while (opModeIsActive()) {

            // ===============================================
            // IMU : Heading
            // Do not remove this comment
            // Read the heading, show it, and reset it on the right bumper
            // ===============================================
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            telemetry.addData("Heading (deg)", Math.toDegrees(heading));
            telemetry.addData("Heading (rad)", heading);
            if (gamepad1.right_bumper) {
                imu.resetYaw();
            }

            // ===============================================
            // Drive : Loop
            // Do not remove this comment
            // Your drive code runs here, every time through the loop.
            // ===============================================
            // 1) SCHEDULER: give every mechanism a turn to advance its own work.
            robot.update();

            // 2) DRIVE: hand the gamepad and the IMU heading to the drivebase.
            robot.drive.driveFieldCentric(gamepad1, heading);

            // 3) READ THE BUTTONS (with edge detection).
            // ===============================================
            // Intake : Loop
            // Do not remove this comment
            // Add your intake button code below
            // ===============================================
            boolean pressedA = gamepad1.a && !previousA;
            boolean pressedB = gamepad1.b && !previousB;
            if (pressedA) {
                if (robot.intake.getState() != Intake.State.OFF) robot.intake.off();
                else robot.intake.forward();
            }
            if (pressedB) {
                if (robot.intake.getState() != Intake.State.OFF) robot.intake.off();
                else robot.intake.reverse();
            }
            previousA = gamepad1.a;
            previousB = gamepad1.b;

            // ===============================================
            // Indexer : Loop
            // Do not remove this comment
            // Add your indexer button code below
            // ===============================================
            // Hold X to feed pieces out. No edge detection needed -- feeding is simply ON while held.
            if (gamepad1.x) robot.indexer.feed();
            else robot.indexer.stop();

            // ===============================================
            // Flywheel : Loop
            // Do not remove this comment
            // Add your flywheel button code below
            // ===============================================
            // Y toggles the flywheel. The controller holds the target; the right stick now trims the TARGET.
            if (gamepad1.y && !previousY) robot.flywheel.toggle();
            previousY = gamepad1.y;
            robot.flywheel.trim(-gamepad1.right_stick_y);

            // ===============================================
            // Hood : Loop
            // Do not remove this comment
            // Add your hood aim code below
            // ===============================================
            // Right stick LEFT/RIGHT aims the hood (up/down trims the flywheel target).
            robot.hood.aim(gamepad1.right_stick_x);

            // ===============================================
            // Gamepad : Read
            // Do not remove this comment
            // Read both sticks and the face buttons, then show them live
            // ===============================================
            double leftStickX = gamepad1.left_stick_x;
            double leftStickY = gamepad1.left_stick_y;
            double rightStickX = gamepad1.right_stick_x;
            double rightStickY = gamepad1.right_stick_y;
            boolean aHeld = gamepad1.a;
            boolean bHeld = gamepad1.b;
            boolean xHeld = gamepad1.x;
            boolean yHeld = gamepad1.y;
            telemetry.addData("Left Stick X", leftStickX);
            telemetry.addData("Left Stick Y", leftStickY);
            telemetry.addData("Right Stick X", rightStickX);
            telemetry.addData("Right Stick Y", rightStickY);
            telemetry.addData("A", aHeld);
            telemetry.addData("B", bHeld);
            telemetry.addData("X", xHeld);
            telemetry.addData("Y", yHeld);

            // ===============================================
            // Telemetry : Status
            // Do not remove this comment
            // Type your telemetry lines below
            // ===============================================
            telemetry.addData("Status", "Hello, Robot!");
            loops += 1;
            telemetry.addData("Loops", loops);

            telemetry.addData("Intake", robot.intake.getState());
            telemetry.addData("Indexer", robot.indexer.isFeeding() ? "FEEDING" : "idle");
            // ===============================================
            // Flywheel : Telemetry
            // Do not remove this comment
            // Show the flywheel status and measured RPM below
            // ===============================================
            telemetry.addData("Flywheel", robot.flywheel.isSpinning()
                ? String.format("ON  %.0f%% power", robot.flywheel.getPower() * 100)
                : "off");
            telemetry.addData("Flywheel RPM", "%.0f", robot.flywheel.getRpm());
            telemetry.addData("Target RPM", "%.0f", robot.flywheel.getTarget());
            telemetry.addData("RPM error", "%.0f", robot.flywheel.getTarget() - robot.flywheel.getRpm());
            telemetry.addData("Hood", "%.2f", robot.hood.getPosition());
            // ===============================================
            // Localization : Telemetry
            // Do not remove this comment
            // Show where the robot thinks it is below
            // ===============================================
            telemetry.addData("Has Pinpoint", robot.localization.hasPinpoint());
            telemetry.addData("Heading (deg)", "%.1f", robot.localization.getHeadingDegrees());
            telemetry.addData("Position (in)", "x %.1f  y %.1f",
                robot.localization.getXInches(), robot.localization.getYInches());
            telemetry.addData("Distance from start (in)", "%.1f",
                robot.localization.getDistanceFromStartInches());

            telemetry.update();
        }
    }
}
