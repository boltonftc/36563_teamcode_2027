package org.firstinspires.ftc.teamcode;

// CAP:FILE version=2
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// Captured with the simulator's Path Capture tool (Capture tab). Runs unmodified on the sim or the real robot.
// This file can be re-opened there (Capture tab -> Edit existing) IF its "// CAP:..." marker comments
// below are left in place: don't remove/reorder a "// CAP:POSE" line or a "// CAP:ACTION"/"// CAP:ACTION-END"
// pair. You MAY add your own code between a CAP:ACTION line and its matching CAP:ACTION-END -- that's
// the only region this tool preserves verbatim when the file is loaded back into the capture GUI.
//
// Coordinates: inches from the field center, seen from YOUR drivers (red or blue -- the field is the same
// from both sides): +X = their right, +Y = away from them, heading in degrees counter-clockwise from +X
// (90 = facing away from the drivers). On blue, place the robot on the mirror-image spot. TeleOp and
// auto-aim use the same frame, so the pose this auto ends at can be handed straight to TeleOp.
@Autonomous(name = "shootauto1", group = "Captured")
public class shootauto1 extends LinearOpMode {

    private static final double SPEED = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap);
        MecanumDrivetrainSubsystem drivetrain = new MecanumDrivetrainSubsystem(hardwareMap);

        FollowerConstants constants = new FollowerConstants()
                .translationalPIDFCoefficients(new PIDFCoefficients(0.3, 0, 0.03, 0))
                .headingPIDFCoefficients(new PIDFCoefficients(2.0, 0, 0.1, 0))
                .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.02, 0, 0.0005, 0.6, 0))
                .mass(13.0)
                .forwardZeroPowerAcceleration(30)
                .lateralZeroPowerAcceleration(50)
                .centripetalScaling(0.0005)
                .holdPointTranslationalScaling(0.45)
                .holdPointHeadingScaling(0.35);

        Follower follower = new Follower(constants, localizer, drivetrain);

        // CAP:POSE seq=1 kind=init
        Pose pose1 = new Pose(64.36, -32.40, Math.toRadians(-180.0));
        // CAP:POSE seq=2 kind=pose
        Pose pose2 = new Pose(58.34, -32.02, Math.toRadians(176.7));
        // CAP:POSE seq=4 kind=pose
        Pose pose3 = new Pose(44.81, -44.79, Math.toRadians(-8.8));
        // CAP:POSE seq=5 kind=pose
        Pose pose4 = new Pose(52.46, -61.67, Math.toRadians(-5.9));
        // CAP:POSE seq=6 kind=pose
        Pose pose5 = new Pose(0.11, -45.25, Math.toRadians(177.6));
        // CAP:POSE seq=7 kind=pose
        Pose pose6 = new Pose(-42.25, -42.53, Math.toRadians(143.2));
        // CAP:POSE seq=8 kind=pose
        Pose pose7 = new Pose(-48.51, -14.79, Math.toRadians(-5.5));
        // CAP:POSE seq=9 kind=pose
        Pose pose8 = new Pose(-60.77, -9.83, Math.toRadians(-0.1));
        // CAP:POSE seq=11 kind=pose
        Pose pose9 = new Pose(-43.21, -10.12, Math.toRadians(50.2));
        // CAP:POSE seq=12 kind=pose
        Pose pose10 = new Pose(-38.11, -59.42, Math.toRadians(87.9));

        PathChain chain1 = follower.pathBuilder()
                .addPath(new BezierLine(pose1, pose2))
                .setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading())
                .build();

        PathChain chain2 = follower.pathBuilder()
                .addPath(new BezierLine(pose2, pose3))
                .setLinearHeadingInterpolation(pose2.getHeading(), pose3.getHeading())
                .addPath(new BezierLine(pose3, pose4))
                .setLinearHeadingInterpolation(pose3.getHeading(), pose4.getHeading())
                .addPath(new BezierLine(pose4, pose5))
                .setLinearHeadingInterpolation(pose4.getHeading(), pose5.getHeading())
                .addPath(new BezierLine(pose5, pose6))
                .setLinearHeadingInterpolation(pose5.getHeading(), pose6.getHeading())
                .addPath(new BezierLine(pose6, pose7))
                .setLinearHeadingInterpolation(pose6.getHeading(), pose7.getHeading())
                .addPath(new BezierLine(pose7, pose8))
                .setLinearHeadingInterpolation(pose7.getHeading(), pose8.getHeading())
                .build();

        PathChain chain3 = follower.pathBuilder()
                .addPath(new BezierLine(pose8, pose9))
                .setLinearHeadingInterpolation(pose8.getHeading(), pose9.getHeading())
                .addPath(new BezierLine(pose9, pose10))
                .setLinearHeadingInterpolation(pose9.getHeading(), pose10.getHeading())
                .build();

        Pose startPose = pose1;
        // this is the ONLY absolute position source we have (no AprilTags yet) -- it MUST
        // match wherever the robot is physically placed on the field before the match starts.
        follower.setStartingPose(startPose);
        follower.update();   // sync the localizer/PoseTracker once before reading getPose() below
        // Auto-aim: tell Localization the same start pose so AutoAim knows where we are, and aim
        // the turret every loop. (Added because this package has AutoAim.java.)
        robot.localization.setPose(startPose.getX(), startPose.getY(), Math.toDegrees(startPose.getHeading()));
        AutoAim aim = new AutoAim();

        telemetry.addData("Status", "Captured auto ready. Hold the robot on the start position, then press START.");
        telemetry.addData("Declared start (x,y,h)", "(%.1f, %.1f, %.1f deg)", startPose.getX(), startPose.getY(), Math.toDegrees(startPose.getHeading()));
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        follower.update();
        // A real Pinpoint's setPosition() needs a few I2C update cycles to report the
        // declared start pose, so settle the localizer before following. No "drive to
        // start" correction: with no absolute sensor the declared start IS ground truth,
        // and driving to it only chased localizer lag -- causing a spurious lunge + spin.
        for (int i = 0; i < 10 && opModeIsActive(); i++) { follower.update(); sleep(20); }
        Pose actualStart = follower.getPose();
        telemetry.addData("Actual pose at START", "(%.1f, %.1f, %.1f deg)", actualStart.getX(), actualStart.getY(), Math.toDegrees(actualStart.getHeading()));
        telemetry.update();

        follower.followPath(chain1, SPEED, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            aim.aim(robot.localization, robot.turret);   // keep the turret on the hive while driving
            robot.update();
            saveForTeleOp(follower);
            telemetry.addData("Phase", "Chain 1");
            telemetryPose(follower, 58.34, -32.02, 176.7);
        }

        robot.update();
        aim.aim(robot.localization, robot.turret);   // final aim from where we stopped (the servo holds it)
        robot.update();
        // CAP:ACTION seq=3 button=X
        // ── Captured action: Button X pressed here ──
        // TODO: call your subsystem here, e.g. robot.intake.forward(); or robot.intake.reverse();
        // To shoot here, see the lesson "Actions in Autonomous: Make It Shoot".
              robot.flywheel.toggle();                                  // flywheel ON (default target RPM)
        for (int i = 0; i < 150 && opModeIsActive(); i++) {       // ~3 s: let it spin up
            follower.update();                                    // hold our spot
            robot.update();                                       // make the flywheel actually spin
            sleep(20);
        }
robot.intake.forward();
        robot.feeder.feed();                                     // same as holding X
        for (int i = 0; i < 250 && opModeIsActive(); i++) {       // ~5 s: shoot
            follower.update();
            robot.update();
            sleep(20);
        }
        robot.flywheel.toggle();                                  // flywheel OFF
        robot.feeder.stop();                                     // indexer OFF
        robot.update();                                           // apply both right away
        
        // CAP:ACTION-END seq=3

        follower.followPath(chain2, SPEED, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            aim.aim(robot.localization, robot.turret);   // keep the turret on the hive while driving
            robot.update();
            saveForTeleOp(follower);
            telemetry.addData("Phase", "Chain 2");
            telemetryPose(follower, -60.77, -9.83, -0.1);
        }

        robot.update();
        aim.aim(robot.localization, robot.turret);   // final aim from where we stopped (the servo holds it)
        robot.update();
        // CAP:ACTION seq=10 button=X
        // ── Captured action: Button X pressed here ──
        // TODO: call your subsystem here, e.g. robot.intake.forward(); or robot.intake.reverse();
        // To shoot here, see the lesson "Actions in Autonomous: Make It Shoot".
        
        // CAP:ACTION-END seq=10

        follower.followPath(chain3, SPEED, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            aim.aim(robot.localization, robot.turret);   // keep the turret on the hive while driving
            robot.update();
            saveForTeleOp(follower);
            telemetry.addData("Phase", "Chain 3");
            telemetryPose(follower, -38.11, -59.42, 87.9);
        }

        drivetrain.breakFollowing();
        robot.intake.off();

        Pose finalPose = follower.getPose();
        saveForTeleOp(follower);
        telemetry.addData("Status", "COMPLETE");
        telemetry.addData("Final pose (x,y,h)", "(%.1f, %.1f, %.1f deg)", finalPose.getX(), finalPose.getY(), Math.toDegrees(finalPose.getHeading()));
        telemetry.update();
    }

    // Hand the robot's position to TeleOp (lesson "Auto -> TeleOp: Keep Your Place"). The SDK's blackboard survives from one OpMode
    // to the next, so TeleOp starts out knowing where auto left the robot. Saved every loop, not just at the
    // end, because the match timer can stop auto partway through a path.
    private void saveForTeleOp(Follower follower) {
        Pose p = follower.getPose();
        blackboard.put("autoPose", new double[] { p.getX(), p.getY(), Math.toDegrees(p.getHeading()) });
    }

    // shared per-loop telemetry: coordinate legend + tracked pose vs the active target + error
    private void telemetryPose(Follower follower, double tx, double ty, double thDeg) {
        Pose p = follower.getPose();
        double hDeg = Math.toDegrees(p.getHeading());
        telemetry.addData("Coords", "inches from field center, +X = your drivers' right, +Y = away from them, heading deg CCW+");
        telemetry.addData("Pinpoint pose (x,y,h)", "(%.1f, %.1f, %.1f)", p.getX(), p.getY(), hDeg);
        telemetry.addData("Target pose  (x,y,h)", "(%.1f, %.1f, %.1f)", tx, ty, thDeg);
        telemetry.addData("Error (dx,dy,dh)", "(%.1f, %.1f, %.1f)", tx - p.getX(), ty - p.getY(), thDeg - hDeg);
        telemetry.addData("Busy", follower.isBusy());
        telemetry.update();
    }
}
