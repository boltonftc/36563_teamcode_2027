


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
@Autonomous(name = "shootauto3V2", group = "Captured")
public class shootauto3V2 extends LinearOpMode {

    private static final double SPEED = 1;

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
        Pose pose1 = new Pose(64.28, -12.57, Math.toRadians(-178.3));
        // CAP:POSE seq=2 kind=pose
        Pose pose2 = new Pose(64.24, -12.51, Math.toRadians(-180.0));
        // CAP:POSE seq=4 kind=pose
        Pose pose3 = new Pose(45.16, -27.01, Math.toRadians(-91.5));
        // CAP:POSE seq=5 kind=pose
        Pose pose4 = new Pose(52.29, -50.90, Math.toRadians(4.5));
        // CAP:POSE seq=6 kind=pose
        Pose pose5 = new Pose(66.64, -62.60, Math.toRadians(-0.0));
        // CAP:POSE seq=8 kind=pose
        Pose pose6 = new Pose(61.56, -62.43, Math.toRadians(-1.4));
        // CAP:POSE seq=10 kind=pose
        Pose pose7 = new Pose(32.83, -46.22, Math.toRadians(1.4));
        // CAP:POSE seq=11 kind=pose
        Pose pose8 = new Pose(-37.48, -37.59, Math.toRadians(-3.4));
        // CAP:POSE seq=12 kind=pose
        Pose pose9 = new Pose(-42.12, -15.30, Math.toRadians(-2.9));
        // CAP:POSE seq=13 kind=pose
        Pose pose10 = new Pose(-55.86, -12.08, Math.toRadians(0.0));
        // CAP:POSE seq=15 kind=pose
        Pose pose11 = new Pose(-41.80, -22.03, Math.toRadians(177.2));
        // CAP:POSE seq=17 kind=pose
        Pose pose12 = new Pose(-53.27, -22.85, Math.toRadians(179.6));
        // CAP:POSE seq=19 kind=pose
        Pose pose13 = new Pose(-48.90, -22.85, Math.toRadians(-179.8));
        // CAP:POSE seq=21 kind=pose
        Pose pose14 = new Pose(-48.11, -59.19, Math.toRadians(84.8));

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
                .build();

        PathChain chain3 = follower.pathBuilder()
                .addPath(new BezierLine(pose5, pose6))
                .setLinearHeadingInterpolation(pose5.getHeading(), pose6.getHeading())
                .build();

        PathChain chain4 = follower.pathBuilder()
                .addPath(new BezierLine(pose6, pose7))
                .setLinearHeadingInterpolation(pose6.getHeading(), pose7.getHeading())
                .addPath(new BezierLine(pose7, pose8))
                .setLinearHeadingInterpolation(pose7.getHeading(), pose8.getHeading())
                .addPath(new BezierLine(pose8, pose9))
                .setLinearHeadingInterpolation(pose8.getHeading(), pose9.getHeading())
                .addPath(new BezierLine(pose9, pose10))
                .setLinearHeadingInterpolation(pose9.getHeading(), pose10.getHeading())
                .build();

        PathChain chain5 = follower.pathBuilder()
                .addPath(new BezierLine(pose10, pose11))
                .setLinearHeadingInterpolation(pose10.getHeading(), pose11.getHeading())
                .build();

        PathChain chain6 = follower.pathBuilder()
                .addPath(new BezierLine(pose11, pose12))
                .setLinearHeadingInterpolation(pose11.getHeading(), pose12.getHeading())
                .build();

        PathChain chain7 = follower.pathBuilder()
                .addPath(new BezierLine(pose12, pose13))
                .setLinearHeadingInterpolation(pose12.getHeading(), pose13.getHeading())
                .build();

        PathChain chain8 = follower.pathBuilder()
                .addPath(new BezierLine(pose13, pose14))
                .setLinearHeadingInterpolation(pose13.getHeading(), pose14.getHeading())
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

        robot.flywheel.toggle();                                  // flywheel ON (default target RPM)
        for (int i = 0; i < 70 && opModeIsActive(); i++) {       // ~3 s: let it spin up
            follower.update();                                    // hold our spot
            robot.update();                                       // make the flywheel actually spin
            sleep(20);
        }
        robot.intake.forward();
        robot.feeder.feed();                                     // same as holding X
        for (int i = 0; i < 50 && opModeIsActive(); i++) {       // ~5 s: shoot
            follower.update();
            robot.update();
            sleep(20);
        }
        //robot.flywheel.toggle();                                  // flywheel OFF
        robot.feeder.stop();                                     // indexer OFF
        robot.update(); 
        
        follower.followPath(chain1, SPEED, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            aim.aim(robot.localization, robot.turret);   // keep the turret on the hive while driving
            robot.update();
            saveForTeleOp(follower);
            telemetry.addData("Phase", "Chain 1");
            telemetryPose(follower, 64.24, -12.51, -180.0);
        }

        robot.update();
        aim.aim(robot.localization, robot.turret);   // final aim from where we stopped (the servo holds it)
        robot.update();
        // CAP:ACTION seq=3 button=X
        // ── Captured action: Button X pressed here ──
        // TODO: call your subsystem here, e.g. robot.intake.forward(); or robot.intake.reverse();
        // To shoot here, see the lesson "Actions in Autonomous: Make It Shoot".
        // CAP:ACTION-END seq=3

        follower.followPath(chain2, SPEED, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            aim.aim(robot.localization, robot.turret);   // keep the turret on the hive while driving
            robot.update();
            saveForTeleOp(follower);
            telemetry.addData("Phase", "Chain 2");
            telemetryPose(follower, 59.64, -62.60, -0.0);
        }

        robot.update();
        aim.aim(robot.localization, robot.turret);   // final aim from where we stopped (the servo holds it)
        robot.update();
        // CAP:ACTION seq=7 button=A
        // ── Captured action: Button A pressed here ──
        // TODO: call your subsystem here, e.g. robot.intake.forward(); or robot.intake.reverse();
        // To shoot here, see the lesson "Actions in Autonomous: Make It Shoot".
        // CAP:ACTION-END seq=7

        follower.followPath(chain3, SPEED, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            aim.aim(robot.localization, robot.turret);   // keep the turret on the hive while driving
            robot.update();
            saveForTeleOp(follower);
            telemetry.addData("Phase", "Chain 3");
            telemetryPose(follower, 61.56, -62.43, -1.4);
        }

        robot.update();
        aim.aim(robot.localization, robot.turret);   // final aim from where we stopped (the servo holds it)
        robot.update();
        // CAP:ACTION seq=9 button=A
        // ── Captured action: Button A pressed here ──
        // TODO: call your subsystem here, e.g. robot.intake.forward(); or robot.intake.reverse();
        // To shoot here, see the lesson "Actions in Autonomous: Make It Shoot".
        // CAP:ACTION-END seq=9

        follower.followPath(chain4, SPEED, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            aim.aim(robot.localization, robot.turret);   // keep the turret on the hive while driving
            robot.update();
            saveForTeleOp(follower);
            telemetry.addData("Phase", "Chain 4");
            telemetryPose(follower, -61.86, -12.08, 0.0);
        }

        robot.update();
        aim.aim(robot.localization, robot.turret);   // final aim from where we stopped (the servo holds it)
        robot.update();
        // CAP:ACTION seq=14 button=X
        // ── Captured action: Button X pressed here ──
        // TODO: call your subsystem here, e.g. robot.intake.forward(); or robot.intake.reverse();
        // To shoot here, see the lesson "Actions in Autonomous: Make It Shoot".
        // CAP:ACTION-END seq=14                               // flywheel ON (default target RPM)
       
        robot.intake.forward();
        robot.feeder.feed();                                     // same as holding X
        for (int i = 0; i < 100 && opModeIsActive(); i++) {       // ~5 s: shoot
            follower.update();
            robot.update();
            sleep(20);
        }
        robot.feeder.stop();                                     // indexer OFF
        robot.update(); 

        follower.followPath(chain5, SPEED, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            aim.aim(robot.localization, robot.turret);   // keep the turret on the hive while driving
            robot.update();
            saveForTeleOp(follower);
            telemetry.addData("Phase", "Chain 5");
            telemetryPose(follower, -41.80, -22.03, 177.2);
        }

        robot.update();
        aim.aim(robot.localization, robot.turret);   // final aim from where we stopped (the servo holds it)
        robot.update();
        // CAP:ACTION seq=16 button=A
        // ── Captured action: Button A pressed here ──
        // TODO: call your subsystem here, e.g. robot.intake.forward(); or robot.intake.reverse();
        // To shoot here, see the lesson "Actions in Autonomous: Make It Shoot".
        // CAP:ACTION-END seq=16
        robot.extractor.deploy();
        robot.update();

        follower.followPath(chain6, SPEED, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            aim.aim(robot.localization, robot.turret);   // keep the turret on the hive while driving
            robot.update();
            saveForTeleOp(follower);
            telemetry.addData("Phase", "Chain 6");
            telemetryPose(follower, -59.27, -22.85, 179.6);
        }

        robot.update();
        aim.aim(robot.localization, robot.turret);   // final aim from where we stopped (the servo holds it)
        robot.update();
        // CAP:ACTION seq=18 button=A
        // ── Captured action: Button A pressed here ──
        // TODO: call your subsystem here, e.g. robot.intake.forward(); or robot.intake.reverse();
        // To shoot here, see the lesson "Actions in Autonomous: Make It Shoot".
        // CAP:ACTION-END seq=18

        follower.followPath(chain7, SPEED, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            aim.aim(robot.localization, robot.turret);   // keep the turret on the hive while driving
            robot.update();
            saveForTeleOp(follower);
            telemetry.addData("Phase", "Chain 7");
            telemetryPose(follower, -53.90, -22.85, -179.8);
        }

        robot.update();
        aim.aim(robot.localization, robot.turret);   // final aim from where we stopped (the servo holds it)
        robot.update();
        robot.extractor.stow();
        robot.update();
        // CAP:ACTION seq=20 button=X
        // ── Captured action: Button X pressed here ──
        // TODO: call your subsystem here, e.g. robot.intake.forward(); or robot.intake.reverse();
        // To shoot here, see the lesson "Actions in Autonomous: Make It Shoot".
        // CAP:ACTION-END seq=20

        follower.followPath(chain8, SPEED, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            aim.aim(robot.localization, robot.turret);   // keep the turret on the hive while driving
            robot.update();
            saveForTeleOp(follower);
            telemetry.addData("Phase", "Chain 8");
            telemetryPose(follower, -48.11, -59.19, 84.8);
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
