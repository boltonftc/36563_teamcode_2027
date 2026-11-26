package org.firstinspires.ftc.teamcode;

// CAP:FILE version=3
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
//
// ======================== KEEPING YOUR OWN CODE ========================
// You can re-open this file in the Capture tab (Load Capture), move or add poses, change speeds, and
// Save again. Saving REBUILDS this file from the captured path. Your own code survives that ONLY here:
//
//  1) STUDENT-CODE blocks -- wrap your code in these two marker lines, almost anywhere in the file:
//         // STUDENT-CODE-BEGIN
//         robot.extractor.deploy();
//         robot.update();
//         // STUDENT-CODE-END
//     WHERE THEY WORK:
//       * inside runOpMode(), between any two statements: during INIT (before waitForStart), right after
//         START (e.g. a preload shot before the first path), INSIDE a path's while-loop (runs every loop
//         while that path drives -- e.g. run the intake on the way), between paths, or at the end;
//       * at class level, outside runOpMode(): your own fields, constants and helper methods.
//     HOW THEY MOVE: a block stays directly after the line it follows now. Moving a pose, changing a
//       speed or adding poses/actions at the end does not change that line. If the line is gone (e.g.
//       you re-captured the path), the block goes to the nearest earlier line that still exists and
//       Save tells you to check it.
//     NOT ALLOWED: inside a generated statement (e.g. in the middle of a pathBuilder() chain), a block
//       inside another block, or a BEGIN without its END (Save refuses until it is fixed).
//  2) CAP:ACTION blocks -- the lines between "// CAP:ACTION seq=N button=X" and "// CAP:ACTION-END seq=N"
//     (made where you pressed A/B/X/Y while capturing) are kept too. No extra markers needed there.
//  3) Kept automatically: import lines you add, and your tuning in the FollowerConstants block.
//
// SPEED and per-pose speeds are set in the Capture tab (Load Capture reads them back from this file).
// Everything else is regenerated. If Save would throw away or reset anything, the Capture tab first
// lists it and lets you Cancel, Save as a new name, or Save anyway.
// DON'T edit, move or delete the "// CAP:..." lines or the numbers in "new Pose(...)" -- move poses
// in the Capture tab instead. Without its CAP: lines this file can't be re-opened there.
//
// Coordinates: inches from the field center, seen from YOUR drivers (red or blue -- the field is the same
// from both sides): +X = their right, +Y = away from them, heading in degrees counter-clockwise from +X
// (90 = facing away from the drivers). On blue, place the robot on the mirror-image spot. TeleOp and
// auto-aim use the same frame, so the pose this auto ends at can be handed straight to TeleOp.
@Autonomous(name = "shootauto3V2", group = "Captured")
public class shootauto3V2 extends LinearOpMode {

    // Default max power (0..1) for every leg without its own speed. Set it in the Capture tab.
    private static final double SPEED = 1.0;

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

        // CAP:POSE seq=1 kind=init id=s1
        Pose pose1 = new Pose(64.28, -12.57, Math.toRadians(-178.3));
        // CAP:POSE seq=2 kind=pose id=s2
        Pose pose2 = new Pose(64.24, -12.51, Math.toRadians(-180.0));
        // CAP:POSE seq=4 kind=pose id=s4
        Pose pose3 = new Pose(45.16, -27.01, Math.toRadians(-91.5));
        // CAP:POSE seq=5 kind=pose id=s5
        Pose pose4 = new Pose(52.29, -50.90, Math.toRadians(4.5));
        // CAP:POSE seq=6 kind=pose speed=0.6 id=s6
        Pose pose5 = new Pose(65.81, -63.85, Math.toRadians(0.0));
        // CAP:POSE seq=8 kind=pose id=s8
        Pose pose6 = new Pose(61.56, -62.43, Math.toRadians(-1.4));
        // CAP:POSE seq=10 kind=pose id=s10
        Pose pose7 = new Pose(32.83, -46.22, Math.toRadians(1.4));
        // CAP:POSE seq=11 kind=pose id=s11
        Pose pose8 = new Pose(-37.48, -37.59, Math.toRadians(-3.4));
        // CAP:POSE seq=12 kind=pose id=s12
        Pose pose9 = new Pose(-42.12, -15.30, Math.toRadians(-2.9));
        // CAP:POSE seq=13 kind=pose id=s13
        Pose pose10 = new Pose(-55.86, -12.08, Math.toRadians(0.0));
        // CAP:POSE seq=15 kind=pose speed=0.5 id=s15
        Pose pose11 = new Pose(-39.70, -26.40, Math.toRadians(177.0));
        // CAP:POSE seq=17 kind=pose speed=0.5 id=s17
        Pose pose12 = new Pose(-52.50, -24.80, Math.toRadians(-180.0));
        // CAP:POSE seq=19 kind=pose speed=0.5 id=ldkcz
        Pose pose13 = new Pose(-44.40, -25.00, Math.toRadians(-180.0));
        // CAP:POSE seq=20 kind=pose speed=0.5 id=e6bis
        Pose pose14 = new Pose(-51.80, -25.20, Math.toRadians(176.0));
        // CAP:POSE seq=21 kind=pose speed=0.5 id=klwdr
        Pose pose15 = new Pose(-44.40, -24.30, Math.toRadians(-180.0));
        // CAP:POSE seq=22 kind=pose speed=0.5 id=44zhi
        Pose pose16 = new Pose(-52.40, -24.30, Math.toRadians(-180.0));
        // CAP:POSE seq=23 kind=pose speed=0.5 id=fn3si
        Pose pose17 = new Pose(-44.40, -24.70, Math.toRadians(-180.0));
        // CAP:POSE seq=24 kind=pose speed=0.5 id=kejj1
        Pose pose18 = new Pose(-51.70, -24.70, Math.toRadians(-180.0));
        // CAP:POSE seq=25 kind=pose speed=0.5 id=9kt78
        Pose pose19 = new Pose(-44.70, -25.50, Math.toRadians(-180.0));
        // CAP:POSE seq=26 kind=pose speed=0.5 id=s19
        Pose pose20 = new Pose(-42.93, -24.84, Math.toRadians(-180.0));
        // CAP:POSE seq=28 kind=pose id=s21
        Pose pose21 = new Pose(-48.11, -59.19, Math.toRadians(84.8));

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
                .addParametricCallback(0, setSpeed(follower, 0.6))   // this leg's speed
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
                .addPath(new BezierLine(pose13, pose14))
                .setLinearHeadingInterpolation(pose13.getHeading(), pose14.getHeading())
                .addPath(new BezierLine(pose14, pose15))
                .setLinearHeadingInterpolation(pose14.getHeading(), pose15.getHeading())
                .addPath(new BezierLine(pose15, pose16))
                .setLinearHeadingInterpolation(pose15.getHeading(), pose16.getHeading())
                .addPath(new BezierLine(pose16, pose17))
                .setLinearHeadingInterpolation(pose16.getHeading(), pose17.getHeading())
                .addPath(new BezierLine(pose17, pose18))
                .setLinearHeadingInterpolation(pose17.getHeading(), pose18.getHeading())
                .addPath(new BezierLine(pose18, pose19))
                .setLinearHeadingInterpolation(pose18.getHeading(), pose19.getHeading())
                .addPath(new BezierLine(pose19, pose20))
                .setLinearHeadingInterpolation(pose19.getHeading(), pose20.getHeading())
                .build();

        PathChain chain8 = follower.pathBuilder()
                .addPath(new BezierLine(pose20, pose21))
                .setLinearHeadingInterpolation(pose20.getHeading(), pose21.getHeading())
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
        aim.aim(robot.localization, robot.turret);   // aim from the start pose before anything else
        robot.update();
        // STUDENT-CODE-BEGIN   preload: spin up, then shoot before the first path
        robot.flywheel.toggle();                                  // flywheel ON (default target RPM)
        for (int i = 0; i < 50 && opModeIsActive(); i++) {       // ~3 s: let it spin up
            follower.update();                                    // hold our spot
            robot.update();                                       // make the flywheel actually spin
            sleep(20);
        }
        robot.intake.forward();
        robot.feeder.feed();                                     // same as holding X
        for (int i = 0; i < 20 && opModeIsActive(); i++) {       // ~5 s: shoot
            follower.update();
            robot.update();
            sleep(20);
        }
        //robot.flywheel.toggle();                                  // flywheel OFF
        robot.feeder.stop();                                     // indexer OFF
        robot.update(); 
        // STUDENT-CODE-END

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
        // CAP:ACTION seq=3 button=X id=s3
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
            telemetryPose(follower, 65.81, -63.85, 0.0);
        }

        robot.update();
        aim.aim(robot.localization, robot.turret);   // final aim from where we stopped (the servo holds it)
        robot.update();
        // CAP:ACTION seq=7 button=A id=s7
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
        // CAP:ACTION seq=9 button=A id=s9
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
            telemetryPose(follower, -55.86, -12.08, 0.0);
        }

        robot.update();
        aim.aim(robot.localization, robot.turret);   // final aim from where we stopped (the servo holds it)
        robot.update();
        // CAP:ACTION seq=14 button=X id=s14
        robot.intake.forward();
        robot.feeder.feed();                                     // same as holding X
        for (int i = 0; i < 100 && opModeIsActive(); i++) {       // ~5 s: shoot
            follower.update();
            robot.update();
            sleep(20);
        }
        robot.feeder.stop();                                     // indexer OFF
        robot.update();
        // CAP:ACTION-END seq=14

        follower.followPath(chain5, 0.5, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            aim.aim(robot.localization, robot.turret);   // keep the turret on the hive while driving
            robot.update();
            saveForTeleOp(follower);
            telemetry.addData("Phase", "Chain 5");
            telemetryPose(follower, -39.70, -26.40, 177.0);
        }

        robot.update();
        aim.aim(robot.localization, robot.turret);   // final aim from where we stopped (the servo holds it)
        robot.update();
        // CAP:ACTION seq=16 button=A id=s16
        robot.extractor.deploy();
        robot.update();
        // CAP:ACTION-END seq=16

        follower.followPath(chain6, 0.5, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            aim.aim(robot.localization, robot.turret);   // keep the turret on the hive while driving
            robot.update();
            saveForTeleOp(follower);
            telemetry.addData("Phase", "Chain 6");
            telemetryPose(follower, -52.50, -24.80, -180.0);
        }

        robot.update();
        aim.aim(robot.localization, robot.turret);   // final aim from where we stopped (the servo holds it)
        robot.update();
        // CAP:ACTION seq=18 button=A id=s18
        // ── Captured action: Button A pressed here ──
        // TODO: call your subsystem here, e.g. robot.intake.forward(); or robot.intake.reverse();
        // To shoot here, see the lesson "Actions in Autonomous: Make It Shoot".
        // CAP:ACTION-END seq=18

        follower.followPath(chain7, 0.5, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            aim.aim(robot.localization, robot.turret);   // keep the turret on the hive while driving
            robot.update();
            saveForTeleOp(follower);
            telemetry.addData("Phase", "Chain 7");
            telemetryPose(follower, -42.93, -24.84, -180.0);
        }

        robot.update();
        aim.aim(robot.localization, robot.turret);   // final aim from where we stopped (the servo holds it)
        robot.update();
        // CAP:ACTION seq=27 button=X id=s20
        robot.extractor.stow();
        robot.update();
        // CAP:ACTION-END seq=27

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

    // Per-leg speed (set on each pose in the Capture tab): Pedro runs this as that leg starts (t = 0)
    // and changes the max power mid-chain, without stopping.
    private static Runnable setSpeed(final Follower follower, final double maxPower) {
        return () -> follower.setMaxPower(maxPower);
    }
}
