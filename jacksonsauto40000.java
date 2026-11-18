package org.firstinspires.ftc.teamcode;

// CAP:FILE version=1
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
@Autonomous(name = "jacksonsauto40000", group = "Captured")
public class jacksonsauto40000 extends LinearOpMode {

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
        Pose pose1 = new Pose(65.43, 35.18, Math.toRadians(180.0));
        // CAP:POSE seq=2 kind=pose
        Pose pose2 = new Pose(13.21, 61.48, Math.toRadians(90.0));

        PathChain chain1 = follower.pathBuilder()
                .addPath(new BezierLine(pose1, pose2))
                .setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading())
                .build();

        Pose startPose = pose1;
        // this is the ONLY absolute position source we have (no AprilTags yet) -- it MUST
        // match wherever the robot is physically placed on the field before the match starts.
        follower.setStartingPose(startPose);
        follower.update();   // sync the localizer/PoseTracker once before reading getPose() below

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
            robot.update();
            telemetry.addData("Phase", "Chain 1");
            telemetryPose(follower, 13.21, 61.48, 90.0);
        }

        // CAP:ACTION seq=3 button=X
        // ── Captured action: Button X pressed here ──
        // TODO: call your subsystem here, e.g. robot.intake.forward(); or robot.intake.reverse();
        // To shoot here, see the lesson "Actions in Autonomous: Make It Shoot".
        
        // CAP:ACTION-END seq=3
        robot.flywheel.toggle();                                  // flywheel ON (default target RPM)
        for (int i = 0; i < 150 && opModeIsActive(); i++) {       // ~3 s: let it spin up
            follower.update();                                    // hold our spot
            robot.update();                                       // make the flywheel actually spin
            sleep(20);
        }
        robot.indexer.feed();                                     // same as holding X
        for (int i = 0; i < 250 && opModeIsActive(); i++) {       // ~5 s: shoot
            follower.update();
            robot.update();
            sleep(20);
        }
        robot.flywheel.toggle();                                  // flywheel OFF
        robot.indexer.stop();                                     // indexer OFF
        robot.update();                                           // apply both right away

        drivetrain.breakFollowing();
        robot.intake.off();

        Pose finalPose = follower.getPose();
        telemetry.addData("Status", "COMPLETE");
        telemetry.addData("Final pose (x,y,h)", "(%.1f, %.1f, %.1f deg)", finalPose.getX(), finalPose.getY(), Math.toDegrees(finalPose.getHeading()));
        telemetry.update();
    }

    // shared per-loop telemetry: coordinate legend + tracked pose vs the active target + error
    private void telemetryPose(Follower follower, double tx, double ty, double thDeg) {
        Pose p = follower.getPose();
        double hDeg = Math.toDegrees(p.getHeading());
        telemetry.addData("Coords", "FTC/Pedro inches, heading deg CCW+, origin = field center");
        telemetry.addData("Pinpoint pose (x,y,h)", "(%.1f, %.1f, %.1f)", p.getX(), p.getY(), hDeg);
        telemetry.addData("Target pose  (x,y,h)", "(%.1f, %.1f, %.1f)", tx, ty, thDeg);
        telemetry.addData("Error (dx,dy,dh)", "(%.1f, %.1f, %.1f)", tx - p.getX(), ty - p.getY(), thDeg - hDeg);
        telemetry.addData("Busy", follower.isBusy());
        telemetry.update();
    }
}
