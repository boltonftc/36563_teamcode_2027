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
@Autonomous(name = "jacksonsauto41000", group = "Captured")
public class jacksonsauto41000 extends LinearOpMode {

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
        Pose pose1 = new Pose(35.18, -65.43, Math.toRadians(90.0));
        // CAP:POSE seq=2 kind=pose
        Pose pose2 = new Pose(61.48, -13.21, Math.toRadians(0.0));
        // CAP:POSE seq=4 kind=pose
        Pose pose3 = new Pose(53.54, -43.81, Math.toRadians(89.9));
        // CAP:POSE seq=5 kind=pose
        Pose pose4 = new Pose(-36.25, -48.11, Math.toRadians(89.9));
        // CAP:POSE seq=6 kind=pose
        Pose pose5 = new Pose(-36.25, -58.88, Math.toRadians(89.9));

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
            saveForTeleOp(follower);
            telemetry.addData("Phase", "Chain 1");
            telemetryPose(follower, 61.48, -13.21, 0.0);
        }

        // CAP:ACTION seq=3 button=X
        // ── Captured action: Button X pressed here ──
        // TODO: call your subsystem here, e.g. robot.intake.forward(); or robot.intake.reverse();
        // To shoot here, see the lesson "Actions in Autonomous: Make It Shoot".
        
        
        // CAP:ACTION-END seq=3

        follower.followPath(chain2, SPEED, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            robot.update();
            saveForTeleOp(follower);
            telemetry.addData("Phase", "Chain 2");
            telemetryPose(follower, -36.25, -58.88, 89.9);
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
