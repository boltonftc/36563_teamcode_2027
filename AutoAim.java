package org.firstinspires.ftc.teamcode;

/*
 * AutoAim -- keeps the turret pointed at a hive cell, using where odometry says we are.
 *
 * Field coordinates (seen from the RED drivers): (0, 0) is the field center, +X is right,
 * +Y is away from the drivers, distances in inches. Angles: 0 = +X, counter-clockwise is +.
 *
 *     target cell  = the one on the same side of the field as the robot
 *     turret angle = field angle to the cell - robot heading + trim
 */
public class AutoAim {

    // Where the red hive's two cells are, in field inches. MEASURE these on your field!
    private static final double LEFT_CELL_X  = -11.4;
    private static final double RIGHT_CELL_X =  10.2;
    private static final double CELL_Y       = -12.75;

    private static final double DEAD_ZONE  = 6.0;    // inches each side of x = 0 where we keep the current cell
    private static final double TRIM_MAX   = 20.0;   // the driver's trim can shift the aim at most this far
    private static final double TRIM_SPEED = 0.5;    // degrees of trim per loop with the stick pushed all the way

    private boolean aimingRight = true;              // the right cell is raised at the start of the match
    private double trimDeg = 0.0;
    private String status = "position unknown";

    /** Shift the aim with a stick: push right to aim further right. The trim stays where you leave it. */
    public void trim(double stick) {
        trimDeg -= stick * TRIM_SPEED;                              // right on the stick = clockwise = minus
        trimDeg = Math.max(-TRIM_MAX, Math.min(TRIM_MAX, trimDeg));
    }

    /** Call every loop: picks the cell, works out the turret angle, and aims. */
    public void aim(Localization localization, Turret turret) {
        if (!turret.hasTurret()) {
            status = "no turret on this robot";
            return;
        }
        // Don't guess! Until we know where we are, point straight ahead and wait.
        if (!localization.isPoseKnown()) {
            turret.setAngle(0);
            status = "position unknown: park at HOME and hold RB";
            return;
        }

        double robotX = localization.getXInches();
        double robotY = localization.getYInches();

        // 0. Which cell? The one on our side. Inside the dead zone, keep the one we had.
        if (robotX > DEAD_ZONE)  aimingRight = true;
        if (robotX < -DEAD_ZONE) aimingRight = false;
        double targetX = aimingRight ? RIGHT_CELL_X : LEFT_CELL_X;
        double targetY = CELL_Y;

        // 1. The arrow from the robot to the cell: how far over, how far up.
        double dx = targetX - robotX;
        double dy = targetY - robotY;

        // 2. Which way does that arrow point on the field?
        double fieldAngle = Math.toDegrees(Math.atan2(dy, dx));

        // 3. Measure it from the robot's front instead, and add the driver's trim.
        double turretAngle = fieldAngle - localization.getHeadingDegrees() + trimDeg;

        // 4. Keep it between -180 and 180 (340 deg and -20 deg point the same way).
        turretAngle = wrapDegrees(turretAngle);

        // 5. Near the back there are TWO ways to point the same direction (like +190 or -170).
        //    Use the other way if the turret can reach it AND it's a shorter swing from where we are now.
        double otherWay = (turretAngle > 0) ? turretAngle - 360 : turretAngle + 360;
        double now = turret.getAngle();
        if (Math.abs(otherWay) <= Turret.MAX_ANGLE
                && Math.abs(otherWay - now) < Math.abs(turretAngle - now)) {
            turretAngle = otherWay;
        }

        // 6. Still out of reach? The cell is behind us, past the turret's limits in BOTH directions.
        //    Hold where we are -- otherwise a tiny wiggle flips +179 / -179 and the turret slams
        //    from one limit to the other. It picks the cell back up as soon as it's in range.
        if (Math.abs(turretAngle) > Turret.MAX_ANGLE) {
            turretAngle = turret.getAngle();   // "aim" at where we already are = don't move
        }

        status = String.format("%s cell, turret %.0f deg", aimingRight ? "RIGHT" : "LEFT", turretAngle);
        turret.setAngle(turretAngle);
    }

    private static double wrapDegrees(double degrees) {
        while (degrees > 180)   degrees -= 360;
        while (degrees <= -180) degrees += 360;
        return degrees;
    }

    public String getStatus() { return status; }
    public double getTrim()   { return trimDeg; }
}