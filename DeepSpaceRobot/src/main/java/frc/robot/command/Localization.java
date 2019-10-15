package frc.robot.command;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.RobotMap.*;

public class Localization extends Command {

    private static Localization instance;

    private static double previousDistanceLeft;
    private static double previousDistanceRight;

    public static double currentAngle;

    public Localization() {

    }

    public static void setStartingPose() {
        previousDistanceLeft = encoderLeft.getDistance();
        previousDistanceRight = encoderRight.getDistance();

        currentAngle = 0;
    }

    public static Localization getInstance() {
        if (instance == null) {
            instance = new Localization();
        }
        return instance;
    }

    @Override
    protected void initialize() {
        System.out.println("Initialized localization");

        previousDistanceLeft = 0;
        previousDistanceRight = 0;
        currentAngle = 0;
    }

    private double normalizeAngle(double angle) {
        double k = angle;

        while (k < 0.0) {
            k += Math.PI * 2;
        }
        while (k >= Math.PI * 2) {
            k -= Math.PI * 2;
        }

        return k;
    }

    @Override
    protected void execute() {
        double currentDistanceLeft = encoderLeft.getDistance();
        double currentDistanceRight = encoderRight.getDistance();
        double dLeft = currentDistanceLeft - previousDistanceLeft;
        double dRight = currentDistanceRight - previousDistanceRight;
        double dCenter = (dLeft + dRight) / 2;
        double phi = (dRight - dLeft) / 0.3935;

        // Option 1
        //currentPose.theta = Math.toRadians(ahrs.getYaw());

        // Option 2
        currentAngle += phi;

        currentAngle = normalizeAngle(currentAngle);

        SmartDashboard.putNumber("CURRENT_POSE_THETA", Math.toDegrees(currentAngle));

        previousDistanceLeft = currentDistanceLeft;
        previousDistanceRight = currentDistanceRight;
    }

    protected boolean isFinished() {
        return false;
    }

    protected void end() {
        System.out.println("Ended localization");
    }

    protected void interrupted() {
        end();
    }

}
