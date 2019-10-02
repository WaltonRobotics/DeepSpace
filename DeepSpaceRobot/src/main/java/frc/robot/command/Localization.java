package frc.robot.command;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lib.motionControl.Pose2d;

import static frc.robot.Robot.drivetrain;
import static frc.robot.RobotMap.*;

public class Localization extends Command {

    private static Localization instance;

    private static Pose2d previousPose;
    private static Pose2d currentPose;

    private static double previousDistanceLeft;
    private static double previousDistanceRight;

    public Localization() {
        requires(drivetrain);
    }

    public static void setStartingPose(Pose2d startingPose) {
        previousPose = new Pose2d(startingPose);
        previousDistanceLeft = encoderLeft.getDistance();
        previousDistanceRight = encoderRight.getDistance();
    }

    public static Localization getInstance() {
        if (instance == null) {
            instance = new Localization();
        }
        return instance;
    }

    public static Pose2d getCurrentPose() {
        return currentPose;
    }

    @Override
    protected void initialize() {
        System.out.println("Initialized localization");

        previousPose = new Pose2d();
        currentPose = new Pose2d();

        previousDistanceLeft = 0;
        previousDistanceRight = 0;
    }

    @Override
    protected void execute() {
        double currentDistanceLeft = encoderLeft.getDistance();
        double currentDistanceRight = encoderRight.getDistance();
        double dLeft = currentDistanceLeft - previousDistanceLeft;
        double dRight = currentDistanceRight - previousDistanceRight;
        double dCenter = (dLeft + dRight) / 2;

        currentPose.x = previousPose.x + dCenter * Math.cos(previousPose.theta);
        currentPose.y = previousPose.y + dCenter * Math.sin(previousPose.theta);
        currentPose.theta = ahrs.getYaw();

        SmartDashboard.putNumber("CURRENT_POSE_X", currentPose.x);
        SmartDashboard.putNumber("CURRENT_POSE_Y", currentPose.y);
        SmartDashboard.putNumber("CURRENT_POSE_THETA", currentPose.theta);

        previousPose = new Pose2d(currentPose);
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
