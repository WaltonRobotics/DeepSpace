package frc.robot.command.auto;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.LEDController;

import static frc.robot.Robot.drivetrain;

public class VisionAlign extends Command {

    private double limelightDriveCommand;
    private double limelightSteerCommand;
    private boolean limelightHasValidTarget;
    private boolean isDone;

    private double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    private double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    private double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    private double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);


    @Override
    protected void initialize() {
        limelightDriveCommand = 0;
        limelightSteerCommand = 0;
        isDone = false;

        if (tv < 1.0) {
            limelightHasValidTarget = false;
            limelightDriveCommand = 0.0;
            limelightSteerCommand = 0.0;
            LEDController.setLEDNoTargetFoundMode();
        } else {
            limelightHasValidTarget = true;
        }
    }

    @Override
    protected void execute() {
        drivetrain.setArcadeSpeeds(limelightDriveCommand, limelightSteerCommand);
    }

    @Override
    protected boolean isFinished() {
        return !limelightHasValidTarget || isDone;
    }

    public void updateLimelightTracking() {
        final double STEER_K = SmartDashboard.getNumber("Steer K", 0.1); // how hard to turn toward the target
        final double DRIVE_K = SmartDashboard.getNumber("Drive K", 0.26); // how hard to drive fwd toward the target
        final double MAX_DRIVE = 0.2;
        final double DESIRED_TARGET_AREA = 13;


        if (tv < 1.0) {
            limelightHasValidTarget = false;
            limelightDriveCommand = 0.0;
            limelightSteerCommand = 0.0;
            LEDController.setLEDNoTargetFoundMode();
            return;
        } else {
            limelightHasValidTarget = true;
        }

        // Start with proportional steering
        double distance = 0.0006083653 * ty * ty * ty + 0.0035045626 * ty * ty + 0.0310867702 * ty + 0.6929105875;
        SmartDashboard.putNumber("Camera Distance", distance);

        distance = Math.max(.5, distance);
        distance = Math.min(2.5, distance);

        // Steer
        limelightSteerCommand = tx * STEER_K / distance;

        // try to drive forward until the target area reaches our desired area
        limelightDriveCommand = (DESIRED_TARGET_AREA - ta) * DRIVE_K;

        if (limelightDriveCommand > MAX_DRIVE) {
            limelightDriveCommand = MAX_DRIVE;
        }

        if (DESIRED_TARGET_AREA - ta <= 0) {
            isDone = true;
        }

    }
}
