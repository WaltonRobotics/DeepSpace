/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.command.teleop;

import static frc.robot.Config.SmartDashboardKeys.DRIVETRAIN_LEFT_JOYSTICK_Y;
import static frc.robot.Config.SmartDashboardKeys.DRIVETRAIN_RIGHT_JOYSTICK_Y;
import static frc.robot.Robot.currentRobot;
import static frc.robot.Robot.drivetrain;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Config.Camera;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.command.teleop.util.Transform;
import frc.robot.util.EnhancedBoolean;
import frc.robot.util.LEDController;

public class Drive extends Command {

  private static boolean enabled = true;
  private final EnhancedBoolean rightTriggerPress = new EnhancedBoolean();
  private boolean isAligning = false;
  private boolean limelightHasValidTarget;
  private double limelightDriveCommand;
  private double limelightSteerCommand;

  public Drive() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(drivetrain);
  }

  public static void setIsEnabled(boolean b) {
    enabled = b;
  }

  private Transform getTransform() {
    return Robot.transformSendableChooser.getSelected();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  private double getLeftYJoystick() {
    return (currentRobot.getLeftJoystickConfig().isInverted() ? -1 : 1) * OI.leftJoystick.getY();
  }

  private double getRightYJoystick() {
    return (currentRobot.getRightJoystickConfig().isInverted() ? -1 : 1) * OI.rightJoystick.getY();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (enabled) {
      rightTriggerPress.set(OI.rightJoystick.getTrigger());

      if (rightTriggerPress.isRisingEdge()) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline")
            .setDouble(Camera.AUTO_ALIGN_PIPELINE);
      } else if (rightTriggerPress.isFallingEdge()) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setDouble(Camera.DRIVER_PIPELINE);
      }

      updateLimelightTracking();

      double leftYJoystick = getLeftYJoystick();
      double rightYJoystick = getRightYJoystick();

      SmartDashboard.putNumber(DRIVETRAIN_LEFT_JOYSTICK_Y, leftYJoystick);
      SmartDashboard.putNumber(DRIVETRAIN_RIGHT_JOYSTICK_Y, rightYJoystick);

      Transform transform = getTransform();
      leftYJoystick = transform.transform(leftYJoystick);
      rightYJoystick = transform.transform(rightYJoystick);

      if (rightTriggerPress.get()) {
        if (limelightHasValidTarget) {
          drivetrain.setArcadeSpeeds(limelightDriveCommand, limelightSteerCommand);
          isAligning = true;
          LEDController.setLEDAutoAlignMode();
        } else {
          isAligning = false;
        }
      } else if (rightTriggerPress.isFallingEdge()) {
        isAligning = false;

      }

      if (!isAligning || !limelightHasValidTarget) {
        drivetrain.setSpeeds(leftYJoystick, rightYJoystick);
      }

      if (OI.shiftUp.get()) {
        drivetrain.shiftUp();
      } else if (OI.shiftDown.get()) {
        drivetrain.shiftDown();
      }
    }
  }

  /**
   * This function implements a simple method of generating driving and steering commands based on the tracking data
   * from a limelight camera.
   */
  public void updateLimelightTracking() {
    // These numbers must be tuned for your Robot!  Be careful!
    double STEER_K = SmartDashboard.getNumber("Steer K", 0.1); // how hard to turn toward the target
    double DRIVE_K = SmartDashboard.getNumber("Drive K", 0.26); // how hard to drive fwd toward the target

    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

    if (tv < 1.0) {
      limelightHasValidTarget = false;
      limelightDriveCommand = 0.0;
      limelightSteerCommand = 0.0;
      LEDController.setLEDNoTargetFoundMode();
      return;
    } else {
      // LEDController.setLEDFoundTargetMode();
      limelightHasValidTarget = true;
    }

    // Start with proportional steering
    double distance = (0.0003645262 * ty * ty * ty) + (-0.0008723340 * ty * ty) + (0.0425549550 * ty) + 0.5546679097;
    SmartDashboard.putNumber("Camera Distance", distance);

    distance = Math.max(0.5, distance);
    distance = Math.min(2.5, distance);

    double steerCmd = (tx * STEER_K) / distance;
    limelightSteerCommand = steerCmd;

    // try to drive forward until the target area reaches our desired area
    double driveCmd = (getLeftYJoystick() + getRightYJoystick()) / 2.0;
//    double maxSpeed = 1;
//    double minSpeed = .3;
//    double driveCmd;
//
//    double decelerationDistance = 1.5;
//    double minDistance = .5;
//    double alpha = (distance - minDistance) / (decelerationDistance - minDistance);
//
//    alpha = Math.max(0, Math.min(1, alpha));
//
//    driveCmd = alpha * maxSpeed + (1 - alpha) * minSpeed;
//    driveCmd = Math.min(1, driveCmd);
//
//    SmartDashboard.putNumber("Drive Speed", driveCmd);
//    SmartDashboard.putNumber("Alpha", alpha);

    limelightDriveCommand = driveCmd;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }

  @Override
  public String toString() {
    return "Drive{" +
        "rightTriggerPress=" + rightTriggerPress +
        ", isAligning=" + isAligning +
        ", limelightHasValidTarget=" + limelightHasValidTarget +
        ", limelightDriveCommand=" + limelightDriveCommand +
        ", limelightSteerCommand=" + limelightSteerCommand +
        "} " + super.toString();
  }

}
