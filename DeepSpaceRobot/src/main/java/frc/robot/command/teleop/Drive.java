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
  private boolean isAlligning = false;
  private EnhancedBoolean rightTriggerPress = new EnhancedBoolean();
  private boolean limelightHasValidTarget;
  private double limelightDriveCommand;
  private double limelightSteerCommand;
  private double deadband = 0.05;

  public Drive() {
    requires(drivetrain);
  }

  public static void setIsEnabled(boolean b) {
    enabled = b;
  }

  private Transform getTransform() {
    return Robot.transformSendableChooser.getSelected();
  }

  private double getLeftYJoystick() {
    if (Math.abs(OI.leftJoystick.getY()) > deadband) {
      return (currentRobot.getLeftJoystickConfig().isInverted() ? -1 : 1) * OI.leftJoystick.getY();
    }
    return 0;
  }

  private double getRightYJoystick() {
    if (Math.abs(OI.rightJoystick.getY()) > deadband) {
      return (currentRobot.getRightJoystickConfig().isInverted() ? -1 : 1) * OI.rightJoystick
              .getY();
    }
    return 0;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
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
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline")
            .setDouble(Camera.DRIVER_PIPELINE);
      }

      updateLimelightTracking();

      double leftYJoystick = getLeftYJoystick();
      double rightYJoystick = getRightYJoystick();

      SmartDashboard.putNumber(DRIVETRAIN_LEFT_JOYSTICK_Y, leftYJoystick);
      SmartDashboard.putNumber(DRIVETRAIN_RIGHT_JOYSTICK_Y, rightYJoystick);
      SmartDashboard.putNumber("Angle", drivetrain.getAngle().getDegrees());

      Transform transform = getTransform();
      leftYJoystick = transform.transform(leftYJoystick);
      rightYJoystick = transform.transform(rightYJoystick);

      if (rightTriggerPress.get()) {
        if (limelightHasValidTarget) {
          drivetrain.setArcadeSpeeds(limelightDriveCommand, limelightSteerCommand);
          isAlligning = true;
          LEDController.setLEDAutoAlignMode();
        } else {
          isAlligning = false;
        }
      } else if (rightTriggerPress.isFallingEdge()) {
        isAlligning = false;

      }

      if (!isAlligning || !limelightHasValidTarget) {
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
   * This function implements a simple method of generating driving and steering commands based on
   * the tracking data from a limelight camera.
   */
  public void updateLimelightTracking() {
    // These numbers must be tuned for your Robot!  Be careful!
    final double STEER_K = SmartDashboard
        .getNumber("Steer K", 0.065); // how hard to turn toward the target
    final double DRIVE_K = SmartDashboard
        .getNumber("Drive K", 0.26); // how hard to drive fwd toward the target

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
    double distance =
        0.0006083653 * ty * ty * ty + 0.0035045626 * ty * ty + 0.0310867702 * ty + 0.6929105875;
    SmartDashboard.putNumber("Camera Distance", distance);

    distance = Math.max(.5, distance);
    distance = Math.min(2.5, distance);

    double steerCmd = tx * STEER_K / distance;
    limelightSteerCommand = steerCmd;

    // try to drive forward until the target area reaches our desired area
    double driveCmd = (getLeftYJoystick() + getRightYJoystick()) / 2.0;
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
}