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
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.command.teleop.util.Transform;
import frc.robot.util.EnhancedBoolean;
import org.waltonrobotics.controller.MotionLogger;
import org.waltonrobotics.metadata.Pose;

public class Drive extends Command {

  private static final double cameraFilter = 0.5;
  private static boolean enabled = true;
  private MotionLogger motionLogger = new MotionLogger();
  private Pose offset = new Pose(0, 0, 0);
  private boolean hasFound = false;
  private boolean isAlligning = false;
  private EnhancedBoolean rightTriggerPress = new EnhancedBoolean();
  private boolean m_LimelightHasValidTarget;
  private double m_LimelightDriveCommand;
  private double m_LimelightSteerCommand;

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
      updateLimelightTracking();

      double leftYJoystick = getLeftYJoystick();
      double rightYJoystick = getRightYJoystick();

      rightTriggerPress.set(OI.rightJoystick.getTrigger());

      SmartDashboard.putNumber(DRIVETRAIN_LEFT_JOYSTICK_Y, leftYJoystick);
      SmartDashboard.putNumber(DRIVETRAIN_RIGHT_JOYSTICK_Y, rightYJoystick);

      Transform transform = getTransform();
      leftYJoystick = transform.transform(leftYJoystick);
      rightYJoystick = transform.transform(rightYJoystick);

      if (rightTriggerPress.get()) {

        if (m_LimelightHasValidTarget) {
          drivetrain.setAcadeSpeeds(m_LimelightDriveCommand, m_LimelightSteerCommand);
          isAlligning = true;
        } else {
          isAlligning = false;
        }
      } else if (rightTriggerPress.isFallingEdge()) {
        isAlligning = false;
      }

      if (!isAlligning) {
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
    final double STEER_K = SmartDashboard.getNumber("Steer K", 0.06); // how hard to turn toward the target
    final double DRIVE_K = SmartDashboard.getNumber("Drive K", 0.26); // how hard to drive fwd toward the target
    final double DESIRED_TARGET_AREA = 12.0;        // Area of the target when the robot reaches the wall
    final double MAX_DRIVE = 0.7;                   // Simple speed limit so we don't drive too fast

    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

    SmartDashboard.putNumber("Area", ta);
    if (tv < 1.0) {
      m_LimelightHasValidTarget = false;
      m_LimelightDriveCommand = 0.0;
      m_LimelightSteerCommand = 0.0;
      return;
    }

    m_LimelightHasValidTarget = true;

    // Start with proportional steering
    double steer_cmd = tx * STEER_K;
    m_LimelightSteerCommand = steer_cmd;

    // try to drive forward until the target area reaches our desired area
    double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;

    // don't let the robot drive too fast into the goal
    if (drive_cmd > MAX_DRIVE) {
      drive_cmd = MAX_DRIVE;
    }
    m_LimelightDriveCommand = drive_cmd;
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
