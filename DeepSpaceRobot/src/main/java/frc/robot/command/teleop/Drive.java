/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.command.teleop;

import static frc.robot.Config.SmartDashboardKeys.CAMERA_DATA_ACTUAL;
import static frc.robot.Config.SmartDashboardKeys.CAMERA_DATA_PROPORTIONAL_POWER;
import static frc.robot.Config.SmartDashboardKeys.CAMERA_DATA_TARGET;
import static frc.robot.Config.SmartDashboardKeys.CAMERA_DATA_TARGET_OFFSET;
import static frc.robot.Config.SmartDashboardKeys.CAMERA_DATA_USES_AUTOASSIST;
import static frc.robot.Config.SmartDashboardKeys.DEBUG_ACTUAL_TARGET;
import static frc.robot.Config.SmartDashboardKeys.DEBUG_CAMERA_OFFSET;
import static frc.robot.Config.SmartDashboardKeys.DEBUG_CHOSEN_TARGET;
import static frc.robot.Config.SmartDashboardKeys.DEBUG_JUST_BEFORE;
import static frc.robot.Config.SmartDashboardKeys.DRIVETRAIN_LEFT_JOYSTICK_Y;
import static frc.robot.Config.SmartDashboardKeys.DRIVETRAIN_RIGHT_JOYSTICK_Y;
import static frc.robot.Robot.currentRobot;
import static frc.robot.Robot.drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.command.auton.QuickAlignment;
import frc.robot.command.auton.SimpleAlignmentDrive;
import frc.robot.command.teleop.util.Transform;
import frc.robot.util.EnhancedBoolean;
import org.waltonrobotics.controller.MotionController;
import org.waltonrobotics.controller.MotionLogger;
import org.waltonrobotics.metadata.CameraData;
import org.waltonrobotics.metadata.ErrorVector;
import org.waltonrobotics.metadata.MotionData;
import org.waltonrobotics.metadata.MotionState;
import org.waltonrobotics.metadata.PathData;
import org.waltonrobotics.metadata.Pose;
import org.waltonrobotics.metadata.RobotPair;

public class Drive extends Command {

  private static final double cameraFilter = 0.5;
  private static boolean enabled = true;
  private MotionLogger motionLogger = new MotionLogger();
  private Pose offset = new Pose(0, 0, 0);
  private boolean hasFound = false;
  private EnhancedBoolean rightTriggerPress = new EnhancedBoolean();

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

    new QuickAlignment().start();

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

      double leftYJoystick = getLeftYJoystick();
      double rightYJoystick = getRightYJoystick();

      rightTriggerPress.set(OI.rightJoystick.getTrigger());

      SmartDashboard.putNumber(DRIVETRAIN_LEFT_JOYSTICK_Y, leftYJoystick);
      SmartDashboard.putNumber(DRIVETRAIN_RIGHT_JOYSTICK_Y, rightYJoystick);

      Transform transform = getTransform();
      leftYJoystick = transform.transform(leftYJoystick);
      rightYJoystick = transform.transform(rightYJoystick);

      if (rightTriggerPress.get() && !hasFound) {
        CameraData cameraData = drivetrain.getCameraData();
//      CameraData cameraData = new CameraData(
//          SmartDashboard.getNumber(CAMERA_DATA_X, 0),
//          SmartDashboard.getNumber(CAMERA_DATA_Y, 0),
//          SmartDashboard.getNumber(CAMERA_DATA_HEIGHT, 0),
//          SmartDashboard.getNumber(CAMERA_DATA_ANGLE, 0),
//          (int) SmartDashboard.getNumber(CAMERA_DATA_NUMBER_OF_TARGETS, 0),
//          SmartDashboard.getNumber(CAMERA_DATA_TIME, 0)
//      );

        if (cameraData.getNumberOfTargets() == 0) {
          hasFound = false;
        } else {
          System.out.println("Found target");
          SmartDashboard.putString(DEBUG_CHOSEN_TARGET, cameraData.toString());
          SmartDashboard.putString(DEBUG_JUST_BEFORE, drivetrain.getActualPosition().toString());
          drivetrain.setStartingPosition(cameraData.getCameraPose());
          SmartDashboard.putString(DEBUG_ACTUAL_TARGET, drivetrain.getActualPosition().toString());
          offset = new Pose();

          hasFound = true;
        }
      }

      if (rightTriggerPress.get() && hasFound) {

        CameraData cameraData = drivetrain.getCurrentCameraData();
        SimpleAlignmentDrive.fixAlignment(cameraData.getCameraPose().getAngle(), Math.atan2(cameraData.getCameraPose().getY(), cameraData.getCameraPose().getX()));
        //SimpleAlignmentDrive.forwardPowerDrive(Math.abs(cameraData.getCameraPose().getX()));
        SmartDashboard.putBoolean(CAMERA_DATA_USES_AUTOASSIST, true);
      }

      if (rightTriggerPress.isFallingEdge() && hasFound) {
        motionLogger.writeMotionDataCSV(true);
        SmartDashboard.putBoolean(CAMERA_DATA_USES_AUTOASSIST, false);
        hasFound = false;
        drivetrain.cancelControllerMotion();
      }

      drivetrain.setSpeeds(leftYJoystick, rightYJoystick);

      if (OI.shiftUp.get()) {
        drivetrain.shiftUp();
      } else if (OI.shiftDown.get()) {
        drivetrain.shiftDown();
      }
    }
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
