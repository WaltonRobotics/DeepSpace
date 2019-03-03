/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.command.teleop;

import static frc.robot.Config.SmartDashboardKeys.CAMERA_DATA_ACTUAL;
import static frc.robot.Config.SmartDashboardKeys.CAMERA_DATA_ANGLE;
import static frc.robot.Config.SmartDashboardKeys.CAMERA_DATA_HEIGHT;
import static frc.robot.Config.SmartDashboardKeys.CAMERA_DATA_NUMBER_OF_TARGETS;
import static frc.robot.Config.SmartDashboardKeys.CAMERA_DATA_TARGET;
import static frc.robot.Config.SmartDashboardKeys.CAMERA_DATA_TIME;
import static frc.robot.Config.SmartDashboardKeys.CAMERA_DATA_USES_AUTOASSIST;
import static frc.robot.Config.SmartDashboardKeys.CAMERA_DATA_X;
import static frc.robot.Config.SmartDashboardKeys.CAMERA_DATA_Y;
import static frc.robot.Config.SmartDashboardKeys.DRIVETRAIN_LEFT_JOYSTICK_Y;
import static frc.robot.Config.SmartDashboardKeys.DRIVETRAIN_RIGHT_JOYSTICK_Y;
import static frc.robot.Robot.currentRobot;
import static frc.robot.Robot.drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;
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

  public MotionLogger motionLogger = new MotionLogger();
  private boolean hasFound = false;
  private EnhancedBoolean triggerPress = new EnhancedBoolean();

  public Drive() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    System.out.println("Driving");
    requires(Robot.drivetrain);
  }

  public Transform getTransform() {
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
    double leftYJoystick = getLeftYJoystick();
    double rightYJoystick = getRightYJoystick();

    triggerPress.set(OI.rightJoystick.getTrigger());

    SmartDashboard.putNumber(DRIVETRAIN_LEFT_JOYSTICK_Y, leftYJoystick);
    SmartDashboard.putNumber(DRIVETRAIN_RIGHT_JOYSTICK_Y, rightYJoystick);

    Transform transform = getTransform();
    leftYJoystick = transform.transform(leftYJoystick);
    rightYJoystick = transform.transform(rightYJoystick);

    if (triggerPress.isRisingEdge()) {
      CameraData currentCameraData = drivetrain.getCurrentCameraData();
//      CameraData currentCameraData = new CameraData(
//          SmartDashboard.getNumber(CAMERA_DATA_X, 0),
//          SmartDashboard.getNumber(CAMERA_DATA_Y, 0),
//          SmartDashboard.getNumber(CAMERA_DATA_HEIGHT, 0),
//          SmartDashboard.getNumber(CAMERA_DATA_ANGLE, 0),
//          (int) SmartDashboard.getNumber(CAMERA_DATA_NUMBER_OF_TARGETS, 0),
//          SmartDashboard.getNumber(CAMERA_DATA_TIME, 0)
//      );
      if (currentCameraData.getNumberOfTargets() != 0) {
        drivetrain.setStartingPosition(currentCameraData.getCameraPose());
        hasFound = true;
      } else {
        hasFound = false;
      }
    }

    if (triggerPress.get() && hasFound) {
      PathData actualPathData = drivetrain.getCurrentRobotState();
      //Get cameradata and get 90 percent of it in
      PathData targetPathData = new PathData(new Pose(actualPathData.getCenterPose().getX(), 0));
      ErrorVector currentError = MotionController
          .findCurrentError(targetPathData,
              actualPathData.getCenterPose());

      double centerPower = (leftYJoystick + rightYJoystick) / 2;

      double steerPowerXTE = currentRobot.getKS() * currentError.getXTrack();
      double steerPowerAngle = currentRobot.getKAng() * currentError.getAngle();

      double steerPower = Math.max(-1.0, Math.min(1.0, steerPowerXTE + steerPowerAngle));

      centerPower = Math
          .max(-1.0 + Math.abs(steerPower),
              Math.min(1.0 - Math.abs(steerPower), centerPower));

      leftYJoystick = centerPower - steerPower;
      rightYJoystick = centerPower + steerPower;

      motionLogger.addMotionData(
          new MotionData(
              actualPathData.getCenterPose(),
              targetPathData.getCenterPose(),
              currentError,
              new RobotPair(
                  leftYJoystick,
                  rightYJoystick,
                  Timer.getFPGATimestamp()
              ),
              0,
              MotionState.MOVING));
      SmartDashboard.putString(
          CAMERA_DATA_ACTUAL, actualPathData.getCenterPose().toString());
      SmartDashboard.putString(
          CAMERA_DATA_TARGET, targetPathData.getCenterPose().toString());
      SmartDashboard.putBoolean(CAMERA_DATA_USES_AUTOASSIST, true);
    }

    if (triggerPress.isFallingEdge() && hasFound) {
      motionLogger.writeMotionDataCSV(true);
      SmartDashboard.putBoolean(CAMERA_DATA_USES_AUTOASSIST, false);
    }

    Robot.drivetrain.setSpeeds(leftYJoystick, rightYJoystick);

    if (OI.shiftUp.get()) {
      Robot.drivetrain.shiftUp();
    } else if (OI.shiftDown.get()) {
      Robot.drivetrain.shiftDown();
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
