/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystem;

import static frc.robot.Config.SmartDashboardKeys.DEBUG_HAS_VALID_CAMERA_DATA;
import static frc.robot.Config.SmartDashboardKeys.DRIVETRAIN_LEFT_MOTOR_PERCENT_OUTPUT;
import static frc.robot.Config.SmartDashboardKeys.DRIVETRAIN_RIGHT_MOTOR_PERCENT_OUTPUT;
import static frc.robot.Robot.currentRobot;
import static frc.robot.Robot.drivetrain;
import static frc.robot.RobotMap.encoderLeft;
import static frc.robot.RobotMap.encoderRight;
import static frc.robot.RobotMap.leftWheels;
import static frc.robot.RobotMap.rightWheels;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.command.teleop.Drive;
import org.waltonrobotics.AbstractDrivetrain;
import org.waltonrobotics.metadata.CameraData;
import org.waltonrobotics.metadata.RobotPair;

/**
 * Add your docs here.
 */
public class Drivetrain extends AbstractDrivetrain {


  private CameraData cameraData = new CameraData();

  public Drivetrain() {
    super(currentRobot);
  }

  @Override
  public RobotPair getWheelPositions() {
    return new RobotPair(encoderLeft.getDistance(), encoderRight.getDistance(), Timer.getFPGATimestamp());
  }

  @Override
  public void periodic() {
    super.periodic();

    cameraData = drivetrain.getCurrentCameraData();
    SmartDashboard.putNumber("Dial", cameraData.getCameraPose().getY());

    if (cameraData.getTime() == -1.0) {
      SmartDashboard.putBoolean(DEBUG_HAS_VALID_CAMERA_DATA, false);
    } else {
      SmartDashboard.putBoolean(DEBUG_HAS_VALID_CAMERA_DATA, true);
    }
  }

  @Override
  public void reset() {
    encoderLeft.reset();
    encoderRight.reset();
  }

  public void setArcadeSpeeds(double xSpeed, double zRotation) {
    xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
    zRotation = Math.copySign(zRotation * zRotation, zRotation);

//    double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);
    double leftMotorOutput;
    double rightMotorOutput;

//    if (xSpeed >= 0.0D) {
//      if (zRotation >= 0.0D) {
//        leftMotorOutput = maxInput;
//        rightMotorOutput = xSpeed - zRotation;
//      } else {
//        leftMotorOutput = xSpeed + zRotation;
//        rightMotorOutput = maxInput;
//      }
//    } else if (zRotation >= 0.0D) {
//      leftMotorOutput = xSpeed + zRotation;
//      rightMotorOutput = maxInput;
//    } else {
//      leftMotorOutput = maxInput;
//      rightMotorOutput = xSpeed - zRotation;
//    }

    xSpeed = Math
        .max(-1.0 + Math.abs(zRotation),
            Math.min(1.0 - Math.abs(zRotation), xSpeed));

    leftMotorOutput = xSpeed + zRotation;
    rightMotorOutput = xSpeed - zRotation;

    setSpeeds(leftMotorOutput, rightMotorOutput);
  }

  @Override
  public void setSpeeds(double leftPower, double rightPower) {
    SmartDashboard.putNumber(DRIVETRAIN_LEFT_MOTOR_PERCENT_OUTPUT, leftWheels.getMotorOutputPercent());
    SmartDashboard.putNumber(DRIVETRAIN_RIGHT_MOTOR_PERCENT_OUTPUT, rightWheels.getMotorOutputPercent());

    leftWheels.set(ControlMode.PercentOutput, leftPower);
    rightWheels.set(ControlMode.PercentOutput, rightPower);
  }

  @Override
  public void setEncoderDistancePerPulse() {
    leftWheels.setInverted(currentRobot.getLeftTalonConfig().isInverted());
    rightWheels.setInverted(currentRobot.getRightTalonConfig().isInverted());

    leftWheels.configPeakOutputForward(1.0);
    leftWheels.configPeakOutputReverse(-1.0);

    leftWheels.setNeutralMode(NeutralMode.Brake);
    rightWheels.setNeutralMode(NeutralMode.Brake);

    rightWheels.configPeakOutputForward(1.0);
    rightWheels.configPeakOutputReverse(-1.0);

    encoderLeft.setDistancePerPulse(currentRobot.getLeftEncoderConfig().getDistancePerPulse());
    encoderRight.setDistancePerPulse(currentRobot.getRightEncoderConfig().getDistancePerPulse());

    encoderLeft.setReverseDirection(currentRobot.getLeftEncoderConfig().isInverted());
    encoderRight.setReverseDirection(currentRobot.getRightEncoderConfig().isInverted());

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new Drive());
  }


  public void shiftUp() {
    if (!RobotMap.shifter.get()) {
      System.out.println("Shifted Up");
      RobotMap.shifter.set(true);
    }
  }

  public void shiftDown() {
    if (RobotMap.shifter.get()) {
      System.out.println("Shifted Down");
      RobotMap.shifter.set(false);
    }
  }

  public CameraData getCameraData() {
    return cameraData;
  }

  @Override
  public String toString() {
    return "Drivetrain{" +
        "cameraData=" + cameraData +
        "} " + super.toString();
  }
}
