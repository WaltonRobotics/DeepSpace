/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystem;

import static frc.robot.Config.SmartDashboardKeys.DEBUG_HAS_VALID_CAMERA_DATA;
import static frc.robot.Robot.currentRobot;
import static frc.robot.RobotMap.*;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    leftWheelsMaster.restoreFactoryDefaults();
    leftWheelsSlave.restoreFactoryDefaults();
    rightWheelsMaster.restoreFactoryDefaults();
    rightWheelsSlave.restoreFactoryDefaults();

    leftWheelsMaster.setInverted(true);

    leftWheelsSlave.follow(leftWheelsMaster);
    rightWheelsSlave.follow(rightWheelsMaster);

    leftWheelsMaster.setOpenLoopRampRate(0);
    leftWheelsSlave.setOpenLoopRampRate(0);
    rightWheelsMaster.setOpenLoopRampRate(0);
    rightWheelsSlave.setOpenLoopRampRate(0);

    leftWheelsMaster.setSmartCurrentLimit(40);
    leftWheelsSlave.setSmartCurrentLimit(40);
    rightWheelsMaster.setSmartCurrentLimit(40);
    rightWheelsSlave.setSmartCurrentLimit(40);

    leftWheelsSlave.setIdleMode(IdleMode.kBrake);
    leftWheelsMaster.setIdleMode(IdleMode.kBrake);
    rightWheelsSlave.setIdleMode(IdleMode.kBrake);
    rightWheelsMaster.setIdleMode(IdleMode.kBrake);

  }

  @Override
  public void periodic() {
    super.periodic();

    SmartDashboard.putNumber("Dial", cameraData.getCameraPose().getY());

    if (cameraData.getTime() == -1.0) {
      SmartDashboard.putBoolean(DEBUG_HAS_VALID_CAMERA_DATA, false);
    } else {
      SmartDashboard.putBoolean(DEBUG_HAS_VALID_CAMERA_DATA, true);
    }
  }

  @Override
  public RobotPair getWheelPositions() {
    return new RobotPair(0, 0, 0);
  }

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

  public void setSpeeds(double leftPower, double rightPower) {
    rightWheelsMaster.set(rightPower);
    // rightWheelsSlave.set(rightPower);
    leftWheelsMaster.set(leftPower);
    // leftWheelsSlave.set(leftPower);
  }

  public void setEncoderDistancePerPulse() {
//    leftWheels.setInverted(currentRobot.getLeftTalonConfig().isInverted());
//    rightWheels.setInverted(currentRobot.getRightTalonConfig().isInverted());
//
//    leftWheels.configPeakOutputForward(1.0);
//    leftWheels.configPeakOutputReverse(-1.0);
//
//    leftWheels.setNeutralMode(NeutralMode.Brake);
//    rightWheels.setNeutralMode(NeutralMode.Brake);
//
//    rightWheels.configPeakOutputForward(1.0);
//    rightWheels.configPeakOutputReverse(-1.0);

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
    if (!shifter.get()) {
      System.out.println("Shifted Up");
      shifter.set(true);
    }
  }

  public void shiftDown() {
    if (shifter.get()) {
      System.out.println("Shifted Down");
      shifter.set(false);
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
