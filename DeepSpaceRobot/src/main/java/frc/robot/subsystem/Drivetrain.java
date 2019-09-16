/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystem;

import static frc.robot.Config.RamseteControllerConstants.DRIVE_RADIUS;
import static frc.robot.Config.SmartMotionConstants.DRIVE_CONTROL_MODE;
import static frc.robot.Config.SmartMotionConstants.LEFT_D;
import static frc.robot.Config.SmartMotionConstants.LEFT_FF;
import static frc.robot.Config.SmartMotionConstants.LEFT_I;
import static frc.robot.Config.SmartMotionConstants.LEFT_P;
import static frc.robot.Config.SmartMotionConstants.RIGHT_D;
import static frc.robot.Config.SmartMotionConstants.RIGHT_FF;
import static frc.robot.Config.SmartMotionConstants.RIGHT_I;
import static frc.robot.Config.SmartMotionConstants.RIGHT_P;
import static frc.robot.Config.SmartMotionConstants.RPM_TO_METERS;
import static frc.robot.Config.SmartMotionConstants.VELOCITY_CONTROL_MODE;
import static frc.robot.Robot.currentRobot;
import static frc.robot.RobotMap.*;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.ControlType;

import frc.robot.command.teleop.Drive;
import lib.Geometry.Pose2d;
import lib.Geometry.Rotation2d;
import lib.Kinematics.DifferentialDriveKinematics;
import lib.Kinematics.DifferentialDriveOdometry;

import org.waltonrobotics.AbstractDrivetrain;
import org.waltonrobotics.metadata.RobotPair;


public class Drivetrain extends AbstractDrivetrain {

  private DifferentialDriveOdometry driveOdometry;
  private DifferentialDriveKinematics differentialDriveKinematics;


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

    differentialDriveKinematics = new DifferentialDriveKinematics(DRIVE_RADIUS);
    driveOdometry = new DifferentialDriveOdometry(differentialDriveKinematics);

    setDriveControlMode();
    setVelocityControlMode();
    setEncoderDistancePerPulse();
  }

  @Override
  public void periodic() {
    super.periodic();
  }

  @Override
  public RobotPair getWheelPositions() {
    return new RobotPair(0, 0, 0);
  }

  public Pose2d updateRobotPose() {
    return driveOdometry.update(encoderLeft.get(), encoderRight.get(), new Rotation2d(ahrs.getAngle()));
  }

  public Pose2d updateRobotPoseRelative(Pose2d relativePose) {
    return driveOdometry.update(encoderLeft.get(), encoderRight.get(), new Rotation2d(ahrs.getAngle())).relativeTo(relativePose);
  }

  public DifferentialDriveOdometry getDriveOdometry() {
    return driveOdometry;
  }

  public void reset() {
    encoderLeft.reset();
    encoderRight.reset();
  }

  public void setArcadeSpeeds(double xSpeed, double zRotation) {
    xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
    zRotation = Math.copySign(zRotation * zRotation, zRotation);

    double leftMotorOutput;
    double rightMotorOutput;

    xSpeed = Math
        .max(-1.0 + Math.abs(zRotation),
            Math.min(1.0 - Math.abs(zRotation), xSpeed));

    leftMotorOutput = xSpeed + zRotation;
    rightMotorOutput = xSpeed - zRotation;

    setSpeeds(leftMotorOutput, rightMotorOutput);
  }

  public void setSpeeds(double leftPower, double rightPower) {
    rightWheelsMaster.set(rightPower);
    leftWheelsMaster.set(leftPower);
  }

  public void setVelocities(double lefVelocity, double rightVelocity) {
    rightWheelsMaster.getPIDController().setReference(rightVelocity / RPM_TO_METERS, ControlType.kVelocity, VELOCITY_CONTROL_MODE);
    leftWheelsMaster.getPIDController().setReference(lefVelocity / RPM_TO_METERS, ControlType.kVelocity, VELOCITY_CONTROL_MODE);
  }

  public void setVoltages(double leftVoltage, double rightVoltage) {
    rightWheelsMaster.getPIDController().setReference(rightVoltage, ControlType.kVoltage);
    leftWheelsMaster.getPIDController().setReference(leftVoltage, ControlType.kVoltage);
  }

  public void setDriveControlMode() {
    rightWheelsMaster.getPIDController().setP(RIGHT_P, DRIVE_CONTROL_MODE);
    rightWheelsMaster.getPIDController().setI(RIGHT_I, DRIVE_CONTROL_MODE);
    rightWheelsMaster.getPIDController().setD(RIGHT_D, DRIVE_CONTROL_MODE);
    rightWheelsMaster.getPIDController().setFF(RIGHT_FF, DRIVE_CONTROL_MODE);
    rightWheelsMaster.getPIDController().setOutputRange(-1, 1, DRIVE_CONTROL_MODE);

    rightWheelsMaster.getPIDController().setSmartMotionAccelStrategy(CANPIDController.AccelStrategy.kTrapezoidal, DRIVE_CONTROL_MODE);

    leftWheelsMaster.getPIDController().setP(LEFT_P, DRIVE_CONTROL_MODE);
    leftWheelsMaster.getPIDController().setI(LEFT_I, DRIVE_CONTROL_MODE);
    leftWheelsMaster.getPIDController().setD(LEFT_D, DRIVE_CONTROL_MODE);
    leftWheelsMaster.getPIDController().setFF(LEFT_FF, DRIVE_CONTROL_MODE);
    leftWheelsMaster.getPIDController().setOutputRange(-1, 1, DRIVE_CONTROL_MODE);

    leftWheelsMaster.getPIDController().setSmartMotionAccelStrategy(CANPIDController.AccelStrategy.kTrapezoidal, DRIVE_CONTROL_MODE);
  }

  public void setVelocityControlMode() {
    rightWheelsMaster.getPIDController().setP(RIGHT_P, VELOCITY_CONTROL_MODE);
    rightWheelsMaster.getPIDController().setI(RIGHT_I, VELOCITY_CONTROL_MODE);
    rightWheelsMaster.getPIDController().setD(RIGHT_D, VELOCITY_CONTROL_MODE);
    rightWheelsMaster.getPIDController().setFF(RIGHT_FF, VELOCITY_CONTROL_MODE);
    rightWheelsMaster.getPIDController().setOutputRange(-0.7, 0.7, VELOCITY_CONTROL_MODE);

    rightWheelsMaster.getPIDController().setSmartMotionAccelStrategy(CANPIDController.AccelStrategy.kTrapezoidal, VELOCITY_CONTROL_MODE);

    leftWheelsMaster.getPIDController().setP(LEFT_P, VELOCITY_CONTROL_MODE);
    leftWheelsMaster.getPIDController().setI(LEFT_I, VELOCITY_CONTROL_MODE);
    leftWheelsMaster.getPIDController().setD(LEFT_D, VELOCITY_CONTROL_MODE);
    leftWheelsMaster.getPIDController().setFF(LEFT_FF, VELOCITY_CONTROL_MODE);
    leftWheelsMaster.getPIDController().setOutputRange(-0.7, 0.7, VELOCITY_CONTROL_MODE);

    leftWheelsMaster.getPIDController().setSmartMotionAccelStrategy(CANPIDController.AccelStrategy.kTrapezoidal, VELOCITY_CONTROL_MODE);
  }


  public void setEncoderDistancePerPulse() {

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


  @Override
  public String toString() {
    return "Drivetrain{" + "} " + super.toString();
  }
}
