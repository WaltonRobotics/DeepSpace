/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystem;

import static frc.robot.Config.RamseteControllerConstants.DRIVE_RADIUS;
import static frc.robot.Config.SmartMotionConstants.DRIVE_CONTROL_MODE;
import static frc.robot.Config.SmartMotionConstants.KT_IN_MILLI;
import static frc.robot.Config.SmartMotionConstants.K_OPENLOOP_RAMP;
import static frc.robot.Config.SmartMotionConstants.K_SMART_CURRENT_LIMIT;
import static frc.robot.Config.SmartMotionConstants.K_VELOCITY_CONVERSION_FACTOR;
import static frc.robot.Config.SmartMotionConstants.K_VOLTAGE_COMPENSATION;
import static frc.robot.Config.SmartMotionConstants.LEFT_KD;
import static frc.robot.Config.SmartMotionConstants.LEFT_KFF;
import static frc.robot.Config.SmartMotionConstants.LEFT_KI;
import static frc.robot.Config.SmartMotionConstants.LEFT_KP;
import static frc.robot.Config.SmartMotionConstants.L_KA;
import static frc.robot.Config.SmartMotionConstants.L_KS;
import static frc.robot.Config.SmartMotionConstants.L_KV;
import static frc.robot.Config.SmartMotionConstants.RIGHT_KD;
import static frc.robot.Config.SmartMotionConstants.RIGHT_KFF;
import static frc.robot.Config.SmartMotionConstants.RIGHT_KI;
import static frc.robot.Config.SmartMotionConstants.RIGHT_KP;
import static frc.robot.Config.SmartMotionConstants.RPM_TO_METERS;
import static frc.robot.Config.SmartMotionConstants.R_KA;
import static frc.robot.Config.SmartMotionConstants.R_KS;
import static frc.robot.Config.SmartMotionConstants.R_KV;
import static frc.robot.Config.SmartMotionConstants.VELOCITY_CONTROL_MODE;
import static frc.robot.Robot.currentRobot;
import static frc.robot.RobotMap.*;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.ControlType;

import frc.robot.command.teleop.Drive;
import lib.Geometry.Pose2d;
import lib.Geometry.Rotation2d;
import lib.Kinematics.DifferentialDriveKinematics;
import lib.Kinematics.DifferentialDriveOdometry;
import lib.Kinematics.DifferentialDriveWheelSpeeds;

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

    leftWheelsMaster.setOpenLoopRampRate(K_OPENLOOP_RAMP);
    leftWheelsSlave.setOpenLoopRampRate(K_OPENLOOP_RAMP);
    rightWheelsMaster.setOpenLoopRampRate(K_OPENLOOP_RAMP);
    rightWheelsSlave.setOpenLoopRampRate(K_OPENLOOP_RAMP);

    leftWheelsMaster.setSmartCurrentLimit(K_SMART_CURRENT_LIMIT);
    leftWheelsSlave.setSmartCurrentLimit(K_SMART_CURRENT_LIMIT);
    rightWheelsMaster.setSmartCurrentLimit(K_SMART_CURRENT_LIMIT);
    rightWheelsSlave.setSmartCurrentLimit(K_SMART_CURRENT_LIMIT);

    leftWheelsMaster.enableVoltageCompensation(K_VOLTAGE_COMPENSATION);
    leftWheelsSlave.enableVoltageCompensation(K_VOLTAGE_COMPENSATION);
    rightWheelsMaster.enableVoltageCompensation(K_VOLTAGE_COMPENSATION);
    rightWheelsSlave.enableVoltageCompensation(K_VOLTAGE_COMPENSATION);

    leftWheelsSlave.setIdleMode(IdleMode.kBrake);
    leftWheelsMaster.setIdleMode(IdleMode.kBrake);
    rightWheelsSlave.setIdleMode(IdleMode.kBrake);
    rightWheelsMaster.setIdleMode(IdleMode.kBrake);

    leftWheelsEncoder.setMeasurementPeriod(KT_IN_MILLI);
    rightWheelsEncoder.setMeasurementPeriod(KT_IN_MILLI);

    leftWheelsEncoder.setVelocityConversionFactor(K_VELOCITY_CONVERSION_FACTOR);
    rightWheelsEncoder.setVelocityConversionFactor(K_VELOCITY_CONVERSION_FACTOR);

    leftWheelsEncoder.setInverted(true);

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
    return driveOdometry.update(Rotation2d.fromDegrees(ahrs.getYaw()), new DifferentialDriveWheelSpeeds(leftWheelsEncoder.getVelocity(), rightWheelsEncoder.getVelocity()));
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

  private double calculateLeftVoltages(double velocity, double acceleration) {
      return L_KV * velocity + L_KA * acceleration + L_KS * Math.signum(velocity);
  }

  private double calculateRightVoltagesVoltages(double velocity, double acceleration) {
    return R_KV * velocity + R_KA * acceleration + R_KS * Math.signum(velocity);
  }

  public void setVoltages(double leftVelocity, double leftAcceleration, double rightVelocity, double rightAcceleration) {
    rightWheelsMaster.getPIDController().setReference(calculateRightVoltagesVoltages(rightVelocity, rightAcceleration), ControlType.kVoltage);
  }

  public void setDriveControlMode() {
    rightWheelsMaster.getPIDController().setP(RIGHT_KP, DRIVE_CONTROL_MODE);
    rightWheelsMaster.getPIDController().setI(RIGHT_KI, DRIVE_CONTROL_MODE);
    rightWheelsMaster.getPIDController().setD(RIGHT_KD, DRIVE_CONTROL_MODE);
    rightWheelsMaster.getPIDController().setFF(RIGHT_KFF, DRIVE_CONTROL_MODE);
    rightWheelsMaster.getPIDController().setOutputRange(-1, 1, DRIVE_CONTROL_MODE);

    rightWheelsMaster.getPIDController().setSmartMotionAccelStrategy(CANPIDController.AccelStrategy.kTrapezoidal, DRIVE_CONTROL_MODE);

    leftWheelsMaster.getPIDController().setP(LEFT_KP, DRIVE_CONTROL_MODE);
    leftWheelsMaster.getPIDController().setI(LEFT_KI, DRIVE_CONTROL_MODE);
    leftWheelsMaster.getPIDController().setD(LEFT_KD, DRIVE_CONTROL_MODE);
    leftWheelsMaster.getPIDController().setFF(LEFT_KFF, DRIVE_CONTROL_MODE);
    leftWheelsMaster.getPIDController().setOutputRange(-1, 1, DRIVE_CONTROL_MODE);

    leftWheelsMaster.getPIDController().setSmartMotionAccelStrategy(CANPIDController.AccelStrategy.kTrapezoidal, DRIVE_CONTROL_MODE);
  }

  public void setVelocityControlMode() {
    rightWheelsMaster.getPIDController().setP(RIGHT_KP, VELOCITY_CONTROL_MODE);
    rightWheelsMaster.getPIDController().setI(RIGHT_KI, VELOCITY_CONTROL_MODE);
    rightWheelsMaster.getPIDController().setD(RIGHT_KD, VELOCITY_CONTROL_MODE);
    rightWheelsMaster.getPIDController().setFF(RIGHT_KFF, VELOCITY_CONTROL_MODE);
    rightWheelsMaster.getPIDController().setOutputRange(-1, 1, VELOCITY_CONTROL_MODE);

    rightWheelsMaster.getPIDController().setSmartMotionAccelStrategy(CANPIDController.AccelStrategy.kTrapezoidal, VELOCITY_CONTROL_MODE);

    leftWheelsMaster.getPIDController().setP(LEFT_KP, VELOCITY_CONTROL_MODE);
    leftWheelsMaster.getPIDController().setI(LEFT_KI, VELOCITY_CONTROL_MODE);
    leftWheelsMaster.getPIDController().setD(LEFT_KD, VELOCITY_CONTROL_MODE);
    leftWheelsMaster.getPIDController().setFF(LEFT_KFF, VELOCITY_CONTROL_MODE);
    leftWheelsMaster.getPIDController().setOutputRange(-1, 1, VELOCITY_CONTROL_MODE);

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
