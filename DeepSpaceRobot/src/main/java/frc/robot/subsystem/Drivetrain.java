/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystem;

import static frc.robot.Config.RamseteControllerConstants.DRIVE_RADIUS;
import static frc.robot.Config.SmartDashboardKeys.DEBUG_HAS_VALID_CAMERA_DATA;
import static frc.robot.Config.SmartMotionConstants.*;
import static frc.robot.Robot.currentRobot;
import static frc.robot.RobotMap.*;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.command.teleop.Drive;
import lib.Geometry.Pose2d;
import lib.Geometry.Rotation2d;
import lib.Kinematics.DifferentialDriveKinematics;
import lib.Kinematics.DifferentialDriveOdometry;
import lib.Kinematics.DifferentialDriveWheelSpeeds;
import org.waltonrobotics.AbstractDrivetrain;
import org.waltonrobotics.metadata.CameraData;
import org.waltonrobotics.metadata.RobotPair;

/**
 * Add your docs here.
 */
public class Drivetrain extends AbstractDrivetrain {

  private CameraData cameraData = new CameraData();
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

    leftWheelsSlave.setIdleMode(CANSparkMax.IdleMode.kBrake);
    leftWheelsMaster.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightWheelsSlave.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightWheelsMaster.setIdleMode(CANSparkMax.IdleMode.kBrake);

    differentialDriveKinematics = new DifferentialDriveKinematics(DRIVE_RADIUS);
    driveOdometry = new DifferentialDriveOdometry(differentialDriveKinematics);

    setDriveControlMode();
    setVelocityControlMode();
    setEncoderDistancePerPulse();

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

  public Pose2d updateRobotPose() {
    return driveOdometry.update(Rotation2d.fromDegrees(ahrs.getYaw()), getWheelSpeeds()); //TODO: Check angle make sure ccw positive.
  }

  private DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(encoderLeft.getRate(), encoderRight.getRate());
  }

  public DifferentialDriveOdometry getDriveOdometry() {
    return driveOdometry;
  }

  private double calculateLeftVoltages(double velocity, double acceleration) {
    return L_KV * velocity + L_KA * acceleration + L_KS * Math.signum(velocity) + L_KP * (velocity - getWheelSpeeds().rightMetersPerSecond);
  }

  private double calculateRightVoltagesVoltages(double velocity, double acceleration) {
    return R_KV * velocity + R_KA * acceleration + R_KS * Math.signum(velocity) + R_KP * (velocity - getWheelSpeeds().leftMetersPerSecond);
  }

  public void setVoltages(double leftVelocity, double leftAcceleration, double rightVelocity, double rightAcceleration) {
    rightWheelsMaster.getPIDController().setReference(calculateRightVoltagesVoltages(rightVelocity, rightAcceleration), ControlType.kVoltage);
    leftWheelsMaster.getPIDController().setReference(calculateLeftVoltages(leftVelocity, leftAcceleration), ControlType.kVoltage);
  }

  private void setDriveControlMode() {
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

  private void setVelocityControlMode() {
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
