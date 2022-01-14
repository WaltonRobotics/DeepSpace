/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystem;

import static frc.robot.Config.RamseteControllerConstants.K_BETA;
import static frc.robot.Config.RamseteControllerConstants.K_ZETA;
import static frc.robot.Config.SmartDashboardKeys.DEBUG_HAS_VALID_CAMERA_DATA;
import static frc.robot.Config.SmartMotionConstants.*;
import static frc.robot.Robot.currentRobot;
import static frc.robot.RobotMap.*;

import com.revrobotics.CANSparkMax.IdleMode;

import com.revrobotics.*;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.command.teleop.Drive;
import frc.robot.waltonrobotics.metadata.CameraData;
import frc.robot.waltonrobotics.metadata.RobotPair;


/**
 * Add your docs here.
 */
public class Drivetrain extends SubsystemBase {

  private CameraData cameraData = new CameraData();

  private DifferentialDriveOdometry driveOdometry;
  private DifferentialDriveKinematics differentialDriveKinematics;
  private RamseteController ramseteController;

  private Pose2d robotPose;

  public Drivetrain() {

    differentialDriveKinematics = new DifferentialDriveKinematics(currentRobot.getRobotWidth());
    driveOdometry = new DifferentialDriveOdometry(getAngle());
    robotPose = new Pose2d();

    setEncoderDistancePerPulse();

    ahrs.reset();
    ahrs.zeroYaw();
    leftWheelsSlave.setIdleMode(IdleMode.kCoast);
    leftWheelsMaster.setIdleMode(IdleMode.kBrake);
    rightWheelsSlave.setIdleMode(IdleMode.kCoast);
    rightWheelsMaster.setIdleMode(IdleMode.kBrake);

    ramseteController = new RamseteController(K_BETA, K_ZETA);
  }

  @Override
  public void periodic() {
    super.periodic();

    SmartDashboard.putNumber("Dial", cameraData.getCameraPose().getY());
    SmartDashboard.putBoolean("Is Shifted Up", isShiftedUp());

    updateRobotPose();

    if (cameraData.getTime() == -1.0) {
      SmartDashboard.putBoolean(DEBUG_HAS_VALID_CAMERA_DATA, false);
    } else {
      SmartDashboard.putBoolean(DEBUG_HAS_VALID_CAMERA_DATA, true);
    }
  }

  public RobotPair getWheelPositions() {
    return new RobotPair(0, 0, 0);
  }

  public double getLeftDistanceTravelled() {
    return leftWheelsMaster.getEncoder().getPosition();
  }

  public void updateRobotPose() {
    robotPose = driveOdometry.update(getAngle(), encoderLeft.getDistance(), encoderRight.getDistance());
  }

  public void updateRobotPoseStartBackwards() {
    robotPose = driveOdometry.update(getAngle().unaryMinus(), encoderLeft.getDistance(), encoderRight.getDistance());
  }

  public Pose2d getRobotPose() {
    return robotPose;
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(encoderLeft.getRate(), encoderRight.getRate());
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(-ahrs.getYaw());
  }



  public double getAngleDegrees() {
    return getAngle().getDegrees();
  }

  public void reset() {
    encoderLeft.reset();
    encoderRight.reset();
    ahrs.zeroYaw();
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

  public void setVoltages(double leftVoltage, double rightVoltage) {
    leftWheelsMaster.getPIDController().setReference(leftVoltage, CANSparkMax.ControlType.kVelocity);
    rightWheelsMaster.getPIDController().setReference(rightVoltage, CANSparkMax.ControlType.kVelocity);
  }

  public void setVelocities(double leftVelocity, double leftFeedForward, double rightVelocity, double rightFeedForward) {
    leftWheelsMaster.getPIDController().setReference(leftVelocity, CANSparkMax.ControlType.kVelocity, VELOCITY_CONTROL_MODE, leftFeedForward);
    rightWheelsMaster.getPIDController().setReference(rightVelocity, CANSparkMax.ControlType.kVelocity, VELOCITY_CONTROL_MODE, rightFeedForward);
  }

  public void setEncoderDistancePerPulse() {

    encoderLeft.setDistancePerPulse(0.000578185267);
    encoderRight.setDistancePerPulse(0.000578185267);

    encoderLeft.setReverseDirection(false);
    encoderRight.setReverseDirection(true);

  }

  public void motorSetUpTeleop() {
    leftWheelsMaster.restoreFactoryDefaults();
    leftWheelsSlave.restoreFactoryDefaults();
    rightWheelsMaster.restoreFactoryDefaults();
    rightWheelsSlave.restoreFactoryDefaults();

    leftWheelsMaster.setInverted(true);

    leftWheelsSlave.setIdleMode(IdleMode.kCoast);
    leftWheelsMaster.setIdleMode(IdleMode.kBrake);
    rightWheelsSlave.setIdleMode(IdleMode.kCoast);
    rightWheelsMaster.setIdleMode(IdleMode.kBrake);

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
  }

  public void motorSetUpAuto() {
    leftWheelsMaster.restoreFactoryDefaults();
    leftWheelsSlave.restoreFactoryDefaults();
    rightWheelsMaster.restoreFactoryDefaults();
    rightWheelsSlave.restoreFactoryDefaults();

    leftWheelsMaster.setInverted(true);

    leftWheelsSlave.setIdleMode(IdleMode.kBrake);
    leftWheelsMaster.setIdleMode(IdleMode.kBrake);
    rightWheelsSlave.setIdleMode(IdleMode.kBrake);
    rightWheelsMaster.setIdleMode(IdleMode.kBrake);

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
  }

  public void initDefaultCommand(boolean interrupted) {
    // Set the default command for a subsystem here.
    setDefaultCommand((Command) new Drive());
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

  public boolean isShiftedUp() {
    return shifter.get();
  }

  public CameraData getCameraData() {
    return cameraData;
  }

  public DifferentialDriveOdometry getDriveOdometry() {
    return driveOdometry;
  }

  public DifferentialDriveKinematics getDriveKinematics() {
    return differentialDriveKinematics;
  }

  public RamseteController getRamseteController() {
    return ramseteController;
  }

  @Override
  public String toString() {
    return "Drivetrain{" +
        "cameraData=" + cameraData +
        "} " + super.toString();
  }
}
