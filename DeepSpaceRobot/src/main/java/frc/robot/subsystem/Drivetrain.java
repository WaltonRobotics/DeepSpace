/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystem;

import static frc.robot.Config.RamseteControllerConstants.TRACK_WIDTH;
import static frc.robot.Config.RamseteControllerConstants.K_BETA;
import static frc.robot.Config.RamseteControllerConstants.K_ZETA;
import static frc.robot.Config.SmartDashboardKeys.DEBUG_HAS_VALID_CAMERA_DATA;
import static frc.robot.Config.SmartMotionConstants.*;
import static frc.robot.Robot.currentRobot;
import static frc.robot.RobotMap.*;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.command.teleop.Drive;
import lib.Controller.RamseteController;
import lib.Geometry.Pose2d;
import lib.Geometry.Rotation2d;
import lib.Kinematics.DifferentialDriveKinematics;
import lib.Kinematics.DifferentialDriveOdometry;
import lib.Kinematics.DifferentialDriveWheelSpeeds;
import lib.Utils.PIDController;

import org.waltonrobotics.AbstractDrivetrain;
import org.waltonrobotics.metadata.CameraData;
import org.waltonrobotics.metadata.RobotPair;

/**
 * Add your docs here.
 */
public class Drivetrain extends AbstractDrivetrain {

  private CameraData cameraData = new CameraData();

  public DifferentialDriveOdometry driveOdometry;
  public DifferentialDriveKinematics differentialDriveKinematics;
  public RamseteController ramseteController;

  public final PIDController m_leftPIDController = new PIDController(1, 0, 0);
  public final PIDController m_rightPIDController = new PIDController(1, 0, 0);


  public Drivetrain() {
    super(currentRobot);

    differentialDriveKinematics = new DifferentialDriveKinematics(TRACK_WIDTH);
    driveOdometry = new DifferentialDriveOdometry(differentialDriveKinematics);

    motorSetUp();
    setEncoderDistancePerPulse();

    ahrs.reset();
    ahrs.zeroYaw();

    ramseteController = new RamseteController(K_BETA, K_ZETA);
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
    return driveOdometry.update(getAngle(), getWheelSpeeds());
  }

  public Pose2d updateRobotPoseOffset() {
    return driveOdometry.update(getAngle().plus(Rotation2d.fromDegrees(180)), getWheelSpeeds());
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
    leftWheelsMaster.getPIDController().setReference(leftVoltage, ControlType.kVoltage);
    rightWheelsMaster.getPIDController().setReference(rightVoltage, ControlType.kVoltage);
  }

  public void setEncoderDistancePerPulse() {

    encoderLeft.setDistancePerPulse(0.000578185267);
    encoderRight.setDistancePerPulse(0.000578185267);

    encoderLeft.setReverseDirection(false);
    encoderRight.setReverseDirection(true);

  }

  public void motorSetUp() {
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
