/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystem;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Config;
import frc.robot.command.teleop.Drive;
import lib.system.Models;
import lib.system.StateSpace;
import lib.system.System;
import org.ejml.simple.SimpleMatrix;
import org.waltonrobotics.metadata.CameraData;

import static frc.robot.Config.LQRControlConstants.MAX_OUTPUT_VOLTAGE;
import static frc.robot.Config.LQRControlConstants.MIN_OUTPUT_VOLTAGE;
import static frc.robot.Config.SmartDashboardKeys.DEBUG_HAS_VALID_CAMERA_DATA;
import static frc.robot.Config.SmartMotionConstants.*;
import static frc.robot.Robot.currentRobot;
import static frc.robot.RobotMap.*;
import static lib.system.Models.MOTOR_NEO;

/**
 * Add your docs here.
 */
public class Drivetrain extends System {

  private CameraData cameraData = new CameraData();
  private boolean inLowGear = false;

  public Drivetrain() throws Exception {
    super(new SimpleMatrix(new double[][]{{MIN_OUTPUT_VOLTAGE}, {MIN_OUTPUT_VOLTAGE}}),
            new SimpleMatrix(new double[][]{{MAX_OUTPUT_VOLTAGE}, {MAX_OUTPUT_VOLTAGE}}),
            Config.LQRControlConstants.dt, new SimpleMatrix(2, 1),
            new SimpleMatrix(2, 1));

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

    leftWheelsEncoder.setPositionConversionFactor(K_POSITION_CONVERSION_FACTOR);
    leftWheelsEncoder.setVelocityConversionFactor(K_VELOCITY_CONVERSION_FACTOR);

    rightWheelsEncoder.setPositionConversionFactor(K_POSITION_CONVERSION_FACTOR);
    rightWheelsEncoder.setVelocityConversionFactor(K_VELOCITY_CONVERSION_FACTOR);

    shiftDown();
    inLowGear = true;

    setDriveControlMode();
    setVelocityControlMode();
    setEncoderDistancePerPulse();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Dial", cameraData.getCameraPose().getY());

    if (cameraData.getTime() == -1.0) {
      SmartDashboard.putBoolean(DEBUG_HAS_VALID_CAMERA_DATA, false);
    } else {
      SmartDashboard.putBoolean(DEBUG_HAS_VALID_CAMERA_DATA, true);
    }
  }

  public void reset() {
    leftWheelsEncoder.setPosition(0.0);
    rightWheelsEncoder.setPosition(0.0);
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
    leftWheelsMaster.set(-leftPower);
    rightWheelsMaster.set(rightPower);
  }

  public void setVoltages(double leftVoltage, double rightVoltage) {
    leftWheelsMaster.getPIDController().setReference(leftVoltage, ControlType.kVoltage);
    rightWheelsMaster.getPIDController().setReference(rightVoltage, ControlType.kVoltage);
  }

  public void setEncoderDistancePerPulse() {
    encoderLeft.setDistancePerPulse(currentRobot.getLeftEncoderConfig().getDistancePerPulse());
    encoderRight.setDistancePerPulse(currentRobot.getRightEncoderConfig().getDistancePerPulse());

    encoderLeft.setReverseDirection(currentRobot.getLeftEncoderConfig().isInverted());
    encoderRight.setReverseDirection(currentRobot.getRightEncoderConfig().isInverted());
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

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new Drive());
  }


  public void shiftUp() {
    /*
    if (!shifter.get()) {
      java.lang.System.out.println("Shifted Up");
      shifter.set(true);
    }
    */
  }

  public void shiftDown() {
    /*
    if (shifter.get()) {
      java.lang.System.out.println("Shifted Down");
      shifter.set(false);
    }
    */
  }

  @Override
  public String toString() {
    return "Drivetrain{" +
        "cameraData=" + cameraData +
        "} " + super.toString();
  }

  @Override
  protected StateSpace createModel(SimpleMatrix states, SimpleMatrix inputs) throws Exception {
    return drivetrain();
  }

  @Override
  public void designControllerObserver() {
    // Have to use Matlab or Python to get LQR and Kalman gain matrix for now

    double[] lqrArray = SmartDashboard.getNumberArray("LQRMatrix", new double[]{
            7.11732344, -0.35178656,
            -0.35178656, 7.11732344,
    });

    SimpleMatrix lqrMatrix = new SimpleMatrix(
            new double[][]{
                    {lqrArray[0], lqrArray[1]},
                    {lqrArray[2], lqrArray[3]},
            }
    );

    designLQR(lqrMatrix);

    designTwoStateFeedforward(null, null);

    double qVel = 1.0;
    double rVel = 0.01;

    double[] kalmanArray = SmartDashboard.getNumberArray("KalmanMatrix", new double[]{
            9.99900017e-01, -3.10515467e-10,
            -3.10515467e-10, 9.99900017e-01,
    });

    SimpleMatrix kalmanGainMatrix = new SimpleMatrix(
            new double[][]{
                    {kalmanArray[0], kalmanArray[1]},
                    {kalmanArray[2], kalmanArray[3]},
            }
    );

    designKalmanFilter(new SimpleMatrix(new double[][]{{qVel, qVel}}), new SimpleMatrix(new double[][]{{rVel, rVel}}), kalmanGainMatrix);
  }

  private StateSpace drivetrain() throws Exception {
    Models.DcBrushedMotor gearbox = Models.gearbox(MOTOR_NEO, 2);

    double m = SmartDashboard.getNumber("Mass", 52.0);
    double r = SmartDashboard.getNumber("Wheel radius", 0.08255 / 2.0);
    double rb = SmartDashboard.getNumber("Robot radius", 0.59055 / 2.0);
    double J = SmartDashboard.getNumber("MOI", 6.0);

    double Gl = SmartDashboard.getNumber("Glow", 60.0 / 11.0);
    double Gr = SmartDashboard.getNumber("Glow", 60.0 / 11.0);

    double C1 = -Math.pow(Gl, 2) * gearbox.kT / (gearbox.kV * gearbox.R * Math.pow(r, 2));
    double C2 = Gl * gearbox.kT / (gearbox.R * r);
    double C3 = -Math.pow(Gr, 2) * gearbox.kT / (gearbox.kV * gearbox.R * Math.pow(r, 2));
    double C4 = Gr * gearbox.kT / (gearbox.R * r);

    SimpleMatrix A = new SimpleMatrix(
            new double[][]{
                    {(1 / m + Math.pow(rb, 2) / J) * C1, (1 / m - Math.pow(rb, 2) / J) * C3},
                    {(1 / m - Math.pow(rb, 2) / J) * C1, (1 / m + Math.pow(rb, 2) / J) * C3},
            }
    );

    SimpleMatrix B = new SimpleMatrix(
            new double[][]{
                    {(1 / m + Math.pow(rb, 2) / J) * C2, (1 / m - Math.pow(rb, 2) / J) * C4},
                    {(1 / m - Math.pow(rb, 2) / J) * C2, (1 / m + Math.pow(rb, 2) / J) * C4},
            }
    );

    SimpleMatrix C = new SimpleMatrix(
            new double[][]{
                    {1, 0},
                    {0, 1}
            }
    );

    SimpleMatrix D = new SimpleMatrix(
            new double[][]{
                    {0, 0},
                    {0, 0}
            }
    );

    return new StateSpace(A, B, C, D, null);
  }
}
