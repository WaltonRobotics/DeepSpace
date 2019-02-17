/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystem;

import static frc.robot.Config.SmartDashboardKeys.CONSTANTS_KACC;
import static frc.robot.Config.SmartDashboardKeys.CONSTANTS_KANGLE;
import static frc.robot.Config.SmartDashboardKeys.CONSTANTS_KK;
import static frc.robot.Config.SmartDashboardKeys.CONSTANTS_KL;
import static frc.robot.Config.SmartDashboardKeys.CONSTANTS_KS;
import static frc.robot.Config.SmartDashboardKeys.CONSTANTS_KV;
import static frc.robot.Config.SmartDashboardKeys.CONSTANTS_MAX_ACCELERATION;
import static frc.robot.Config.SmartDashboardKeys.CONSTANTS_MAX_VELOCITY;
import static frc.robot.Robot.currentRobot;
import static frc.robot.RobotMap.encoderLeft;
import static frc.robot.RobotMap.encoderRight;
import static frc.robot.RobotMap.leftWheel1;
import static frc.robot.RobotMap.leftWheel2;
import static frc.robot.RobotMap.rightWheel1;
import static frc.robot.RobotMap.rightWheel2;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.command.teleop.Drive;
import org.waltonrobotics.AbstractDrivetrain;
import org.waltonrobotics.controller.RobotPair;

/**
 * Add your docs here.
 */
public class Drivetrain extends AbstractDrivetrain {


  public Drivetrain() {
    super(currentRobot);

    SmartDashboard.putNumber(CONSTANTS_KV, currentRobot.getKV());
    SmartDashboard.putNumber(CONSTANTS_KACC, currentRobot.getKAcc());
    SmartDashboard.putNumber(CONSTANTS_KK, currentRobot.getKK());
    SmartDashboard.putNumber(CONSTANTS_KS, currentRobot.getKS());
    SmartDashboard.putNumber(CONSTANTS_KANGLE, currentRobot.getKAng());
    SmartDashboard.putNumber(CONSTANTS_MAX_VELOCITY, currentRobot.getMaxVelocity());
    SmartDashboard.putNumber(CONSTANTS_MAX_ACCELERATION, currentRobot.getMaxAcceleration());
    SmartDashboard.putNumber(CONSTANTS_KL, currentRobot.getKL());
  }

  @Override
  public RobotPair getWheelPositions() {
    return new RobotPair(encoderLeft.getDistance(), encoderRight.getDistance(), Timer.getFPGATimestamp());
  }

  @Override
  public void reset() {
    encoderLeft.reset();
    encoderRight.reset();
  }

  @Override
  public void setSpeeds(double leftYJoystick, double rightYJoystick) {
    SmartDashboard.putNumber("leftSpeed", leftYJoystick);
    SmartDashboard.putNumber("leftMotor1", leftWheel1.getMotorOutputPercent());
    SmartDashboard.putNumber("leftMotor2", leftWheel2.getMotorOutputPercent());
    SmartDashboard.putNumber("rightSpeed", rightYJoystick);
    SmartDashboard.putNumber("rightMotor1", rightWheel1.getMotorOutputPercent());
    SmartDashboard.putNumber("rightMotor2", rightWheel2.getMotorOutputPercent());
    leftWheel1.set(ControlMode.PercentOutput ,leftYJoystick);
    leftWheel2.set(ControlMode.PercentOutput ,leftYJoystick);
    rightWheel1.set(ControlMode.PercentOutput, rightYJoystick);
    rightWheel2.set(ControlMode.PercentOutput, rightYJoystick);
  }

  @Override
  public void setEncoderDistancePerPulse() {
    leftWheel1.setInverted(currentRobot.getLeftTalonConfig().isInverted());
    leftWheel2.setInverted(currentRobot.getLeftTalonConfig().isInverted());
    rightWheel1.setInverted(currentRobot.getRightTalonConfig().isInverted());
    rightWheel2.setInverted(currentRobot.getRightTalonConfig().isInverted());

    leftWheel1.configPeakOutputForward(1);
    leftWheel2.configPeakOutputForward(1);
    leftWheel1.configPeakOutputReverse(-1);
    leftWheel2.configPeakOutputReverse(-1);

    rightWheel1.configPeakOutputForward(1);
    rightWheel2.configPeakOutputForward(1);
    rightWheel1.configPeakOutputReverse(-1);
    rightWheel2.configPeakOutputReverse(-1);

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
    if (RobotMap.shifter.get()) {
      RobotMap.shifter.set(false);
    }
  }

  public void shiftDown() {
    if (!RobotMap.shifter.get()) {
      RobotMap.shifter.set(true);
    }
  }
}
