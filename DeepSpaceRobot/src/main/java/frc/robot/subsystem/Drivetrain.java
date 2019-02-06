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
import static frc.robot.RobotMap.leftWheel;
import static frc.robot.RobotMap.rightWheel;

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
    SmartDashboard.putNumber(CONSTANTS_KV, currentRobot.getkV());
    SmartDashboard.putNumber(CONSTANTS_KACC, currentRobot.getkAcc());
    SmartDashboard.putNumber(CONSTANTS_KK, currentRobot.getkK());
    SmartDashboard.putNumber(CONSTANTS_KS, currentRobot.getkS());
    SmartDashboard.putNumber(CONSTANTS_KANGLE, currentRobot.getkAng());
    SmartDashboard.putNumber(CONSTANTS_MAX_VELOCITY, currentRobot.getMaxVelocity());
    SmartDashboard.putNumber(CONSTANTS_MAX_ACCELERATION, currentRobot.getMaxAcceleration());
    SmartDashboard.putNumber(CONSTANTS_KL, currentRobot.getkL());
  }

  @Override
  public RobotPair getWheelPositions() {
    return new RobotPair(encoderLeft.getDistance(), encoderRight.getDistance(), Timer.getFPGATimestamp());
  }

  @Override
  public double getRobotWidth() {
    return currentRobot.getRobotWidth();
  }

  @Override
  public double getRobotLength() {
    return currentRobot.getRobotLength();
  }

  @Override
  public void reset() {
    encoderLeft.reset();
    encoderRight.reset();
  }

  @Override
  public void setSpeeds(double leftYJoystick, double rightYJoystick) {
    SmartDashboard.putNumber("leftSpeed", leftYJoystick);
    SmartDashboard.putNumber("leftMotor", leftWheel.get());
    SmartDashboard.putNumber("rightSpeed", rightYJoystick);
    SmartDashboard.putNumber("rightMotor", rightWheel.get());

    leftWheel.set(leftYJoystick);
    rightWheel.set(rightYJoystick);
  }

  @Override
  public void setEncoderDistancePerPulse() {
    leftWheel.setInverted(currentRobot.isMotorLeftInverted());
    rightWheel.setInverted(currentRobot.isMotorRightInverted());
    encoderLeft.setDistancePerPulse(currentRobot.getDistancePerPulse());
    encoderRight.setDistancePerPulse(currentRobot.getDistancePerPulse());
    encoderLeft.setReverseDirection(currentRobot.isEncoderLeftInverted());
    encoderRight.setReverseDirection(currentRobot.isEncoderRightInverted());

  }

  @Override
  public double getKV() {
    return SmartDashboard.getNumber(CONSTANTS_KV, currentRobot.getkV());
  }

  @Override
  public double getKAcc() {
    return SmartDashboard.getNumber(CONSTANTS_KACC, currentRobot.getkAcc());
  }

  @Override
  public double getKK() {
    return SmartDashboard.getNumber(CONSTANTS_KK, currentRobot.getkK());
  }

  @Override
  public double getKS() {
    return SmartDashboard.getNumber(CONSTANTS_KS, currentRobot.getkS());
  }

  @Override
  public double getKAng() {
    return SmartDashboard.getNumber(CONSTANTS_KANGLE, currentRobot.getkAng());
  }

  @Override
  public double getKL() {
    return SmartDashboard.getNumber(CONSTANTS_KL, currentRobot.getkL());
  }

  @Override
  public double getILag() {
    return currentRobot.getiL();
  }

  @Override
  public double getIAng() {
    return currentRobot.getiAng();
  }

  @Override
  public double getMaxVelocity() {
    return SmartDashboard.getNumber(CONSTANTS_MAX_VELOCITY, currentRobot.getMaxVelocity());
  }

  @Override
  public double getMaxAcceleration() {
    return SmartDashboard.getNumber(CONSTANTS_MAX_ACCELERATION, currentRobot.getMaxAcceleration());
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
