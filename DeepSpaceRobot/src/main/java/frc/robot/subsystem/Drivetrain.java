/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystem;

import static frc.robot.Config.Constants.iAng;
import static frc.robot.Config.Constants.iL;
import static frc.robot.Config.Constants.kAcc;
import static frc.robot.Config.Constants.kAng;
import static frc.robot.Config.Constants.kK;
import static frc.robot.Config.Constants.kL;
import static frc.robot.Config.Constants.kS;
import static frc.robot.Config.Constants.kV;
import static frc.robot.Config.Constants.maxAcceleration;
import static frc.robot.Config.Constants.maxVelocity;
import static frc.robot.Config.Constants.robotWidth;
import static frc.robot.Config.Hardware.DISTANCE_PER_PULSE;
import static frc.robot.Config.SmartDashboardKeys.CONSTANTS_KACC;
import static frc.robot.Config.SmartDashboardKeys.CONSTANTS_KANGLE;
import static frc.robot.Config.SmartDashboardKeys.CONSTANTS_KK;
import static frc.robot.Config.SmartDashboardKeys.CONSTANTS_KL;
import static frc.robot.Config.SmartDashboardKeys.CONSTANTS_KS;
import static frc.robot.Config.SmartDashboardKeys.CONSTANTS_KV;
import static frc.robot.Config.SmartDashboardKeys.CONSTANTS_MAX_ACCELERATION;
import static frc.robot.Config.SmartDashboardKeys.CONSTANTS_MAX_VELOCITY;
import static frc.robot.Config.SmartDashboardKeys.MAX_ACCELERATION;
import static frc.robot.Config.SmartDashboardKeys.MAX_VELOCITY;
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
    SmartDashboard.putNumber(CONSTANTS_KV, kV);
    SmartDashboard.putNumber(CONSTANTS_KACC, kAcc);
    SmartDashboard.putNumber(CONSTANTS_KK, kK);
    SmartDashboard.putNumber(CONSTANTS_KS, kS);
    SmartDashboard.putNumber(CONSTANTS_KANGLE, kAng);
    SmartDashboard.putNumber(CONSTANTS_MAX_VELOCITY, maxVelocity);
    SmartDashboard.putNumber(CONSTANTS_MAX_ACCELERATION, maxAcceleration);
    SmartDashboard.putNumber(CONSTANTS_KL, kL);
    SmartDashboard.putNumber(MAX_VELOCITY, kL);
    SmartDashboard.putNumber(MAX_ACCELERATION, kL);
  }

  @Override
  public RobotPair getWheelPositions() {
    return new RobotPair(encoderLeft.getDistance(), encoderRight.getDistance(), Timer.getFPGATimestamp());
  }

  @Override
  public double getRobotWidth() {
    return robotWidth;
  }

  @Override
  public void reset() {
    encoderLeft.reset();
    encoderRight.reset();
  }

  @Override
  public void setSpeeds(double leftYJoystick, double rightYJoystick) {
    leftWheel.set(leftYJoystick);
    rightWheel.set(rightYJoystick);
  }

  @Override
  public void setEncoderDistancePerPulse() {
    leftWheel.setInverted(true);
    encoderLeft.setDistancePerPulse(DISTANCE_PER_PULSE);
    encoderLeft.setReverseDirection(true);
    encoderRight.setDistancePerPulse(DISTANCE_PER_PULSE);

  }

  @Override
  public double getKV() {
    return SmartDashboard.getNumber(CONSTANTS_KV, kV);
  }

  @Override
  public double getKAcc() {
    return SmartDashboard.getNumber(CONSTANTS_KACC, kAcc);
  }

  @Override
  public double getKK() {
    return SmartDashboard.getNumber(CONSTANTS_KK, kK);
  }

  @Override
  public double getKS() {
    return SmartDashboard.getNumber(CONSTANTS_KS, kS);
  }

  @Override
  public double getKAng() {
    return SmartDashboard.getNumber(CONSTANTS_KANGLE, kAng);
  }

  @Override
  public double getKL() {
    return SmartDashboard.getNumber(CONSTANTS_KL, kL);
  }

  @Override
  public double getILag() {
    return iL;
  }

  @Override
  public double getIAng() {
    return iAng;
  }

  @Override
  public double getMaxVelocity() {
    return SmartDashboard.getNumber(MAX_VELOCITY, maxVelocity);
  }

  @Override
  public double getMaxAcceleration() {
    return SmartDashboard.getNumber(MAX_ACCELERATION, maxAcceleration);
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
