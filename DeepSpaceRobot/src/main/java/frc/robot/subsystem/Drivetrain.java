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
    SmartDashboard.putNumber("KV", kV);
    SmartDashboard.putNumber("KAcc", kAcc);
    SmartDashboard.putNumber("KK", kK);
    SmartDashboard.putNumber("KS", kS);
    SmartDashboard.putNumber("KAngle", kAng);
    SmartDashboard.putNumber("Max Velocity", maxVelocity);
    SmartDashboard.putNumber("Max Acceleration", maxAcceleration);
    SmartDashboard.putNumber("KL", kL);
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
    return SmartDashboard.getNumber("KV", kV);
  }

  @Override
  public double getKAcc() {
    return SmartDashboard.getNumber("KAcc", kAcc);
  }

  @Override
  public double getKK() {
    return SmartDashboard.getNumber("KK", kK);
  }

  @Override
  public double getKS() {
    return SmartDashboard.getNumber("KS", kS);
  }

  @Override
  public double getKAng() {
    return SmartDashboard.getNumber("KAngle", kAng);
  }

  @Override
  public double getKL() {
    return SmartDashboard.getNumber("KL", kL);
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
    return SmartDashboard.getNumber("Max Velocity", maxVelocity);
  }

  @Override
  public double getMaxAcceleration() {
    return SmartDashboard.getNumber("Max Acceleration", maxAcceleration);
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
