/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystem;

import static frc.robot.RobotMap.leftWheel;
import static frc.robot.RobotMap.rightWheel;

import frc.robot.command.teleop.Drive;
import frc.robot.RobotMap;
import org.waltonrobotics.AbstractDrivetrain;
import org.waltonrobotics.controller.RobotPair;

/**
 * Add your docs here.
 */
public class Drivetrain extends AbstractDrivetrain {
  // Put methods for controlling this Subsystem
  // here. Call these from Commands.


  public Drivetrain() {

  }

  @Override
  public RobotPair getWheelPositions() {
    return null;
  }

  @Override
  public double getRobotWidth() {
    return 0;
  }

  @Override
  public void reset() {

  }

  @Override
  public void setSpeeds(double leftYJoystick, double rightYJoystick) {
    leftWheel.set(leftYJoystick);
    rightWheel.set(rightYJoystick);
  }

  @Override
  public void setEncoderDistancePerPulse() {

  }

  @Override
  public double getKV() {
    return 0;
  }

  @Override
  public double getKAcc() {
    return 0;
  }

  @Override
  public double getKK() {
    return 0;
  }

  @Override
  public double getKS() {
    return 0;
  }

  @Override
  public double getKAng() {
    return 0;
  }

  @Override
  public double getKL() {
    return 0;
  }

  @Override
  public double getILag() {
    return 0;
  }

  @Override
  public double getIAng() {
    return 0;
  }

  @Override
  public double getMaxVelocity() {
    return 0;
  }

  @Override
  public double getMaxAcceleration() {
    return 0;
  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a Subsystem here.
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