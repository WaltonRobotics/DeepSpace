/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystem;


import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotMap;
import frc.robot.command.teleop.Drive;
import org.waltonrobotics.AbstractDrivetrain;
import org.waltonrobotics.controller.RobotPair;

import static frc.robot.Config.Hardware.DISTANCE_PER_PULSE;
import static frc.robot.RobotMap.*;

/**
 * Add your docs here.
 */
public class Drivetrain extends AbstractDrivetrain {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.


  public Drivetrain() {

  }

  @Override
  public RobotPair getWheelPositions() {
    return new RobotPair(encoderLeft.getDistance(), encoderRight.getDistance(), Timer.getFPGATimestamp());
  }

  @Override
  public double getRobotWidth() {
    return 0;
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
