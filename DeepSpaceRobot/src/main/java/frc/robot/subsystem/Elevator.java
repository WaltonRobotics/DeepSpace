/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystem;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Elevator extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  double elevatorOriginalHeight = 500; //makeshift number
  double elevatorCurrentHeight;
  double distanceBetweenOgAndCurrent = elevatorCurrentHeight - elevatorOriginalHeight;

  private static final Elevator instance = new Elevator();

  private Elevator() {


  }

  public static Elevator getInstance() {
    return instance;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  //returns elevator to original height
  public void returnNormHeight() {
    if (elevatorCurrentHeight != elevatorCurrentHeight) {
      RobotMap.elevatorMotor.set(Math.signum(elevatorCurrentHeight - elevatorOriginalHeight) * Math.abs(distanceBetweenOgAndCurrent)); //may work, not expecting to
    }
  }
}
