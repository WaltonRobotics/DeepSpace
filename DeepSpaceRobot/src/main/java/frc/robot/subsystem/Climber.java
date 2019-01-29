/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystem;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Climber extends Subsystem {
  // Put methods for controlling this Subsystem
  // here. Call these from Commands.


  private static final Climber instance = new Climber();

  private Climber() {
  }

  public static Climber getInstance() {
    return instance;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a Subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
