/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystem;

import edu.wpi.first.wpilibj.command.Subsystem;

import static frc.robot.OI.gamepad;
import static frc.robot.OI.hatchLoadButton;

/**
 * Add your docs here.
 */
public class HatchIntaker extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private boolean lastHatchLoadButtonState = false;

  private static final HatchIntaker instance = new HatchIntaker();

  private HatchIntaker() {
  }

  public static HatchIntaker getHinstance() {
    return instance;
  }

  public boolean wasHatchLoadButtonPressed() {
    boolean currentValue = hatchLoadButton.getPressed(gamepad);
    return (currentValue != lastHatchLoadButtonState) && currentValue;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  @Override
  public void periodic() {
    /* Do logic. */

    lastHatchLoadButtonState = hatchLoadButton.getPressed(gamepad);
  }

}
