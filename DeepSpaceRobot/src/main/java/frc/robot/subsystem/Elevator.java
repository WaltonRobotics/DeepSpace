/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystem;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

import static frc.robot.OI.elevatorDownButton;
import static frc.robot.OI.elevatorUpButton;
import static frc.robot.OI.gamepad;

/**
 * Add your docs here.
 */
public class Elevator extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static final Elevator instance = new Elevator();

  private boolean lastUpButtonState = false;
  private boolean lastDownButtonState = false;

  private Elevator() {


  }

  public boolean isUpButtonPressed() {
    return elevatorUpButton.getPressed(gamepad);
  }

  public boolean isDownButtonPressed() {
    return elevatorDownButton.getPressed(gamepad);
  }

  public boolean wasUpButtonPressed() {
    boolean currentValue = isUpButtonPressed();
    return (currentValue != lastUpButtonState) && currentValue;
  }

  public boolean wasDownButtonPressed() {
    boolean currentValue = isDownButtonPressed();
    return (currentValue != lastDownButtonState) && currentValue;
  }

  public static Elevator getInstance() {
    return instance;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  @Override
  public void periodic() {
    /* Do logic. */

    lastUpButtonState = isUpButtonPressed();
    lastDownButtonState = isDownButtonPressed();
  }

}
