/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystem;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

import static frc.robot.OI.*;

/**
 * Add your docs here.
 */
public class Elevator extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static final Elevator instance = new Elevator();

  private boolean lastUpButtonPressed = false;
  private boolean lastDownButtonPressed = false;
  private boolean currentUpButtonPressed = false;
  private boolean currentDownButtonPressed = false;

  private static final int baseLevelMinimum = 0;
  private static final int levelOneMinimum = 100;
  private static final int levelTwoMinimum = 200;
  private static final int levelThreeMinimum = 300;

  private int currentEncoderPosition = 0;

  private Elevator() {

  }

  public boolean isUpButtonPressed() {
    return currentUpButtonPressed;
  }

  public boolean isDownButtonPressed() {
    return currentDownButtonPressed;
  }

  public boolean wasUpButtonPressed() {
    return (currentUpButtonPressed != lastUpButtonPressed) && currentUpButtonPressed;
  }

  public boolean wasDownButtonPressed() {
    return (currentDownButtonPressed != lastDownButtonPressed) && currentDownButtonPressed;
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
    /* Read state of inputs. */
    lastUpButtonPressed = currentUpButtonPressed;
    currentUpButtonPressed = elevatorUpButton.getPressed(gamepad);

    lastDownButtonPressed = currentDownButtonPressed;
    currentDownButtonPressed = elevatorDownButton.getPressed(gamepad);

    /* Process values relevant to subsystem. */
  }

}
