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
import static frc.robot.RobotMap.hatchProngs;
import static frc.robot.RobotMap.hatchSensor;

/**
 * Add your docs here.
 */
public class HatchIntaker extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private boolean lastHatchLoadButtonPressed = false;
  private boolean currentHatchLoadButtonPressed = false;
  private boolean isHatchLoose = false;

  private enum ProngsPosition {
    OPEN, CLOSED
  }

  private static final HatchIntaker instance = new HatchIntaker();

  private HatchIntaker() {
  }

  public static HatchIntaker getinstance() {
    return instance;
  }

  public boolean wasHatchLoadButtonPressed() {
    return (currentHatchLoadButtonPressed != lastHatchLoadButtonPressed) && currentHatchLoadButtonPressed;
  }

  public boolean isHatchLoose() {
    return isHatchLoose;
  }

  public void setProngsPosition(ProngsPosition p) {
    if (p == ProngsPosition.OPEN) {
      hatchProngs.set(true);
    } else if (p == ProngsPosition.CLOSED) {
      hatchProngs.set(false);
    } else {
      hatchProngs.set(false);
    }
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  @Override
  public void periodic() {
    /* Read state of inputs. */
    lastHatchLoadButtonPressed = currentHatchLoadButtonPressed;
    currentHatchLoadButtonPressed = hatchLoadButton.getPressed(gamepad);

    isHatchLoose = !hatchSensor.get();

    /* Process values relevant to subsystem. */
  }

}
