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
import static frc.robot.RobotMap.elevatorMotor;

/**
 * Add your docs here.
 */
public class Elevator extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public enum Level {
    UNKNOWN(0), BASE(100), LEVEL1(200), LEVEL2(300), LEVEL3(400);

    double target;

    Level(double target) {
      this.target = target;
    }

    public double getTarget() {
      return target;
    }
  }

  public enum ControlMode {
    AUTO, POWER
  }

  private static final Elevator instance = new Elevator();

  private boolean lastUpButtonPressed = false;
  private boolean lastDownButtonPressed = false;
  private boolean currentUpButtonPressed = false;
  private boolean currentDownButtonPressed = false;

  private int currentEncoderPosition = 0;

  private ControlMode controlMode;

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

  /* Get raw height of elevator from encoder ticks. */
  public int getHeight() {
    return currentEncoderPosition;
  }

  public Level getLevel() {
    int currentHeight = getHeight();

    if (currentHeight >= Level.BASE.target && currentHeight < Level.LEVEL1.target) return Level.BASE;
    if (currentHeight >= Level.LEVEL1.target && currentHeight < Level.LEVEL2.target) return Level.LEVEL1;
    if (currentHeight >= Level.LEVEL2.target && currentHeight < Level.LEVEL3.target) return Level.LEVEL2;
    if (currentHeight >= Level.LEVEL3.target) return Level.LEVEL3;

    return Level.UNKNOWN;
  }

  public void setLevel(Level level) {
    elevatorMotor.set(com.ctre.phoenix.motorcontrol.ControlMode.MotionMagic, level.target);
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

    currentEncoderPosition = elevatorMotor.getSelectedSensorPosition(0);

    /* Process values relevant to subsystem. */
  }

}
