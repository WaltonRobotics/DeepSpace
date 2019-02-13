/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.utils.Logger;

import static frc.robot.Config.Elevator.LOWERING_TO_BASE_POWER;
import static frc.robot.Config.Elevator.LOWERING_TO_BASE_TIMEOUT;
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

  private static final Elevator instance = new Elevator();

  private Timer runtime;

  private boolean lastUpButtonPressed;
  private boolean lastDownButtonPressed;
  private boolean currentUpButtonPressed;
  private boolean currentDownButtonPressed;

  private int currentEncoderPosition;

  private double currentPower;

  private double loweringToBaseStartTime;
  private boolean isLoweringToBase;
  private boolean isAtBase;

  private Logger elevatorLogger;

  private Elevator() {
    reset();
  }

  public void reset() {
    runtime = new Timer();
    runtime.reset();

    lastUpButtonPressed = false;
    lastDownButtonPressed = false;
    currentUpButtonPressed = false;
    currentDownButtonPressed = false;

    currentEncoderPosition = 0;

    currentPower = 0.0;

    loweringToBaseStartTime = 0.0;
    isLoweringToBase = true;
    isAtBase = false;

    elevatorLogger = new Logger();

    zero();
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

  public void setPower(double percent) {
    currentPower = percent;

    elevatorMotor.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, percent);
  }

  public void zero() {
    isLoweringToBase = true;
    isAtBase = false;
    elevatorMotor.configReverseSoftLimitEnable(false, 10);

    loweringToBaseStartTime = runtime.get();
  }

  public static Elevator getInstance() {
    return instance;
  }

  public boolean isAtBase() {
    return isAtBase;
  }

  public boolean isLoweringToBase() {
    return isLoweringToBase;
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

    elevatorLogger.logInfo("[" + Double.toString(runtime.get()) + "]: Height: " + Integer.toString(getHeight()) + ", Current Power: " + Double.toString(currentPower));

    if (isLoweringToBase) {
      setPower(LOWERING_TO_BASE_POWER);

      if (getHeight() < Level.BASE.target || (runtime.get() - loweringToBaseStartTime > LOWERING_TO_BASE_TIMEOUT)) {
        isLoweringToBase = false;
        isAtBase = true;
      }
    }
  }

}
