package frc.robot.subsystem;


import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.util.Logger;

import static frc.robot.Config.Elevator.LOWERING_TO_BASE_POWER;
import static frc.robot.Config.Elevator.LOWERING_TO_BASE_TIMEOUT_SECONDS;
import static frc.robot.OI.elevatorDownButton;
import static frc.robot.OI.elevatorUpButton;
import static frc.robot.OI.gamepad;
import static frc.robot.RobotMap.*;
import static frc.robot.RobotMap.hatchRotationMotor;

public class ElevatorCargoHatchSubsystem extends Subsystem {

  public ActiveState currentActiveState = ActiveState.ROBOT_SWITCHED_ON;

  public ActiveState getCurrentActiveState() {
    return currentActiveState;
  }

  public enum ElevatorLevel {
    UNKNOWN(0), BASE(100), LEVEL1(200), LEVEL2(300), LEVEL3(400);

    double target;

    ElevatorLevel(double target) {
      this.target = target;
    }

    public double getTarget() {
      return target;
    }
  }

  public enum ElevatorControlMode {
    AUTO, MANUAL
  }

  private Timer elevatorRuntime;

  private boolean elevatorLastUpButtonPressed;
  private boolean elevatorLastDownButtonPressed;
  private boolean elevatorCurrentUpButtonPressed;
  private boolean elevatorCurrentDownButtonPressed;

  private int elevatorCurrentEncoderPosition;

  private double elevatorCurrentPower;

  private double elevatorLoweringToBaseStartTime;
  private boolean elevatorIsLoweringToBase;
  private boolean elevatorIsAtBase;

  private ElevatorControlMode elevatorControlMode;

  private Logger elevatorLogger;

  public void intakeCargo() {

    RobotMap.leftIntakeMotor.set(1);
    RobotMap.rightIntakeMotor.set(1);

  }

  public void outTakeCargo() {

    RobotMap.leftIntakeMotor.set(-1);
    RobotMap.rightIntakeMotor.set(-1);

  }

  public void flipOutClawSystem() {
    RobotMap.clawRotationMotor.set(ControlMode.MotionMagic, 1);
  }

  public void flipInClawSystem() {
    RobotMap.clawRotationMotor.set(ControlMode.MotionMagic, -1);
  }

  public void openHatchIntake() {
    if(!hatchIntake.get()) {
      hatchIntake.set(true);
    }
  }

  public void closeHatchIntake() {
    if(hatchIntake.get()) {
      hatchIntake.set(false);
    }
  }

  public void flipOutHatchIntake() {
    hatchRotationMotor.set(ControlMode.MotionMagic, 1);
  }

  public void flipInHatchIntake() {
    hatchRotationMotor.set(ControlMode.MotionMagic, -1);
  }


  public void setCurrentActiveState(ActiveState currentActiveState) {
    this.currentActiveState = currentActiveState;
  }

  public void resetElevator() {
    elevatorRuntime = new Timer();
    elevatorRuntime.reset();

    elevatorLastUpButtonPressed = false;
    elevatorLastDownButtonPressed = false;
    elevatorCurrentUpButtonPressed = false;
    elevatorCurrentDownButtonPressed = false;

    elevatorCurrentEncoderPosition = 0;

    elevatorCurrentPower = 0.0;

    elevatorLoweringToBaseStartTime = 0.0;
    elevatorIsLoweringToBase = true;
    elevatorIsAtBase = false;

    elevatorControlMode = ElevatorControlMode.MANUAL;

    elevatorLogger = new Logger();

    zero();
  }

  public boolean isUpButtonPressed() {
    return elevatorCurrentUpButtonPressed;
  }

  public boolean isDownButtonPressed() {
    return elevatorCurrentDownButtonPressed;
  }

  public boolean wasUpButtonPressed() {
    return (elevatorCurrentUpButtonPressed != elevatorLastUpButtonPressed) && elevatorCurrentUpButtonPressed;
  }

  public boolean wasDownButtonPressed() {
    return (elevatorCurrentDownButtonPressed != elevatorLastDownButtonPressed) && elevatorCurrentDownButtonPressed;
  }

  /* Get raw height of elevator from encoder ticks. */
  public int getHeight() {
    return elevatorCurrentEncoderPosition;
  }

  public ElevatorLevel getLevel() {
    int currentHeight = getHeight();

    if (currentHeight >= ElevatorLevel.BASE.target && currentHeight < ElevatorLevel.LEVEL1.target) return ElevatorLevel.BASE;
    if (currentHeight >= ElevatorLevel.LEVEL1.target && currentHeight < ElevatorLevel.LEVEL2.target) return ElevatorLevel.LEVEL1;
    if (currentHeight >= ElevatorLevel.LEVEL2.target && currentHeight < ElevatorLevel.LEVEL3.target) return ElevatorLevel.LEVEL2;
    if (currentHeight >= ElevatorLevel.LEVEL3.target) return ElevatorLevel.LEVEL3;

    return ElevatorLevel.UNKNOWN;
  }

  public void setLevel(ElevatorLevel level) {
    elevatorMotor.set(com.ctre.phoenix.motorcontrol.ControlMode.MotionMagic, level.target);
  }

  public void setPower(double percent) {
    elevatorCurrentPower = percent;

    elevatorMotor.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, percent);
  }

  public void zero() {
    elevatorIsLoweringToBase = true;
    elevatorIsAtBase = false;
    elevatorMotor.configReverseSoftLimitEnable(false, 10);

    elevatorLoweringToBaseStartTime = elevatorRuntime.get();
  }

  public boolean isAtBase() {
    return elevatorIsAtBase;
  }

  public boolean isLoweringToBase() {
    return elevatorIsLoweringToBase;
  }

  public ElevatorControlMode getControlMode() {
    return elevatorControlMode;
  }

  public void setControlMode(ElevatorControlMode controlMode) {
    this.elevatorControlMode = controlMode;
  }

  @Override
  public void periodic() {
    collectSensorData();
    processSensorData();
  }

  private void processSensorData() {
    String logOutput = String.format("[%s]: Encoder height: %d, Current power: %f", elevatorRuntime.get(), getHeight(), elevatorCurrentPower);
    elevatorLogger.logInfo(logOutput);

    if (elevatorIsLoweringToBase) {
      setPower(LOWERING_TO_BASE_POWER);

      if (getHeight() < ElevatorLevel.BASE.target || (elevatorRuntime.get() - elevatorLoweringToBaseStartTime > LOWERING_TO_BASE_TIMEOUT_SECONDS)) {
        elevatorIsLoweringToBase = false;
        elevatorIsAtBase = true;
      }
    }
  }

  private void collectSensorData() {
    /* Read state of inputs. */
    elevatorLastUpButtonPressed = elevatorCurrentUpButtonPressed;
    elevatorCurrentUpButtonPressed = elevatorUpButton.getPressed(gamepad);

    elevatorLastDownButtonPressed = elevatorCurrentDownButtonPressed;
    elevatorCurrentDownButtonPressed = elevatorDownButton.getPressed(gamepad);

    elevatorCurrentEncoderPosition = elevatorMotor.getSelectedSensorPosition(0);
  }

  @Override
  protected void initDefaultCommand() {

  }

  public enum ActiveState {
    ROBOT_SWITCHED_ON,
    CARGO_HANDLING,
    DEFENSE,
    HATCH_HANDLING
  }
}
