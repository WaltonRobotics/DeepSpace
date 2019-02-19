package frc.robot.subsystem;


import static frc.robot.Config.SmartDashboardKeys.MOTORS_CARGO_ANGLE;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_CARGO_POWER;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_CARGO_TARGET;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_CLAW_ForwardSoftLimit;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_CLAW_ReverseSoftLimit;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_ELEVATOR_ForwardSoftLimit;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_ELEVATOR_HEIGHT;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_ELEVATOR_POWER;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_ELEVATOR_ReverseSoftLimit;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_ELEVATOR_TARGET;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_HATCH_ANGLE;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_HATCH_ForwardSoftLimit;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_HATCH_POWER;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_HATCH_ReverseSoftLimit;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_HATCH_TARGET;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_LOWER_LIMIT;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_STATE;
import static frc.robot.OI.cargoModeButton;
import static frc.robot.OI.defenseModeButton;
import static frc.robot.OI.elevatorLevel1Button;
import static frc.robot.OI.elevatorLevel2Button;
import static frc.robot.OI.elevatorLevel3Button;
import static frc.robot.OI.elevatorZeroButton;
import static frc.robot.OI.gamepad;
import static frc.robot.OI.hatchIntakeButton;
import static frc.robot.OI.hatchModeButton;
import static frc.robot.OI.intakeCargoButton;
import static frc.robot.OI.outtakeCargoButtonFast;
import static frc.robot.OI.outtakeCargoButtonSlow;
import static frc.robot.Robot.currentRobot;
import static frc.robot.RobotMap.clawRotationMotor;
import static frc.robot.RobotMap.elevatorLowerLimit;
import static frc.robot.RobotMap.elevatorMotor;
import static frc.robot.RobotMap.hatchIntake;
import static frc.robot.RobotMap.hatchRotationMotor;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.robotState.Disabled;
import frc.robot.state.StateBuilder;
import frc.robot.util.Logger;

public class ElevatorCargoHatchSubsystem extends Subsystem {

  private final Elevator elevator = new Elevator();
  private final Cargo cargo = new Cargo();
  private final Hatch hatch = new Hatch();
  private ActiveState currentActiveState = ActiveState.ROBOT_SWITCHED_ON;
  private long currentTime = 0;
  private boolean lastDefenceModePressed;
  private boolean currentDefenceModePressed;
  private boolean lastCargoModePressed;
  private boolean currentCargoModePressed;
  private boolean lastHatchModePressed;
  private boolean currentHatchModePressed;
  private boolean isEnabled = false;
  private StateBuilder stateMachine;


  public ElevatorCargoHatchSubsystem() {
    elevator.initialize();
    cargo.initialize();
    hatch.initialize();
    // Set sense of encoder
    // Set sense of motors
    // Set soft targets on
    // Configure elevator encoder

  }

  public boolean isEnabled() {
    return isEnabled;
  }

  public void setEnabled(boolean enabled) {
    isEnabled = enabled;
  }

  public Elevator getElevator() {
    return elevator;
  }

  public Cargo getCargo() {
    return cargo;
  }

  // Inputs

  // Output

  // ???

  public Hatch getHatch() {
    return hatch;
  }

  public ActiveState getCurrentActiveState() {
    return currentActiveState;
  }

  public void setCurrentActiveState(ActiveState currentActiveState) {
    this.currentActiveState = currentActiveState;
  }

  @Override
  public void periodic() {
    if (stateMachine == null) {
      stateMachine = new StateBuilder(new Disabled());
    }

    collectSensorData();
    processSensorData();
    output();

  }

  private void collectSensorData() {
    /* Read state of inputs. */
    currentTime = System.currentTimeMillis();
    elevator.collectData();
    cargo.collectData();
    hatch.collectData();

    lastCargoModePressed = currentCargoModePressed;
    currentCargoModePressed = cargoModeButton.get();
    lastHatchModePressed = currentHatchModePressed;
    currentHatchModePressed = hatchModeButton.get();
    lastDefenceModePressed = currentDefenceModePressed;
    currentDefenceModePressed = defenseModeButton.get();
  }

  public boolean cargoModeRising() {
    return currentCargoModePressed && !lastCargoModePressed;
  }

  public boolean hatchModeRising() {
    return currentHatchModePressed && !lastHatchModePressed;
  }

  public boolean defenceModeRising() {
    return currentDefenceModePressed && !lastDefenceModePressed;
  }

  private void processSensorData() {
    stateMachine.step();
  }

  private void output() {
    // Here's where we actually set the motors etc...

    elevator.outputData();
    cargo.outputData();
    hatch.outputData();

    SmartDashboard.putString(MOTORS_STATE, stateMachine.getCurrentState().getClass().getSimpleName());
    SmartDashboard.putNumber(MOTORS_ELEVATOR_HEIGHT, getElevator().getElevatorHeight());
    SmartDashboard.putNumber(MOTORS_ELEVATOR_POWER, getElevator().getElevatorPower());
    SmartDashboard.putNumber(MOTORS_ELEVATOR_TARGET, getElevator().getElevatorCurrentTarget());

    SmartDashboard.putNumber(MOTORS_HATCH_ANGLE, getHatch().getAngle());
    SmartDashboard.putNumber(MOTORS_HATCH_POWER, getHatch().getHatchRotationPower());
    SmartDashboard.putNumber(MOTORS_HATCH_TARGET, getHatch().getHatchTarget());

    SmartDashboard.putNumber(MOTORS_CARGO_ANGLE, getCargo().getAngle());
    SmartDashboard.putNumber(MOTORS_CARGO_POWER, getCargo().getClawRotationPower());
    SmartDashboard.putNumber(MOTORS_CARGO_TARGET, getCargo().getClawTarget());

    SmartDashboard.putBoolean(MOTORS_LOWER_LIMIT, getElevator().isLowerLimit());

    Faults faults = new Faults();
    elevatorMotor.getFaults(faults);
    SmartDashboard.putBoolean(MOTORS_ELEVATOR_ForwardSoftLimit, faults.ForwardSoftLimit);
    SmartDashboard.putBoolean(MOTORS_ELEVATOR_ReverseSoftLimit, faults.ReverseSoftLimit);

    clawRotationMotor.getFaults(faults);
    SmartDashboard.putBoolean(MOTORS_CLAW_ForwardSoftLimit, faults.ForwardSoftLimit);
    SmartDashboard.putBoolean(MOTORS_CLAW_ReverseSoftLimit, faults.ReverseSoftLimit);

    hatchRotationMotor.getFaults(faults);
    SmartDashboard.putBoolean(MOTORS_HATCH_ForwardSoftLimit, faults.ForwardSoftLimit);
    SmartDashboard.putBoolean(MOTORS_HATCH_ReverseSoftLimit, faults.ReverseSoftLimit);
  }

  @Override
  protected void initDefaultCommand() {

  }

  /**
   * @return if the getAngle() value is in the enumerated range above the hatch position will be returned
   */

  public HatchPosition findHatchClosestPosition(HatchPosition hatchPosition, int angle) {
    if (currentRobot.getTarget(HatchPosition.DEPLOY).inRange(angle)) {
      return HatchPosition.DEPLOY;
    } else if (currentRobot.getTarget(HatchPosition.SAFE).inRange(angle)) {
      return HatchPosition.SAFE;
    } else if (currentRobot.getTarget(HatchPosition.HATCH_START).inRange(angle)) {
      return HatchPosition.HATCH_START;
    } else {
      return HatchPosition.CARGO_START;
    }
  }

  public enum ElevatorLevel {
    UNKNOWN, BASE, CARGO1, HATCH1, CARGO2, HATCH2, CARGO3, HATCH3
  }

  public enum ElevatorControlMode {
    DISABLED, ZEROING, AUTO, MANUAL
  }

  public enum ClawControlMode {
    DISABLED, AUTO, MANUAL
  }

  public enum HatchControlMode {
    DISABLED, AUTO, MANUAL
  }

  public enum ActiveState {
    ROBOT_SWITCHED_ON,
    CARGO_HANDLING,
    DEFENSE,
    HATCH_HANDLING
  }

  public enum CargoPosition {

    DEPLOY,
    SAFE
  }

  public enum HatchPosition {
    DEPLOY,
    SAFE,
    HATCH_START,
    CARGO_START
  }

  public class Elevator implements SubSubsystem {

    private final Logger elevatorLogger;
    // Inputs
    private boolean lastLevel3ButtonPressed;
    private boolean currentLevel3ButtonPressed;
    private boolean lastLevel2ButtonPressed;
    private boolean currentLevel2ButtonPressed;
    private boolean lastLevel1ButtonPressed;
    private boolean currentLevel1ButtonPressed;
    private boolean lowerLimit;
    private boolean isZeroed;
    private boolean resetLimits = false;
    private boolean releaseLower = false;
    private double elevatorJoystick;
    private boolean baseIsPressed;
    private int currentEncoderPosition;
    private ElevatorLevel limits = ElevatorLevel.BASE;
    // Output
    private double elevatorCurrentPower;
    private double elevatorCurrentTarget;
    private ElevatorControlMode elevatorControlMode;

    public Elevator() {
      elevatorLogger = new Logger();
    }

    public double getElevatorCurrentTarget() {
      return elevatorCurrentTarget;
    }

    public boolean isZeroed() {
      return isZeroed;
    }

    public void setZeroed(boolean zeroed) {
      isZeroed = zeroed;
    }

    public boolean isLowerLimit() {
      return lowerLimit;
    }

    @Override
    public void collectData() {
      lastLevel3ButtonPressed = currentLevel3ButtonPressed;
      currentLevel3ButtonPressed = elevatorLevel3Button.get();

      lastLevel2ButtonPressed = currentLevel2ButtonPressed;
      currentLevel2ButtonPressed = elevatorLevel2Button.get();

      lastLevel1ButtonPressed = currentLevel1ButtonPressed;
      currentLevel1ButtonPressed = elevatorLevel1Button.get();

      elevatorJoystick = -gamepad.getRightY();
      currentEncoderPosition = elevatorMotor.getSelectedSensorPosition(0);

      lowerLimit = !elevatorLowerLimit.get();
      baseIsPressed = elevatorZeroButton.get();
    }

    @Override
    public void outputData() {
//      String logOutput = String
//          .format("[%s]: Encoder height: %d, Current height target: %f, Current power: %f", currentTime,
//              getElevatorHeight(), elevatorCurrentTarget, getElevatorPower());
//      elevatorLogger.logInfo(logOutput);

      if (resetLimits) {
        currentRobot.setElevatorLimit(elevatorMotor, limits);
        resetLimits = false;
      }

      if (releaseLower) {
        elevatorMotor.configReverseSoftLimitEnable(false);
      }

      switch (elevatorControlMode) {
        case ZEROING:
          elevatorMotor.set(ControlMode.PercentOutput, -0.1);
          elevatorMotor.setSelectedSensorPosition(0);
          elevatorCurrentTarget = 0;
          break;
        case AUTO:
          elevatorMotor.set(ControlMode.MotionMagic, elevatorCurrentTarget);
          break;
        case MANUAL:
          elevatorMotor.set(ControlMode.PercentOutput, elevatorCurrentPower);
          elevatorCurrentTarget = currentEncoderPosition;
          break;
        case DISABLED:
          elevatorMotor.set(ControlMode.Disabled, 0);
          break;
      }
    }

    @Override
    public void initialize() {

    }

    public boolean isBasePressed() {
      return baseIsPressed;
    }

    public double getElevatorJoystick() {
      return elevatorJoystick;
    }

    public boolean isElevatorLevel3ButtonPressed() {
      return currentLevel3ButtonPressed;
    }

    public boolean isElevatorLevel2ButtonPressed() {
      return currentLevel2ButtonPressed;
    }

    public boolean isElevatorLevel1ButtonPressed() {
      return currentLevel1ButtonPressed;
    }

    public boolean wasElevatorLevel3ButtonPressed() {
      return (currentLevel3ButtonPressed != lastLevel3ButtonPressed) && currentLevel3ButtonPressed;
    }

    public boolean wasElevatorLevel2ButtonPressed() {
      return (currentLevel2ButtonPressed != lastLevel2ButtonPressed) && currentLevel2ButtonPressed;
    }

    public boolean wasElevatorLevel1ButtonPressed() {
      return ((currentLevel1ButtonPressed != lastLevel1ButtonPressed) && currentLevel1ButtonPressed);
    }

    /* Get raw height of elevator from encoder ticks. */
    public int getElevatorHeight() {
      return currentEncoderPosition;
    }

    public ElevatorLevel getElevatorLevel() {
      int currentHeight = getElevatorHeight();

      return ElevatorLevel.UNKNOWN;
    }

    public void setElevatorLevel(ElevatorLevel level) {
      elevatorCurrentTarget = currentRobot.getTarget(level).getTarget();
    }

    public double getElevatorPower() {
      return elevatorCurrentPower;
    }

    public void setElevatorPower(double percent) {
      elevatorCurrentPower = percent;
    }

    public ElevatorControlMode getElevatorControlMode() {
      return elevatorControlMode;
    }

    public void setElevatorControlMode(ElevatorControlMode controlMode) {
      this.elevatorControlMode = controlMode;
    }

    public void setLimits(ElevatorLevel limits) {
      this.limits = limits;
      resetLimits = true;
    }

    public void releaseLowerLimit() {
      releaseLower = true;
    }
  }

  public class Cargo implements SubSubsystem {

    private final Logger cargoLogger;
    private long intakeTimeout = 0;
    // Inputs
    private boolean lastSlowOutButtonPressed;
    private boolean currentSlowOuttakePressed;
    private boolean lastInButtonPressed;
    private boolean currentInButtonPressed;
    private boolean lastFastOutButtonPressed;
    private boolean currentFastOutButtonPressed;
    private int angle;
    private boolean resetLimits = false;
    private double cargoJoystick;
    private CargoPosition limits = CargoPosition.SAFE;
    // Outputs
    private double intakePower;
    private double clawRotationPower;
    private int clawTarget;
    private ClawControlMode clawControlMode;

    public Cargo() {
      cargoLogger = new Logger();
    }

    public int getClawTarget() {
      return clawTarget;
    }

    public void setClawTarget(CargoPosition clawTarget) {
      this.clawTarget = currentRobot.getTarget(clawTarget).getTarget();
    }

    @Override
    public void collectData() {
      lastInButtonPressed = currentInButtonPressed;
      currentInButtonPressed = intakeCargoButton.get();
      lastSlowOutButtonPressed = currentSlowOuttakePressed;
      currentSlowOuttakePressed = outtakeCargoButtonSlow.get();
      lastFastOutButtonPressed = currentFastOutButtonPressed;
      currentFastOutButtonPressed = outtakeCargoButtonFast.get();
      angle = clawRotationMotor.getSelectedSensorPosition();
      cargoJoystick = -gamepad.getLeftY();
    }

    public void intakeCargo(int timeout) {
      intakeTimeout = currentTime + timeout;
      intakePower = 1;
    }

    public void outtakeCargoSlow(int timeout) {
      intakeTimeout = currentTime + timeout;
      intakePower = -0.5;
    }

    public void outtakeCargoFast(int timeout) {
      intakeTimeout = currentTime + timeout;
      intakePower = -1;
    }

    public void holdCargo() {
      intakePower = 0;
    }

    public ClawControlMode getClawControlMode() {
      return clawControlMode;
    }

    public void setClawControlMode(ClawControlMode clawControlMode) {
      this.clawControlMode = clawControlMode;
    }

    public double getClawRotationPower() {
      return clawRotationPower;
    }

    public void setClawRotationPower(double clawRotationPower) {
      this.clawRotationPower = clawRotationPower;
    }

    public void setClawAngle(int angle) {
      this.clawTarget = angle;
    }

    public boolean inButtonPressed() {
      return currentInButtonPressed;
    }

    public boolean inButtonRising() {
      return currentInButtonPressed && !lastInButtonPressed;
    }

    public boolean outSlowButtonPressed() {
      return currentSlowOuttakePressed;
    }

    public boolean outSlowButtonRising() {
      return currentSlowOuttakePressed && !lastSlowOutButtonPressed;
    }

    public boolean outFastButtonPressed() {
      return currentFastOutButtonPressed;
    }

    public boolean outFastButtonRising() {
      return currentFastOutButtonPressed && !lastFastOutButtonPressed;
    }

    public double getCargoJoystick() {
      return cargoJoystick;
    }

    public int getAngle() {
      return angle;
    }

    @Override
    public void outputData() {
//      String logOutput = String
//          .format("[%s]: Cargo height  : %d, Current height target: %d, Current power: %f", currentTime,
//              getAngle(), clawTarget, getCargoJoystick());
//      cargoLogger.logInfo(logOutput);

      if (resetLimits) {
        currentRobot.setCargoLimit(clawRotationMotor, limits);
        resetLimits = false;
      }

      switch (clawControlMode) {
        case AUTO:
          clawRotationMotor.set(ControlMode.Position, clawTarget);
          break;
        case MANUAL:
          clawRotationMotor.set(ControlMode.PercentOutput, clawRotationPower);
          clawTarget = angle;
          break;
        case DISABLED:
          clawRotationMotor.set(ControlMode.Disabled, 0);
          break;
      }

      if (currentTime <= intakeTimeout) {
        RobotMap.leftIntakeMotor.set(intakePower);
        RobotMap.rightIntakeMotor.set(intakePower);
      } else {
        RobotMap.leftIntakeMotor.set(0);
        RobotMap.rightIntakeMotor.set(0);
      }
    }

    public void setLimits(CargoPosition limits) {
      this.limits = limits;
      resetLimits = true;
    }

    @Override
    public void initialize() {

    }
  }

  public class Hatch implements SubSubsystem {
    // Output

    private final Logger hatchLogger;
    private boolean lastIntakeButtonPressed;
    private boolean currentIntakeButtonPressed;
    private boolean resetLimits = false;
    private HatchPosition limits = HatchPosition.SAFE;
    private int angle;
    private boolean intakeIsSet;
    private double hatchRotationPower;
    private int hatchTarget;
    private HatchControlMode hatchControlMode;

    public Hatch() {
      hatchLogger = new Logger();
    }

    public double getHatchRotationPower() {
      return hatchRotationPower;
    }

    public void setHatchRotationPower(double hatchRotationPower) {
      this.hatchRotationPower = hatchRotationPower;
    }

    public int getHatchTarget() {
      return hatchTarget;
    }

    public void setHatchTarget(HatchPosition hatchTarget) {
      this.hatchTarget = currentRobot.getTarget(hatchTarget).getTarget();
    }

    @Override
    public void collectData() {
      lastIntakeButtonPressed = currentIntakeButtonPressed;
      currentIntakeButtonPressed = hatchIntakeButton.get();
      angle = hatchRotationMotor.getSelectedSensorPosition();
    }

    public boolean isCurrentIntakeButtonPressed() {
      return currentIntakeButtonPressed;
    }

    @Override
    public void outputData() {
//      String logOutput = String
//          .format("[%s]: Hatch height  : %d, Current height target: %d, Current power: %f", currentTime,
//              getAngle(), hatchTarget, hatchRotationPower);
//      hatchLogger.logInfo(logOutput);

      if (resetLimits) {
        currentRobot.setHatchLimit(hatchRotationMotor, limits);
        resetLimits = false;
      }

      switch (hatchControlMode) {
        case AUTO:
          hatchRotationMotor.set(ControlMode.Position, hatchTarget);
          break;
        case MANUAL:
          hatchRotationMotor.set(ControlMode.PercentOutput, hatchRotationPower);
          hatchTarget = angle;
          break;
        case DISABLED:
          hatchRotationMotor.set(ControlMode.Disabled, 0);
          break;
      }

      hatchIntake.set(intakeIsSet);
    }

    @Override
    public void initialize() {

    }

    public int getAngle() {
      return angle;
    }

    public boolean getIntakeIsSet() {
      return intakeIsSet;
    }

    public void setIntake(boolean setOpen) {
      intakeIsSet = setOpen;
    }

    public HatchControlMode getHatchControlMode() {
      return hatchControlMode;
    }

    public void setHatchControlMode(HatchControlMode hatchControlMode) {
      this.hatchControlMode = hatchControlMode;
    }

    public void setHatchAngle(int angle) {
      this.hatchTarget = angle;
    }

    public boolean intakeButtonRising() {
      return currentIntakeButtonPressed && !lastIntakeButtonPressed;
    }

    public void setLimits(HatchPosition limits) {
      this.limits = limits;
      resetLimits = true;
    }
  }
}
