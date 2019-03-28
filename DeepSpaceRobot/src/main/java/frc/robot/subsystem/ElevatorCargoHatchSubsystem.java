package frc.robot.subsystem;


import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kOff;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;
import static frc.robot.Config.Elevator.ZEROING;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_CARGO_ANGLE;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_CARGO_MODE;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_CARGO_POWER;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_CARGO_TARGET;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_CLAW_ForwardSoftLimit;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_CLAW_ReverseSoftLimit;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_CLIMBER_MODE;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_CLIMBER_POWER;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_ELEVATOR_ForwardSoftLimit;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_ELEVATOR_HEIGHT;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_ELEVATOR_MODE;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_ELEVATOR_POWER;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_ELEVATOR_ReverseSoftLimit;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_ELEVATOR_TARGET;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_HATCH_ANGLE;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_HATCH_ForwardSoftLimit;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_HATCH_MODE;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_HATCH_POWER;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_HATCH_ReverseSoftLimit;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_HATCH_TARGET;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_INTAKE_OPEN;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_LOWER_LIMIT;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_STATE;
import static frc.robot.OI.bringDown;
import static frc.robot.OI.bringUp;
import static frc.robot.OI.cargoModeButton;
import static frc.robot.OI.cargoStart;
import static frc.robot.OI.elevatorLevel1Button;
import static frc.robot.OI.elevatorLevel2Button;
import static frc.robot.OI.elevatorLevel3Button;
import static frc.robot.OI.elevatorZeroButton;
import static frc.robot.OI.gamepad;
import static frc.robot.OI.hatchIntakeButton;
import static frc.robot.OI.hatchModeButton;
import static frc.robot.OI.hatchStart;
import static frc.robot.OI.intakeCargoButton;
import static frc.robot.OI.outtakeCargoButtonFast;
import static frc.robot.OI.outtakeCargoButtonSlow;
import static frc.robot.Robot.currentRobot;
import static frc.robot.RobotMap.clawRotationMotor;
import static frc.robot.RobotMap.climberMotor;
import static frc.robot.RobotMap.elevatorLowerLimit;
import static frc.robot.RobotMap.elevatorMotor;
import static frc.robot.RobotMap.hatchIntake;
import static frc.robot.RobotMap.hatchRotationMotor;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Gamepad.POV;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.robotState.Disabled;
import frc.robot.state.StateBuilder;
import frc.robot.util.EnhancedBoolean;
import frc.robot.util.Logger;

public class ElevatorCargoHatchSubsystem extends Subsystem {

  private final Elevator elevator = new Elevator();
  private final Cargo cargo = new Cargo();
  private final Hatch hatch = new Hatch();
  private final Climber climber = new Climber();
  private ActiveState currentActiveState = ActiveState.ROBOT_SWITCHED_ON;
  private long currentTime = 0L;
  private EnhancedBoolean currentDefenceModePressed = new EnhancedBoolean();
  private EnhancedBoolean currentCargoModePressed = new EnhancedBoolean();
  private EnhancedBoolean currentHatchModePressed = new EnhancedBoolean();
  private boolean isEnabled = false;
  private StateBuilder stateMachine;
  private EnhancedBoolean currentSetStartModePressed = new EnhancedBoolean();
  private EnhancedBoolean currentSetStartCargoModePressed = new EnhancedBoolean();
  private EnhancedBoolean climberPreset = new EnhancedBoolean();
  private EnhancedBoolean autoClimbMode = new EnhancedBoolean();
  private EnhancedBoolean masterOverride = new EnhancedBoolean();

  public ElevatorCargoHatchSubsystem() {
    elevator.initialize();
    cargo.initialize();
    hatch.initialize();
    climber.initialize();
    // Set sense of encoder
    // Set sense of motors
    // Set soft targets on
    // Configure elevator encoder

  }

  public Climber getClimber() {
    return climber;
  }

  public long getCurrentTime() {
    return currentTime;
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

  // Inputs

  // Output

  // ???

  public Cargo getCargo() {
    return cargo;
  }

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
    climber.collectData();

    currentCargoModePressed.set(cargoModeButton.get());
    currentHatchModePressed.set(hatchModeButton.get());
//    currentDefenceModePressed = defenseModeButton.get();
    currentDefenceModePressed.set(gamepad.getPOVButton(POV.S));
    currentSetStartModePressed.set(hatchStart.get());
    currentSetStartCargoModePressed.set(cargoStart.get());
    climberPreset.set(gamepad.getPOVButton(POV.W));
    autoClimbMode.set(OI.leftJoystick.getTrigger());
    masterOverride.set(OI.masterOverride.get());

  }

  public boolean isMasterOverride() {
    return masterOverride.get();
  }


  public boolean autoClimbRising() {
    return autoClimbMode.isRisingEdge();
  }

  public boolean climbModeRising() {
    return climberPreset.isRisingEdge();
  }

  public boolean cargoModeRising() {
    return currentCargoModePressed.isRisingEdge();
  }

  public boolean hatchModeRising() {
    return currentHatchModePressed.isRisingEdge();
  }

  public boolean defenceModeRising() {
    return currentDefenceModePressed.isRisingEdge();
  }

  public boolean setCompStartHatchModeRising() {
    return currentSetStartModePressed.isRisingEdge();
  }


  public boolean setCompStartCargoModeRising() {
    return currentSetStartCargoModePressed.isRisingEdge();
  }

  private void processSensorData() {
    stateMachine.step();
  }

  private void output() {
    // Here's where we actually set the motors etc...

    elevator.outputData();
    cargo.outputData();
    hatch.outputData();
    climber.outputData();

    SmartDashboard.putString(MOTORS_STATE, stateMachine.getCurrentState().getClass().getSimpleName());
    SmartDashboard.putNumber(MOTORS_ELEVATOR_HEIGHT, getElevator().getElevatorHeight());
    SmartDashboard.putNumber(MOTORS_ELEVATOR_POWER, getElevator().getElevatorPower());
    SmartDashboard.putNumber(MOTORS_ELEVATOR_TARGET, getElevator().getCurrentTarget());
    SmartDashboard.putString(MOTORS_ELEVATOR_MODE, getElevator().getControlMode().name());

    SmartDashboard.putNumber(MOTORS_HATCH_ANGLE, getHatch().getAngle());
    SmartDashboard.putNumber(MOTORS_HATCH_POWER, getHatch().getRotationPower());
    SmartDashboard.putNumber(MOTORS_HATCH_TARGET, getHatch().getCurrentTarget());
    SmartDashboard.putString(MOTORS_HATCH_MODE, getHatch().getControlMode().name());
    SmartDashboard.putBoolean(MOTORS_INTAKE_OPEN, getHatch().getIntakeIsSet());

    SmartDashboard.putNumber(MOTORS_CARGO_ANGLE, getCargo().getAngle());
    SmartDashboard.putNumber(MOTORS_CARGO_POWER, getCargo().getRotationPower());
    SmartDashboard.putNumber(MOTORS_CARGO_TARGET, getCargo().getCurrentTarget());
    SmartDashboard.putString(MOTORS_CARGO_MODE, getCargo().getControlMode().name());

    SmartDashboard.putNumber(MOTORS_CLIMBER_POWER, getClimber().getClimberPower());
    SmartDashboard.putString(MOTORS_CLIMBER_MODE, getClimber().getClimberControlMode().name());

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

  public HatchPosition findHatchClosestPosition(int angle) {
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
    CARGO_BASE, HATCH_BASE, CARGO_HAB, CARGO_ROCKET, CARGO2, HATCH2, CARGO3, CLIMB, HATCH3
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
    CARGO_3,
    CARGO_2, CARGO_1, CARGO_START, CLIMB, HAB, SAFE
  }

  public enum HatchPosition {
    DEPLOY,
    SAFE,
    HATCH_START,
    DEFENSE, CARGO_START
  }

  public enum ClimberControlMode {
    AUTO, MANUAL, TIMED, DISABLED
  }

  public class Elevator implements SubSubsystem {

    private final Logger elevatorLogger;
    // Inputs
    private EnhancedBoolean level3Button = new EnhancedBoolean();
    private EnhancedBoolean level2Button = new EnhancedBoolean();
    private EnhancedBoolean level1Button = new EnhancedBoolean();
    private boolean lowerLimit;
    private EnhancedBoolean isZeroed = new EnhancedBoolean();
    private boolean resetLimits = false;
    private boolean releaseLower = false;
    private double elevatorJoystick;
    private boolean baseIsPressed;
    private int currentEncoderPosition;
    private ElevatorLevel limits = ElevatorLevel.CARGO_BASE;
    // Output
    private double currentPower;
    private double currentTarget;
    private ElevatorControlMode controlMode;
    private int lastEncoderPosition;
    private boolean cargoShipPressed;

    public Elevator() {
      elevatorLogger = new Logger();
    }

    public int getLastEncoderPosition() {
      return lastEncoderPosition;
    }

    public double getCurrentTarget() {
      return currentTarget;
    }

    public boolean isZeroed() {
      return isZeroed.get();
    }

    public void setZeroed(boolean zeroed) {
      isZeroed.set(zeroed);
    }

    public boolean isLowerLimit() {
      return lowerLimit;
    }

    @Override
    public void collectData() {
      level3Button.set(elevatorLevel3Button.get());
      level2Button.set(elevatorLevel2Button.get());
      level1Button.set(elevatorLevel1Button.get());

      elevatorJoystick = -gamepad.getRightY();

      lastEncoderPosition = currentEncoderPosition;
      currentEncoderPosition = elevatorMotor.getSelectedSensorPosition(0);

      lowerLimit = !elevatorLowerLimit.get();
      baseIsPressed = elevatorZeroButton.get();
      cargoShipPressed = gamepad.getPOVButton(POV.N);
    }

    @Override
    public void outputData() {
//      String logOutput = String
//          .format("[%s]: Encoder height: %d, Current height target: %f, Current power: %f", currentTime,
//              getElevatorHeight(), currentTarget, getElevatorPower());
//      elevatorLogger.logInfo(logOutput);

      if (resetLimits) {
        currentRobot.setElevatorLimit(elevatorMotor, limits);
        resetLimits = false;
      }

      if (releaseLower) {
        elevatorMotor.configReverseSoftLimitEnable(false);
      } else {
        elevatorMotor.configReverseSoftLimitEnable(true);
      }

      switch (controlMode) {
        case ZEROING:
          elevatorMotor.set(ControlMode.PercentOutput, ZEROING);
          if (isZeroRising()) {
            elevatorMotor.setSelectedSensorPosition(0);
          }
          currentTarget = 0;
          break;
        case AUTO:
          elevatorMotor.set(ControlMode.MotionMagic, currentTarget);
          currentPower = 0;
          break;
        case MANUAL:
          elevatorMotor.set(ControlMode.PercentOutput, currentPower);
          currentTarget = currentEncoderPosition;
          break;
        case DISABLED:
          elevatorMotor.set(ControlMode.Disabled, 0);
          currentPower = 0;
          currentTarget = 0;
          break;
      }
    }

    public boolean isZeroRising() {
      return isZeroed.isRisingEdge();
    }

    public void enableLowerLimit() {
      releaseLower = false;
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
      return level3Button.get();
    }

    public boolean isElevatorLevel2ButtonPressed() {
      return level2Button.get();
    }

    public boolean isElevatorLevel1ButtonPressed() {
      return level1Button.get();
    }

    public boolean wasElevatorLevel3ButtonPressed() {
      return level3Button.hasChanged() && level3Button.get();
    }

    public boolean wasElevatorLevel2ButtonPressed() {
      return level2Button.hasChanged() && level2Button.get();
    }

    public boolean wasElevatorLevel1ButtonPressed() {
      return level1Button.hasChanged() && level1Button.get();
    }

    /* Get raw height of elevator from encoder ticks. */
    public int getElevatorHeight() {
      return currentEncoderPosition;
    }

    public void setElevatorLevel(ElevatorLevel level) {
      currentTarget = currentRobot.getTarget(level).getTarget();
    }

    public double getElevatorPower() {
      return currentPower;
    }

    public void setElevatorPower(double percent) {
      currentPower = percent;
    }

    public ElevatorControlMode getControlMode() {
      return controlMode;
    }

    public void setControlMode(ElevatorControlMode controlMode) {
      this.controlMode = controlMode;
    }

    public void setLimits(ElevatorLevel limits) {
      this.limits = limits;
      resetLimits = true;
    }

    public void releaseLowerLimit() {
      releaseLower = true;
    }

    public boolean isElevatorCargoShipButtonPressed() {
      return cargoShipPressed;
    }
  }

  public class Cargo implements SubSubsystem {

    private final Logger cargoLogger;
    private long intakeTimeout = 0L;
    // Inputs
    private int angle;
    private boolean resetLimits = false;
    private double cargoJoystick;
    private CargoPosition limits = CargoPosition.SAFE;
    // Outputs
    private double intakePower;
    private double rotationPower;
    private int currentTarget;
    private ClawControlMode controlMode;

    private EnhancedBoolean slowOutake = new EnhancedBoolean();
    private EnhancedBoolean fastOuttake = new EnhancedBoolean();
    private EnhancedBoolean fastIntake = new EnhancedBoolean();
    private EnhancedBoolean slowIntake = new EnhancedBoolean();
    private EnhancedBoolean cargoTurbo = new EnhancedBoolean();

    public Cargo() {
      cargoLogger = new Logger();
    }

    public int getCurrentTarget() {
      return currentTarget;
    }

    public void setCurrentTarget(CargoPosition currentTarget) {
      this.currentTarget = currentRobot.getTarget(currentTarget).getTarget();
    }

    @Override
    public void collectData() {
      fastIntake.set(intakeCargoButton.get());
      slowOutake.set(outtakeCargoButtonSlow.get());
      fastOuttake.set(outtakeCargoButtonFast.get());
      slowIntake.set(hatchIntakeButton.get());
      cargoTurbo.set(OI.cargoTurbo.get());

      angle = clawRotationMotor.getSelectedSensorPosition();
      cargoJoystick = -gamepad.getLeftY();
    }

    public boolean cargoTurbo() {
      return cargoTurbo.get();
    }

    public void intakeCargoFast(int timeout) {
      intakeTimeout = currentTime + timeout;
      intakePower = 1.0;
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

    public ClawControlMode getControlMode() {
      return controlMode;
    }

    public void setControlMode(ClawControlMode controlMode) {
      this.controlMode = controlMode;
    }

    public double getRotationPower() {
      return rotationPower;
    }

    public void setRotationPower(double rotationPower) {
      this.rotationPower = rotationPower;
    }

    public void setClawAngle(int angle) {
      this.currentTarget = angle;
    }

    public boolean inFastButtonPressed() {
      return fastIntake.get();
    }

    public boolean inFastButtonRising() {
      return fastIntake.isRisingEdge();
    }

    public boolean outSlowButtonPressed() {
      return slowOutake.get();
    }

    public boolean outSlowButtonRising() {
      return slowOutake.isRisingEdge();
    }

    public boolean outFastButtonPressed() {
      return fastOuttake.get();
    }

    public boolean outFastButtonRising() {
      return fastOuttake.isRisingEdge();
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
//              getAngle(), currentTarget, getCargoJoystick());
//      cargoLogger.logInfo(logOutput);

      if (resetLimits) {
        currentRobot.setCargoLimit(clawRotationMotor, limits);
        resetLimits = false;
      }

      switch (controlMode) {
        case AUTO:
          clawRotationMotor.set(ControlMode.Position, currentTarget);
          rotationPower = 0;
          break;
        case MANUAL:
          clawRotationMotor.set(ControlMode.PercentOutput, rotationPower);
          currentTarget = angle;
          break;
        case DISABLED:
          clawRotationMotor.set(ControlMode.Disabled, 0);
          rotationPower = 0;
          currentTarget = 0;
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

    public void intakeCargoSlow(int timeout) {
      intakeTimeout = currentTime + timeout;
      intakePower = 0.5;
    }

    public boolean inSlowButtonPressed() {
      return slowIntake.get();
    }

    public void intakeCargoHold() {
      intakePower = 0.1;
      intakeTimeout = currentTime;
    }
  }

  public class Hatch implements SubSubsystem {
    // Output

    private final Logger hatchLogger;
    private EnhancedBoolean intake = new EnhancedBoolean();
    private boolean resetLimits = false;
    private HatchPosition limits = HatchPosition.SAFE;
    private int angle;
    private boolean intakeIsSet;
    private double rotationPower;
    private int currentTarget;
    private HatchControlMode controlMode;

    public Hatch() {
      hatchLogger = new Logger();
    }

    public double getRotationPower() {
      return rotationPower;
    }

    public void setRotationPower(double rotationPower) {
      this.rotationPower = rotationPower;
    }

    public int getCurrentTarget() {
      return currentTarget;
    }

    public void setCurrentTarget(HatchPosition currentTarget) {
      this.currentTarget = currentRobot.getTarget(currentTarget).getTarget();
    }

    @Override
    public void collectData() {
      intake.set(hatchIntakeButton.get());
      angle = hatchRotationMotor.getSelectedSensorPosition();
    }

    public boolean isCurrentIntakeButtonPressed() {
      return intake.get();
    }

    @Override
    public void outputData() {
//      String logOutput = String
//          .format("[%s]: Hatch height  : %d, Current height target: %d, Current power: %f", currentTime,
//              getAngle(), currentTarget, rotationPower);
//      hatchLogger.logInfo(logOutput);

      if (resetLimits) {
        currentRobot.setHatchLimit(hatchRotationMotor, limits);
        resetLimits = false;
      }

      switch (controlMode) {
        case AUTO:
          hatchRotationMotor.set(ControlMode.Position, currentTarget);
          rotationPower = 0;
          break;
        case MANUAL:
          hatchRotationMotor.set(ControlMode.PercentOutput, rotationPower);
          currentTarget = angle;
          break;
        case DISABLED:
          hatchRotationMotor.set(ControlMode.Disabled, 0);
          currentTarget = 0;
          rotationPower = 0;
          break;
      }

      Value hatchMode = kOff;
      if (intakeIsSet) {
        hatchMode = kForward;
      } else {
        hatchMode = kReverse;
      }

      hatchIntake.set(hatchMode);
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
      System.out.println(stateMachine.getCurrentState().getClass().getSimpleName() + "\t" + setOpen);
      intakeIsSet = setOpen;
    }

    public HatchControlMode getControlMode() {
      return controlMode;
    }

    public void setControlMode(HatchControlMode controlMode) {
      this.controlMode = controlMode;
    }

    public void setHatchAngle(int angle) {
      this.currentTarget = angle;
    }

    public boolean intakeButtonRising() {
      return intake.isRisingEdge();
    }

    public void setLimits(HatchPosition limits) {
      this.limits = limits;
      resetLimits = true;
    }
  }

  public class Climber implements SubSubsystem {


    public long timeout;
    private EnhancedBoolean climberUp = new EnhancedBoolean();
    private EnhancedBoolean climberDown = new EnhancedBoolean();
    private ClimberControlMode climberControlMode;
    private double climberPower;

    public ClimberControlMode getClimberControlMode() {
      return climberControlMode;
    }

    public void setClimberControlMode(ClimberControlMode climberControlMode) {
      this.climberControlMode = climberControlMode;
    }

    public double getClimberPower() {
      return climberPower;
    }

    public void setClimberPower(double climberPower) {
      this.climberPower = climberPower;
    }

    public boolean isClimberUpRising() {
      return climberUp.isRisingEdge();
    }

    public boolean isClimberUpPressed() {
      return climberUp.get();
    }

    public boolean isClimberDownPressed() {
      return climberDown.get();
    }

    public boolean isClimberDownRising() {
      return climberDown.isRisingEdge();
    }

    @Override
    public void collectData() {
      climberUp.set(bringUp.get());
      climberDown.set(bringDown.get());
    }

    @Override
    public void outputData() {
      switch (climberControlMode) {
        case TIMED:
          if (currentTime <= timeout) {
            climberMotor.set(climberPower);
          } else {
            climberControlMode = ClimberControlMode.MANUAL;
            climberPower = 0;
          }
          break;
        case MANUAL:
          climberMotor.set(climberPower);
          break;
        case DISABLED:
          climberMotor.set(0);
//          climberMotor.disable();
          break;
      }
    }

    public void setTimer(int interval, double power) {
      timeout = currentTime + interval;
      climberPower = power;
      climberControlMode = ClimberControlMode.TIMED;
    }

    @Override
    public void initialize() {

    }

    public void setTimer(double power) {
      setTimer(750, power);
    }
  }
}
