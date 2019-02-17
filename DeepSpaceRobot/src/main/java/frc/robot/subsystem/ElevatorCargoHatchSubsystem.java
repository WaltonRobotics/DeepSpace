package frc.robot.subsystem;


import static frc.robot.OI.elevatorLevel1Button;
import static frc.robot.OI.elevatorLevel2Button;
import static frc.robot.OI.elevatorLevel3Button;
import static frc.robot.OI.elevatorZeroButton;
import static frc.robot.OI.gamepad;
import static frc.robot.OI.hatchIntakeButton;
import static frc.robot.OI.intakeCargoButton;
import static frc.robot.OI.outtakeCargoButtonFast;
import static frc.robot.OI.outtakeCargoButtonSlow;
import static frc.robot.RobotMap.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.robotState.Disabled;
import frc.robot.state.StateBuilder;
import frc.robot.util.Logger;

public class ElevatorCargoHatchSubsystem extends Subsystem {

  private Elevator elevator = new Elevator();
  private Cargo cargo = new Cargo();
  private Hatch hatch = new Hatch();
  private ActiveState currentActiveState = ActiveState.ROBOT_SWITCHED_ON;
  private long currentTime = 0;
  private boolean lastDefenceModePressed;
  private boolean currentDefenceModePressed;
  private boolean lastCargoModePressed;
  private boolean currentCargoModePressed;
  private boolean lastHatchModePressed;
  private boolean currentHatchModePressed;
  private boolean isEnabled = false;


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

  public Hatch getHatch() {
    return hatch;
  }

  private StateBuilder stateMachine;

  // Inputs

  // Output

  // ???

  public ElevatorCargoHatchSubsystem() {
    elevator.initialize();
    cargo.initialize();
    hatch.initialize();
    // Set sense of encoder
    // Set sense of motors
    // Set soft targets on
    // Configure elevator encoder

  }

  public ActiveState getCurrentActiveState() {
    return currentActiveState;
  }

  public void setCurrentActiveState(ActiveState currentActiveState) {
    this.currentActiveState = currentActiveState;
  }

  @Override
  public void periodic() {
    if (stateMachine == null){
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
    //TODO:currentCargoModePressed = button;
    lastHatchModePressed = currentHatchModePressed;
    //TODO:currentHatchModePressed = button;
    lastDefenceModePressed = currentDefenceModePressed;
    //TODO:currentDefenceModePressed = button;
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
  }

  @Override
  protected void initDefaultCommand() {

  }

  public enum ElevatorLevel {
    UNKNOWN(0), BASE(100), CARGO1(200), HATCH1(250), CARGO2(300), HATCH2(350), CARGO3(400), HATCH3(450);

    private double target;

    ElevatorLevel(double target) {
      this.target = target;
    }

    public double getTarget() {
      return target;
    }
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

  public class Elevator implements SubSubsystem {

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

    public boolean isZeroed() {
      return isZeroed;
    }

    public void setZeroed(boolean zeroed) {
      isZeroed = zeroed;
    }

    public boolean isLowerLimit() {
      return lowerLimit;
    }

    // Output
    private double elevatorCurrentPower;
    private double elevatorCurrentTarget;

    private ElevatorControlMode elevatorControlMode;
    private Logger elevatorLogger;

    public Elevator() {
      elevatorLogger = new Logger();
    }

    @Override
    public void collectData() {
      lastLevel3ButtonPressed = currentLevel3ButtonPressed;
      currentLevel3ButtonPressed = elevatorLevel3Button.get();

      lastLevel2ButtonPressed = currentLevel2ButtonPressed;
      currentLevel2ButtonPressed = elevatorLevel2Button.get();

      lastLevel1ButtonPressed = currentLevel1ButtonPressed;
      currentLevel1ButtonPressed = elevatorLevel1Button.get();

      elevatorJoystick = gamepad.getRightY();
      currentEncoderPosition = elevatorMotor.getSelectedSensorPosition(0);

      lowerLimit = elevatorLowerLimit.get();
      baseIsPressed = elevatorZeroButton.get();
    }

    @Override
    public void outputData() {
      String logOutput = String
          .format("[%s]: Encoder height: %d, Current height target: %f, Current power: %f", currentTime,
              getElevatorHeight(), elevatorCurrentTarget, getElevatorPower());
      elevatorLogger.logInfo(logOutput);

      if(resetLimits) {
        Robot.currentRobot.setElevatorLimit(elevatorMotor, limits);
        resetLimits = false;
      }

      if(releaseLower) {
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

    public boolean wasElevatorlevel2ButtonPressed() {
      return (currentLevel2ButtonPressed != lastLevel2ButtonPressed) && currentLevel2ButtonPressed;
    }

    public boolean wasElevatorLevel1ButtonPressed() {
      return (currentLevel1ButtonPressed != lastLevel1ButtonPressed && currentLevel1ButtonPressed);
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
      elevatorCurrentTarget = level.getTarget();
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

    private Logger cargoLogger;
    public Cargo() {
      cargoLogger  = new Logger();
    }

    // Outputs
    private double intakePower;
    private double clawRotationPower;
    private int clawTarget;
    private ClawControlMode clawControlMode;

    @Override
    public void collectData() {
      lastInButtonPressed = currentInButtonPressed;
      currentInButtonPressed = intakeCargoButton.get();
      lastSlowOutButtonPressed = currentSlowOuttakePressed;
      currentSlowOuttakePressed = outtakeCargoButtonSlow.get();
      lastFastOutButtonPressed = currentFastOutButtonPressed;
      currentFastOutButtonPressed = outtakeCargoButtonFast.get();
      angle = clawRotationMotor.getSelectedSensorPosition();
      cargoJoystick = gamepad.getLeftY();
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

    public void setClawTarget(CargoPosition clawTarget) {
      this.clawTarget = clawTarget.getAngle();
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
      String logOutput = String
          .format("[%s]: Cargo height  : %d, Current height target: %d, Current power: %f", currentTime,
              getAngle(), clawTarget, getCargoJoystick());
      cargoLogger.logInfo(logOutput);

      if(resetLimits) {
        Robot.currentRobot.setCargoLimit(clawRotationMotor, limits);
        resetLimits = false;
      }

      switch (clawControlMode) {
        case AUTO:
          clawRotationMotor.set(ControlMode.MotionMagic, clawTarget);
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

  public enum CargoPosition {

    DEPLOY(0, 40),
    SAFE(0, 90),
    HATCH_START(100, 140),
    CARGO_START(180, 200),
    ANGLED(0, 10);

    private int angle;
    private int upperBound;

    CargoPosition(int angle, int upperBound) {
      this.angle = angle;
      this.upperBound = upperBound;

    }

    public int getAngle() {
      return angle;
    }

    public boolean inRange(int angle) {
      return angle < upperBound;
    }

    public boolean isClose(int angle) {
      return Math.abs(angle - this.angle) < 10;
    }
  }

  public class Hatch implements SubSubsystem {
    // Output

    private boolean lastIntakeButtonPressed;
    private boolean currentIntakeButtonPressed;
    private boolean lastFlipButtonPressed;
    private boolean currentFlipButtonPressed;
    private boolean resetLimits = false;

    private HatchPosition limits = HatchPosition.SAFE;
    private int angle;
    private boolean intakeIsSet;
    private double hatchRotationPower;
    private int hatchTarget;
    private HatchControlMode hatchControlMode;

    private Logger hatchLogger;

    public Hatch() {
      hatchLogger = new Logger();
    }

    @Override
    public void collectData() {
      lastIntakeButtonPressed = currentIntakeButtonPressed;
      currentIntakeButtonPressed = hatchIntakeButton.get();
      lastFlipButtonPressed = currentFlipButtonPressed;
      currentIntakeButtonPressed = hatchIntakeButton.get();
      angle = hatchRotationMotor.getSelectedSensorPosition();
    }

    public boolean isCurrentIntakeButtonPressed() {
      return currentIntakeButtonPressed;
    }

    @Override
    public void outputData() {
      String logOutput = String
          .format("[%s]: Hatch height  : %d, Current height target: %d, Current power: %f", currentTime,
              getAngle(), hatchTarget, hatchRotationPower);
      hatchLogger.logInfo(logOutput);

      if(resetLimits) {
        Robot.currentRobot.setHatchLimit(hatchRotationMotor, limits);
        resetLimits = false;
      }

      switch (hatchControlMode) {
        case AUTO:
          hatchRotationMotor.set(ControlMode.MotionMagic, hatchTarget);
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


    public void setHatchRotationPower(double hatchRotationPower) {
      this.hatchRotationPower = hatchRotationPower;
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

    public void setHatchTarget(HatchPosition hatchTarget) {
      this.hatchTarget = hatchTarget.getAngle();
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

  public enum HatchPosition {
    DEPLOY(0, 40),
    SAFE(80, 90),
    HATCH_START(100, 140),
    CARGO_START(180, 200);

    private int angle;
    private int upperBound;  // halfway between two different positions

    HatchPosition(int angle, int upperBound) {
      this.angle = angle;
      this.upperBound = upperBound;
    }

    public int getAngle() {
      return angle;
    }

    public boolean inRange(int angle) {
      return angle < upperBound;
    }

    public boolean isClose(int angle) {
      return Math.abs(angle - this.angle) < 10;
    }
  }

  /**
   * @return if the getAngle() value is in the enumerated range above the hatch position will be returned
   */

  public HatchPosition findHatchClosestPosition(HatchPosition hatchPosition, int angle) {
    if (HatchPosition.DEPLOY.inRange(angle)) {
      return HatchPosition.DEPLOY;
    } else if (HatchPosition.SAFE.inRange(angle)) {
      return HatchPosition.SAFE;
    } else if (HatchPosition.HATCH_START.inRange(angle)) {
      return HatchPosition.HATCH_START;
    } else {
      return HatchPosition.CARGO_START;
    }
  }
}
