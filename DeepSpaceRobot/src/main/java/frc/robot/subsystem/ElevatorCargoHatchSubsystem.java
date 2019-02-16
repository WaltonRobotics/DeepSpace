package frc.robot.subsystem;


import static frc.robot.OI.elevatorDownButton;
import static frc.robot.OI.elevatorUpButton;
import static frc.robot.OI.flipCargoIntakeButton;
import static frc.robot.OI.intakeCargoButton;
import static frc.robot.OI.outtakeCargoButton;
import static frc.robot.RobotMap.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.robotState.Disabled;
import frc.robot.state.StateBuilder;
import frc.robot.util.Logger;

import javax.naming.ldap.Control;

public class ElevatorCargoHatchSubsystem extends Subsystem {

  private Elevator elevator = new Elevator();
  private Cargo cargo = new Cargo();
  private Hatch hatch = new Hatch();
  private ActiveState currentActiveState = ActiveState.ROBOT_SWITCHED_ON;

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
    stateMachine = new StateBuilder(new Disabled());

    elevator.intialize();
    cargo.intialize();
    hatch.intialize();
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


  public void openHatchIntake() {
    if (!hatchIntake.get()) {
      hatchIntake.set(true);
    }
  }

  public void closeHatchIntake() {
    if (hatchIntake.get()) {
      hatchIntake.set(false);
    }
  }

  public void flipOutHatchIntake() {
    hatchRotationMotor.set(ControlMode.PercentOutput, 1);
  }

  public void flipInHatchIntake() {
    hatchRotationMotor.set(ControlMode.PercentOutput, -1);
  }

  @Override
  public void periodic() {
    collectSensorData();
    processSensorData();
    output();
  }

  private void collectSensorData() {
    /* Read state of inputs. */
    elevator.collectData();
    cargo.collectData();
    hatch.collectData();
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
    UNKNOWN(0), BASE(100), LEVEL1(200), LEVEL2(300), LEVEL3(400);

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

  public enum ActiveState {
    ROBOT_SWITCHED_ON,
    CARGO_HANDLING,
    DEFENSE,
    HATCH_HANDLING
  }

  public class Elevator implements SubSubsystem {

    // Inputs
    private boolean elevatorLastUpButtonPressed;
    private boolean elevatorLastDownButtonPressed;
    private boolean elevatorCurrentUpButtonPressed;
    private boolean elevatorCurrentDownButtonPressed;
    private int elevatorCurrentEncoderPosition;

    // Output
    private double elevatorCurrentPower;
    private double elevatorCurrentTarget;

    private Timer elevatorRuntime;

    private ElevatorControlMode elevatorControlMode;
    private Logger elevatorLogger;

    public Elevator() {
      elevatorRuntime = new Timer();
      elevatorLogger = new Logger();
      elevatorControlMode = ElevatorControlMode.DISABLED;
    }

    @Override
    public void collectData() {
      elevatorLastUpButtonPressed = elevatorCurrentUpButtonPressed;
      elevatorCurrentUpButtonPressed = elevatorUpButton.get();

      elevatorLastDownButtonPressed = elevatorCurrentDownButtonPressed;
      elevatorCurrentDownButtonPressed = elevatorDownButton.get();

      elevatorCurrentEncoderPosition = elevatorMotor.getSelectedSensorPosition(0);

    }

    @Override
    public void outputData() {
      String logOutput = String
              .format("[%s]: Encoder height: %d, Current height target: %f, Current power: %f", elevatorRuntime.get(),
                      getElevatorHeight(), elevatorCurrentTarget, getElevatorPower());
      elevatorLogger.logInfo(logOutput);

      switch (elevatorControlMode) {
        case ZEROING:
          elevatorMotor.set(ControlMode.PercentOutput, elevatorCurrentPower);
          break;
        case AUTO:
          elevatorMotor.set(ControlMode.MotionMagic, elevatorCurrentTarget);
          break;
        case MANUAL:
          elevatorMotor.set(ControlMode.PercentOutput, elevatorCurrentPower);
          break;
        default:
          break;
      }
    }

    @Override
    public void intialize() {

    }

    public void resetElevator() {
      elevatorRuntime.reset();

      elevatorLastUpButtonPressed = false;
      elevatorLastDownButtonPressed = false;

      elevatorCurrentPower = 0.0;
      elevatorCurrentTarget = elevatorMotor.getSelectedSensorPosition(0);

      elevatorControlMode = ElevatorControlMode.AUTO;
    }

    public boolean isElevatorUpButtonPressed() {
      return elevatorCurrentUpButtonPressed;
    }

    public boolean isElevatorDownButtonPressed() {
      return elevatorCurrentDownButtonPressed;
    }

    public boolean wasElevatorUpButtonPressed() {
      return (elevatorCurrentUpButtonPressed != elevatorLastUpButtonPressed) && elevatorCurrentUpButtonPressed;
    }

    public boolean wasElevatorDownButtonPressed() {
      return (elevatorCurrentDownButtonPressed != elevatorLastDownButtonPressed) && elevatorCurrentDownButtonPressed;
    }

    /* Get raw height of elevator from encoder ticks. */
    public int getElevatorHeight() {
      return elevatorCurrentEncoderPosition;
    }

    public ElevatorLevel getElevatorLevel() {
      int currentHeight = getElevatorHeight();

      if (currentHeight >= ElevatorLevel.BASE.target && currentHeight < ElevatorLevel.LEVEL1.target) {
        return ElevatorLevel.BASE;
      }
      if (currentHeight >= ElevatorLevel.LEVEL1.target && currentHeight < ElevatorLevel.LEVEL2.target) {
        return ElevatorLevel.LEVEL1;
      }
      if (currentHeight >= ElevatorLevel.LEVEL2.target && currentHeight < ElevatorLevel.LEVEL3.target) {
        return ElevatorLevel.LEVEL2;
      }
      if (currentHeight >= ElevatorLevel.LEVEL3.target) {
        return ElevatorLevel.LEVEL3;
      }

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
  }

  public class Cargo implements SubSubsystem {

    // Inputs
    private boolean lastOutButtonPressed;
    private boolean lastInButtonPressed;
    private boolean lastFlipButtonPressed;
    private boolean currentOutButtonPressed;
    private boolean currentInButtonPressed;
    private boolean currentFlipButtonPressed;
    private int angle;
    // Outputs
    private double intakePower;
    private double clawRotationPower;
    private double clawTarget;
    private ClawControlMode clawControlMode;

    @Override
    public void collectData() {
      lastInButtonPressed = currentInButtonPressed;
      currentInButtonPressed = intakeCargoButton.get();
      lastOutButtonPressed = currentOutButtonPressed;
      currentOutButtonPressed = outtakeCargoButton.get();
      lastFlipButtonPressed = currentFlipButtonPressed;
      currentFlipButtonPressed = flipCargoIntakeButton.get();
      angle = clawRotationMotor.getSelectedSensorPosition();
    }

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

    @Override
    public void outputData() {

      switch (clawControlMode) {
        case AUTO:

        case MANUAL:

        case DISABLED:
      }
    }

    @Override
    public void intialize() {

    }
  }

  public enum CargoPosition {

    DEPLOY(0, 40),
    SAFE(0, 90),
    HATCH_START(100, 140),
    CARGO_START(180, 200);

    private int angle;
    private int upperBound;

    CargoPosition(int angle, int upperBound) {
      this.angle = angle;
      this.upperBound = upperBound;

    }

    public int getAngle() {
      return angle;
    }

    public boolean inRange(double angle) {
      return angle < upperBound;
    }

    public boolean isClose(double angle) {
      return Math.abs(angle - this.angle) < 10;
    }
  }



  public class Hatch implements SubSubsystem {
    // Output

    private double angle;
    private boolean intakePower;
    private double clawPower;
    private double clawTarget;

    @Override
    public void collectData() {

    }

    @Override
    public void outputData() {

    }

    @Override
    public void intialize() {

    }

    public double getAngle() {
      return angle;
    }
    public boolean getIntakePower() {
      return intakePower;
    }

    public double getClawPower() {
      return clawPower;
    }

    public double getClawTarget() {
      return clawTarget;
    }

    public void setIntakePower(boolean intakePower) {
      this.intakePower = intakePower;
    }

    public void setClawPower(double clawPower) {
      this.clawPower = clawPower;
    }

    public void setClawTarget(double clawTarget) {
      this.clawTarget = clawTarget;
    }
  }

  public enum HatchControlMode {
    DISABLED, AUTO, MANUAL
  }

  public enum HatchPosition {
    DEPLOY(0, 40),
    SAFE(80, 90),
    HATCH_START(100, 140),
    CARGO_START(180, 200);

    private double angle;
    private double upperBound;  // halfway between two different positions

    HatchPosition(double angle, double upperBound) {
      this.angle = angle;
      this.upperBound = upperBound;
    }

    public double getAngle() {
      return angle;
    }

    public boolean inRange(double angle) {
      return angle < upperBound;
    }

    public boolean isClose(double angle) {
      return Math.abs(angle - this.angle) < 10;
    }
  }

  /**
   * @return if the getAngle() value is in the enumerated range above the hatch position will be returned
   */
  public HatchPosition findHatchClosestPosition(HatchPosition hatchPosition, double angle) {
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
