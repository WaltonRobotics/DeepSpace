package frc.robot.robotState;

import static frc.robot.Config.Cargo.CLIMB_MAX;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.state.State;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.Cargo;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ClawControlMode;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.Climber;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.Elevator;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ElevatorControlMode;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.Hatch;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.HatchControlMode;

/**
 * @author Marius Juston
 **/
public class ClimbHandling implements State {

  private final Hatch hatch = Robot.godSubsystem.getHatch();
  private final Elevator elevator = Robot.godSubsystem.getElevator();
  private final Cargo cargo = Robot.godSubsystem.getCargo();
  private final Climber climber = Robot.godSubsystem.getClimber();


  @Override
  public void initialize() {
    hatch.setHatchControlMode(HatchControlMode.DISABLED);

  }

  @Override
  public State periodic() {
    if (!Robot.godSubsystem.isEnabled()) {
      return new Disabled();
    }

    if (Robot.godSubsystem.cargoModeRising()) {
      return new CargoHandlingTransition();
    }
    if (Robot.godSubsystem.hatchModeRising()) {
      return new HatchHandlingTransition();
    }
    if (Robot.godSubsystem.defenceModeRising()) {
      return new DefenseTransition();
    }

    boolean elevatorManual = Math.abs(elevator.getElevatorJoystick()) > 0.1;
    boolean cargoManual = Math.abs(cargo.getCargoJoystick()) > 0.1;

    if (elevatorManual || cargoManual) {
      if (elevatorManual) {
        elevator.setElevatorControlMode(ElevatorControlMode.MANUAL);
        elevator.setElevatorPower(elevator.getElevatorJoystick());
      } else {
        elevator.setElevatorControlMode(ElevatorControlMode.AUTO);
      }

      if (cargoManual) {
        cargo.setClawControlMode(ClawControlMode.MANUAL);

        double cargoJoystick = cargo.getCargoJoystick();

        cargoJoystick = Math.signum(cargoJoystick) * Math.min(Math.abs(cargoJoystick), CLIMB_MAX);
        cargo.setCargoRotationPower(cargoJoystick);
      } else {
        cargo.setClawControlMode(ClawControlMode.AUTO);
      }
    } else {
      elevator.setElevatorControlMode(ElevatorControlMode.AUTO);
      cargo.setClawControlMode(ClawControlMode.AUTO);
    }

    if (climber.isClimberUpPressed()) {
      climber.setClimberPower(-0.5);
    } else if (climber.isClimberDownPressed()) {
      climber.setClimberPower(1.0);
    } else {
      climber.setClimberPower(0.0);
    }

    if (cargo.outSlowButtonPressed()) {
      cargo.outtakeCargoSlow(0);
    } else if (cargo.outFastButtonPressed()) {
      cargo.outtakeCargoFast(0);
    } else if (cargo.inButtonPressed()) {
      cargo.intakeCargo(0);
    }

    return this;
  }

  @Override
  public void finish() {
    RobotMap.clawRotationMotor
        .configPeakOutputForward(Robot.currentRobot.getCargoSubsystemLimits().getPeakOutputForward());
    RobotMap.clawRotationMotor
        .configPeakOutputReverse(Robot.currentRobot.getCargoSubsystemLimits().getPeakOutputReverse());
    hatch.setHatchControlMode(HatchControlMode.AUTO);
  }
}
