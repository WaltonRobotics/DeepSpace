package frc.robot.robotState;

import static frc.robot.Config.Cargo.CLIMB_MAX;
import static frc.robot.RobotMap.climberSolenoid;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.state.State;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.*;

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
    hatch.setControlMode(HatchControlMode.DISABLED);
    cargo.setControlMode(ClawControlMode.DISABLED);
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

//    if (Robot.godSubsystem.autoClimbRising()) {
//      return new PrepareAutoClimb();
//    }

     boolean elevatorManual = Math.abs(elevator.getElevatorJoystick()) > 0.1;

     if (elevatorManual) {
       climber.setClimberControlMode(ClimberControlMode.MANUAL);
       climber.setClimberPower(elevator.getElevatorJoystick());
     } else {
       climber.setClimberControlMode(ClimberControlMode.DISABLED);
     }

     if (climber.isClimberDeployPressed() == DoubleSolenoid.Value.kForward) {
       climberSolenoid.set(DoubleSolenoid.Value.kForward);
     }

    return this;
  }

  @Override
  public void finish() {
    RobotMap.clawRotationMotor
        .configPeakOutputForward(Robot.currentRobot.getCargoSubsystemLimits().getPeakOutputForward());
    RobotMap.clawRotationMotor
        .configPeakOutputReverse(Robot.currentRobot.getCargoSubsystemLimits().getPeakOutputReverse());
    hatch.setControlMode(HatchControlMode.AUTO);
  }

  @Override
  public String toString() {
    return "ClimbHandling{" +
        "hatch=" + hatch +
        ", elevator=" + elevator +
        ", cargo=" + cargo +
        ", climber=" + climber +
        '}';
  }
}
