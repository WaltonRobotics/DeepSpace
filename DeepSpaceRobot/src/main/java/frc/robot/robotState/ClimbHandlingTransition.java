package frc.robot.robotState;

import static frc.robot.Config.Cargo.CLIMB_MAX;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.state.State;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ActiveState;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.CargoPosition;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ClimberControlMode;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ElevatorControlMode;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ElevatorLevel;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.HatchPosition;

/**
 * @author Marius Juston
 **/
public class ClimbHandlingTransition implements State {

  @Override
  public void initialize() {
    Robot.godSubsystem.setCurrentActiveState(ActiveState.CARGO_HANDLING);

    Robot.godSubsystem.getHatch().setCurrentTarget(HatchPosition.SAFE);
    Robot.godSubsystem.getCargo().setCurrentTarget(CargoPosition.CLIMB);
    Robot.godSubsystem.getCargo().setLimits(CargoPosition.CLIMB);

    Robot.godSubsystem.getElevator().setControlMode(ElevatorControlMode.AUTO);
    Robot.godSubsystem.getElevator().setElevatorLevel(ElevatorLevel.CLIMB);
    Robot.godSubsystem.getElevator().setLimits(ElevatorLevel.CARGO_BASE);

    RobotMap.clawRotationMotor.configPeakOutputForward(CLIMB_MAX);
    RobotMap.clawRotationMotor.configPeakOutputReverse(-CLIMB_MAX);
    Robot.godSubsystem.getClimber().setClimberControlMode(ClimberControlMode.MANUAL);
  }

  @Override
  public State periodic() {
    if (!Robot.godSubsystem.isEnabled()) {
      return new Disabled();
    }

    if (Robot.godSubsystem.isMasterOverride()) {
      return new ClimbHandling();
    }

    int elevatorHeight = Robot.godSubsystem.getElevator().getElevatorHeight();
    int cargoAngle = Robot.godSubsystem.getCargo().getAngle();

    return (Robot.currentRobot.getTarget(CargoPosition.CLIMB).isClose(cargoAngle, 50) && Robot.currentRobot
        .getTarget(ElevatorLevel.CLIMB).isClose(elevatorHeight, 250)) ? new ClimbHandling() : this;
  }

  @Override
  public void finish() {

  }
}
