package frc.robot.robotState;

import frc.robot.Robot;
import frc.robot.state.State;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ActiveState;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.CargoPosition;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.HatchPosition;

public class DefenseTransition implements State {


  @Override
  public void initialize() {
    Robot.godSubsystem.setCurrentActiveState(ActiveState.DEFENSE);
    Robot.godSubsystem.getHatch().setHatchTarget(HatchPosition.SAFE);
    Robot.godSubsystem.getCargo().setClawTarget(CargoPosition.SAFE);

  }

  @Override
  public State periodic() {

    if (!Robot.godSubsystem.isEnabled()) {
      return new Disabled();
    }

    int cargoAngle = Robot.godSubsystem.getCargo().getAngle();
    int hatchAngle = Robot.godSubsystem.getHatch().getAngle();

    return CargoPosition.SAFE.isClose(cargoAngle) && HatchPosition.SAFE.isClose(hatchAngle) ? new Defense() : this;
  }

  @Override
  public void finish() {

  }
}
