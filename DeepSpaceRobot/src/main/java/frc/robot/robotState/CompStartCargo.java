package frc.robot.robotState;

import frc.robot.Robot;
import frc.robot.state.State;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.CargoPosition;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.HatchPosition;

public class CompStartCargo implements State
{
  @Override
  public void initialize() {

  }

  @Override
  public State periodic() {
    int hatchAngle = Robot.godSubsystem.getHatch().getAngle();
    int cargoAngle = Robot.godSubsystem.getCargo().getAngle();


    if (HatchPosition.DEPLOY.isClose(cargoAngle) && CargoPosition.SAFE.isClose(hatchAngle)){
      return new HatchHandlingTransition();
    }
    else if(HatchPosition.DEPLOY.inRange(cargoAngle) && CargoPosition.SAFE.inRange(hatchAngle))
    {
      return new HatchHandling();
    }

    return null;
  }

  @Override
  public void finish()
  {

  }
}
