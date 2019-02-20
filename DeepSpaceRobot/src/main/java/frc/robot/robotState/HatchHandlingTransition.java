package frc.robot.robotState;

import frc.robot.Robot;
import frc.robot.config.Target;
import frc.robot.state.State;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ActiveState;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.CargoPosition;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ElevatorControlMode;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ElevatorLevel;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.HatchPosition;

public class HatchHandlingTransition implements State {

  @Override
  public void initialize() {
    Robot.godSubsystem.setCurrentActiveState(ActiveState.HATCH_HANDLING);
    Robot.godSubsystem.getCargo().setClawTarget(CargoPosition.SAFE);
    Robot.godSubsystem.getHatch().setHatchTarget(HatchPosition.DEPLOY);
    Robot.godSubsystem.getHatch().setLimits(HatchPosition.DEPLOY);

    Robot.godSubsystem.getElevator().setElevatorControlMode(ElevatorControlMode.AUTO);
    Robot.godSubsystem.getElevator().setElevatorLevel(ElevatorLevel.HATCH_BASE);
    Robot.godSubsystem.getElevator().setLimits(ElevatorLevel.HATCH_BASE);
  }

  @Override
  public State periodic() {

    if (!Robot.godSubsystem.isEnabled()) {
      return new Disabled();
    }
    int cargoAngle = Robot.godSubsystem.getCargo().getAngle();
    int hatchAngle = Robot.godSubsystem.getHatch().getAngle();

    if (Robot.currentRobot.getTarget(CargoPosition.SAFE).isClose(cargoAngle) && Robot.currentRobot
        .getTarget(HatchPosition.DEPLOY).isClose(hatchAngle)) {
      return new HatchHandling();
    }

    return this;
  }

  @Override
  public void finish() {

  }
}
