package frc.robot.robotState;

import frc.robot.Robot;
import frc.robot.state.State;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ActiveState;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.CargoPosition;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ElevatorControlMode;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ElevatorLevel;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.HatchPosition;

public class DefenseTransition implements State {


  @Override
  public void initialize() {
    Robot.godSubsystem.setCurrentActiveState(ActiveState.DEFENSE);
    Robot.godSubsystem.getHatch().setCurrentTarget(HatchPosition.DEFENSE);
    Robot.godSubsystem.getHatch().setLimits(HatchPosition.SAFE);
    Robot.godSubsystem.getCargo().setCurrentTarget(CargoPosition.SAFE);
    Robot.godSubsystem.getCargo().setLimits(CargoPosition.SAFE);

    Robot.godSubsystem.getElevator().setControlMode(ElevatorControlMode.AUTO);
    Robot.godSubsystem.getElevator().setElevatorLevel(ElevatorLevel.HATCH_BASE);
    Robot.godSubsystem.getElevator().setLimits(ElevatorLevel.HATCH_BASE);
  }

  @Override
  public State periodic() {

    if (!Robot.godSubsystem.isEnabled()) {
      return new Disabled();
    }

    if (Robot.godSubsystem.climbModeRising()) {
      return new ClimbHandlingTransition();
    }

    int cargoAngle = Robot.godSubsystem.getCargo().getAngle();
    int hatchAngle = Robot.godSubsystem.getHatch().getAngle();

    return (Robot.currentRobot.getTarget(CargoPosition.SAFE).isClose(cargoAngle, 50) && Robot.currentRobot
        .getTarget(HatchPosition.SAFE).isClose(hatchAngle, 50)) || Robot.godSubsystem.isMasterOverride() ? new Defense()
        : this;
  }

  @Override
  public void finish() {

  }
}
