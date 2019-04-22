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

  public static final Target CARGO_SAFE = Robot.currentRobot.getTarget(CargoPosition.SAFE);
  public static final Target HATCH_ANGLE = Robot.currentRobot.getTarget(HatchPosition.DEPLOY);

  @Override
  public void initialize() {
    Robot.godSubsystem.setCurrentActiveState(ActiveState.HATCH_HANDLING);
    Robot.godSubsystem.getCargo().setCurrentTarget(CargoPosition.SAFE);
    Robot.godSubsystem.getHatch().setCurrentTarget(HatchPosition.DEPLOY);
    Robot.godSubsystem.getHatch().setLimits(HatchPosition.DEPLOY);

    Robot.godSubsystem.getElevator().setControlMode(ElevatorControlMode.AUTO);
    Robot.godSubsystem.getElevator().setElevatorLevel(ElevatorLevel.HATCH_BASE);
    Robot.godSubsystem.getElevator().setLimits(ElevatorLevel.HATCH_BASE);
  }

  @Override
  public State periodic() {

    if (!Robot.godSubsystem.isEnabled()) {
      return new Disabled();
    }

    if (Robot.godSubsystem.isMasterOverride()) {
      return new HatchHandling();
    }

    int cargoAngle = Robot.godSubsystem.getCargo().getAngle();
    int hatchAngle = Robot.godSubsystem.getHatch().getAngle();

    if (CARGO_SAFE.isClose(cargoAngle, 50) && HATCH_ANGLE.isClose(hatchAngle, 50)) {
      return new HatchHandling();
    }

    return this;
  }

  @Override
  public void finish() {

  }
}
