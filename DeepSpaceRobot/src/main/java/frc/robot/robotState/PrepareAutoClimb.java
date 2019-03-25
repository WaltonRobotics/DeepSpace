package frc.robot.robotState;


import static frc.robot.Robot.godSubsystem;

import frc.robot.Robot;
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
public class PrepareAutoClimb implements State {

  @Override
  public void initialize() {
    godSubsystem.setCurrentActiveState(ActiveState.CARGO_HANDLING);

    godSubsystem.getHatch().setCurrentTarget(HatchPosition.SAFE);
    godSubsystem.getCargo().setCurrentTarget(CargoPosition.CLIMB);
    godSubsystem.getCargo().setLimits(CargoPosition.CLIMB);

    godSubsystem.getElevator().setControlMode(ElevatorControlMode.AUTO);
    godSubsystem.getElevator().setElevatorLevel(ElevatorLevel.CLIMB);
    godSubsystem.getElevator().setLimits(ElevatorLevel.CARGO_BASE);

    godSubsystem.getClimber().setClimberControlMode(ClimberControlMode.MANUAL);
  }

  @Override
  public State periodic() {
    if (!godSubsystem.isEnabled()) {
      return new Disabled();
    }

    if (godSubsystem.isMasterOverride()) {
      return new ClimbHandlingTransition();
    }

    int elevatorHeight = godSubsystem.getElevator().getElevatorHeight();
    int cargoAngle = godSubsystem.getCargo().getAngle();

    return ((Robot.currentRobot.getTarget(CargoPosition.CLIMB)
        .isClose(cargoAngle, 50) &&
        Robot.currentRobot.getTarget(ElevatorLevel.CLIMB)
            .isClose(elevatorHeight, 250)) && godSubsystem.autoClimbRising()) || Robot.godSubsystem.isMasterOverride() ?
        new AutoClimbStage1() : this;

  }

  @Override
  public void finish() {

  }
}