package frc.robot.robotState;

import static frc.robot.Robot.drivetrain;
import static frc.robot.Robot.godSubsystem;

import frc.robot.Robot;
import frc.robot.command.teleop.Drive;
import frc.robot.state.State;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ActiveState;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.CargoPosition;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.Climber;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ElevatorControlMode;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ElevatorLevel;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.HatchPosition;

/**
 * @author Marius Juston
 **/
public class AutoClimbStage3 implements State {


  private Climber climber = Robot.godSubsystem.getClimber();

  private double timeout;

  @Override
  public void initialize() {
    Drive.setIsEnabled(false);
    timeout = godSubsystem.getCurrentTime() + 1000;

    climber.setTimer(3000, -1);
    drivetrain.setSpeeds(.5, .5);

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
    if (godSubsystem.isMasterOverride()) {
      return new ClimbHandlingTransition();
    }

    if (godSubsystem.getCurrentTime() > timeout) {
      return new DefenseTransition();
    }

    return this;
  }

  @Override
  public void finish() {
    drivetrain.setSpeeds(0, 0);
    Drive.setIsEnabled(true);
  }
}
