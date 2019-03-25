package frc.robot.robotState;

import static frc.robot.Robot.drivetrain;
import static frc.robot.Robot.godSubsystem;

import frc.robot.Robot;
import frc.robot.command.teleop.Drive;
import frc.robot.state.State;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.CargoPosition;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.Climber;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ClimberControlMode;

/**
 * @author Marius Juston
 **/
public class AutoClimbStage2 implements State {

  private Climber climber = Robot.godSubsystem.getClimber();

//  private long timeout;

  @Override
  public void initialize() {
    Drive.setIsEnabled(false);

    Robot.godSubsystem.getCargo().setCurrentTarget(CargoPosition.SAFE);
    Robot.godSubsystem.getCargo().setLimits(CargoPosition.SAFE);

    climber.setTimer(3000, 1);
//    timeout = godSubsystem.getCurrentTime() + 2000;
    climber.setClimberControlMode(ClimberControlMode.TIMED);
  }

  @Override
  public State periodic() {
    if (!Robot.godSubsystem.isEnabled()) {
      return new Disabled();
    }

    if (godSubsystem.isMasterOverride()) {
      return new ClimbHandlingTransition();
    }

    if (climber.getClimberControlMode() == ClimberControlMode.MANUAL) {
      climber.setClimberPower(.5);
    }

    drivetrain.setSpeeds(1, 1);
    if (
//        godSubsystem.getCurrentTime() > timeout ||
        godSubsystem.autoClimbRising()
            || Robot.godSubsystem.isMasterOverride()) {
      return new AutoClimbStage3();
    }
    return this;
  }

  @Override
  public void finish() {
    Drive.setIsEnabled(true);
    drivetrain.setSpeeds(0, 0);
  }
}