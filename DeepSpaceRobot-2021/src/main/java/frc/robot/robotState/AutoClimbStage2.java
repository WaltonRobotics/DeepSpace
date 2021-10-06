package frc.robot.robotState;

import static frc.robot.Robot.drivetrain;
import static frc.robot.Robot.godSubsystem;

import frc.robot.command.teleop.Drive;
import frc.robot.state.State;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.CargoPosition;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.Climber;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ClimberControlMode;

/**
 * @author Marius Juston
 **/
public class AutoClimbStage2 implements State {

  private final Climber climber = godSubsystem.getClimber();

  @Override
  public void initialize() {
    Drive.setIsEnabled(false);

    godSubsystem.getCargo().setCurrentTarget(CargoPosition.SAFE);
    godSubsystem.getCargo().setLimits(CargoPosition.SAFE);

    climber.setTimer(3000, 1.0);
//    timeout = godSubsystem.getCurrentTime() + 2000;
    climber.setClimberControlMode(ClimberControlMode.TIMED);
  }
//  private long timeout;

  @Override
  public State periodic() {
    if (!godSubsystem.isEnabled()) {
      return new Disabled();
    }

    if (godSubsystem.isMasterOverride()) {
      return new ClimbHandlingTransition();
    }

    if (climber.getClimberControlMode() == ClimberControlMode.MANUAL) {
      climber.setClimberPower(0.5);
    }

    drivetrain.setSpeeds(1.0, 1.0);
    if (
//        godSubsystem.getCurrentTime() > timeout ||
        godSubsystem.autoClimbRising()
            || godSubsystem.isMasterOverride()) {
      return new AutoClimbStage3();
    }
    return this;
  }

  @Override
  public void finish() {
    Drive.setIsEnabled(true);
    drivetrain.setSpeeds(0, 0);
  }

  @Override
  public String toString() {
    return "AutoClimbStage2{" +
        "climber=" + climber +
        '}';
  }
}