package frc.robot.robotState;

import static frc.robot.Robot.currentRobot;
import static frc.robot.Robot.drivetrain;
import static frc.robot.Robot.godSubsystem;
import static frc.robot.RobotMap.elevatorMotor;

import frc.robot.Robot;
import frc.robot.command.teleop.Drive;
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
public class AutoClimbStage1 implements State {

  private int velcotiy = 400;

  @Override
  public void initialize() {
    Robot.godSubsystem.setCurrentActiveState(ActiveState.CARGO_HANDLING);

    Robot.godSubsystem.getHatch().setCurrentTarget(HatchPosition.SAFE);
    Robot.godSubsystem.getCargo().setCurrentTarget(CargoPosition.CLIMB);
    Robot.godSubsystem.getCargo().setLimits(CargoPosition.CLIMB);

    Robot.godSubsystem.getElevator().setControlMode(ElevatorControlMode.AUTO);
    Robot.godSubsystem.getElevator().setLimits(ElevatorLevel.HATCH_BASE);
    Robot.godSubsystem.getElevator().setElevatorLevel(ElevatorLevel.HATCH_BASE);

    Robot.godSubsystem.getClimber().setClimberControlMode(ClimberControlMode.MANUAL);
    elevatorMotor.configMotionCruiseVelocity(velcotiy);

    Drive.setIsEnabled(false);
  }

  @Override
  public State periodic() {
    if (!Robot.godSubsystem.isEnabled()) {
      return new Disabled();
    }

    if (godSubsystem.isMasterOverride()) {
      return new ClimbHandlingTransition();
    }

    drivetrain.setSpeeds(1, 1);

    int elevatorHeight = Robot.godSubsystem.getElevator().getElevatorHeight();
    int cargoAngle = Robot.godSubsystem.getCargo().getAngle();

    return ((Robot.currentRobot.getTarget(CargoPosition.CLIMB)
        .isClose(cargoAngle, 50) &&
        Robot.currentRobot.getTarget(ElevatorLevel.HATCH_BASE)
            .isClose(elevatorHeight, 250)) && godSubsystem.autoClimbRising()) || Robot.godSubsystem.isMasterOverride() ?
        new AutoClimbStage2() : this;
  }

  @Override
  public void finish() {
    Drive.setIsEnabled(true);
    elevatorMotor.configMotionCruiseVelocity(currentRobot.getElevatorSubsystemLimits().getMotionCruiseVelocity());
  }
}