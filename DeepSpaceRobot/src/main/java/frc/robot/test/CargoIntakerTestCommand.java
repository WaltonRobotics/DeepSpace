package frc.robot.test;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.CargoPosition;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ClawControlMode;
import frc.robot.util.TestCommand;

public class CargoIntakerTestCommand extends TestCommand {

  @Override
  protected void initializeTest() {
    Robot.godSubsystem.getCargo().setClawControlMode(ClawControlMode.AUTO);
    Robot.godSubsystem.getCargo().setClawTarget(CargoPosition.DEPLOY);
  }

  @Override
  protected void executeTest() {
    if (!CargoPosition.DEPLOY.isClose(Robot.godSubsystem.getCargo().getAngle())) {
      throw new AssertionError("Cargo angle issue");
    }
    if(Robot.godSubsystem.getCargo().inButtonPressed()){
      if (RobotMap.leftIntakeMotor.get() != 1 || RobotMap.rightIntakeMotor.get() != 1) {
        throw new AssertionError("Intake issue");
      }
    }
    if(Robot.godSubsystem.getCargo().outSlowButtonPressed()){
      if (RobotMap.leftIntakeMotor.get() != -.5 || RobotMap.rightIntakeMotor.get() != -.5) {
        throw new AssertionError("Outtake slow issue");
      }
    }
    if(Robot.godSubsystem.getCargo().outFastButtonPressed()){
      if (RobotMap.leftIntakeMotor.get() != -1 || RobotMap.rightIntakeMotor.get() != -1) {
        throw new AssertionError("Outtake fast issue");
      }
    }
  }

  @Override
  protected void endTest() {

  }

  @Override
  protected boolean isFinished() {
    if(Robot.godSubsystem.getCargo().inButtonPressed() && Robot.godSubsystem.getCargo().outFastButtonPressed() && Robot.godSubsystem.getCargo().outSlowButtonPressed()){
      return true;
    }
    return false;
  }
}
