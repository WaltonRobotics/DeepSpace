package frc.robot.test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.Hatch;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.HatchPosition;
import frc.robot.util.TestCommand;

public class HatchIntakerTestCommand extends TestCommand {

  @Override
  protected void initializeTest() {

    //Angle tests
    Robot.godSubsystem.getHatch().setHatchAngle(HatchPosition.DEPLOY.getAngle());
    if (HatchPosition.DEPLOY.inRange(Robot.godSubsystem.getHatch().getAngle())) {
      throw new AssertionError("Hatch deploy angle issue");
    }

    Robot.godSubsystem.getHatch().setHatchAngle(HatchPosition.SAFE.getAngle());
    if (HatchPosition.SAFE.inRange(Robot.godSubsystem.getHatch().getAngle())) {
      throw new AssertionError("Hatch safe angle issue");
    }

    Robot.godSubsystem.getHatch().setHatchAngle(HatchPosition.HATCH_START.getAngle());
    if (HatchPosition.HATCH_START.inRange(Robot.godSubsystem.getHatch().getAngle())) {
      throw new AssertionError("Hatch hatch start angle issue");
    }

    Robot.godSubsystem.getHatch().setHatchAngle(HatchPosition.CARGO_START.getAngle());
    if (HatchPosition.CARGO_START.inRange(Robot.godSubsystem.getHatch().getAngle())) {
      throw new AssertionError("Hatch safe angle issue");
    }

    //Intake tests
    Robot.godSubsystem.getHatch().setIntake(true);
    if ((!Robot.godSubsystem.getHatch().getIntakeIsSet())) {
      throw new AssertionError();
    }

    Robot.godSubsystem.getHatch().setIntake(false);
  }

  @Override
  protected void executeTest()
  {

  }

  @Override
  protected void endTest() {

  }

  @Override
  protected boolean isFinished() {
    return true;
  }
}
