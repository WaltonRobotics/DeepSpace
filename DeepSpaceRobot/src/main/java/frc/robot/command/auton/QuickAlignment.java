package frc.robot.command.auton;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import org.waltonrobotics.command.SimpleMotion;
import org.waltonrobotics.controller.CameraData;


public class QuickAlignment extends Command {


  private boolean foundTarget = false;


  @Override
  protected void execute() {
    CameraData currentCameraData = SimpleMotion.getDrivetrain().getCurrentCameraData();

//    double targetAngle = Math.atan2(currentCameraData.getCameraPose().getY(), currentCameraData.getCameraPose().getX());
//    double startAngle = currentCameraData.getCameraPose().getAngle();


    if (currentCameraData.getNumberOfTargets() > 0) {
      if (OI.rightJoystick.getTrigger()) {
        SimpleAlignmentDrive.fixAlignment(currentCameraData.getCameraPose().getAngle());
        SimpleAlignmentDrive.forwardPowerDrive(Math.abs(currentCameraData.getCameraPose().getX()));
        foundTarget = true;
      }
    }
  }

  @Override
  protected boolean isFinished() {
    return foundTarget;
  }
}

