package frc.robot.command.auton;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import org.waltonrobotics.command.SimpleCameraPositioning;
import org.waltonrobotics.command.SimpleMotion;
import org.waltonrobotics.metadata.CameraData;

public class AutoAssist extends Command {

  private boolean foundTarget = false;

  @Override
  protected void execute() {
    CameraData currentCameraData = SimpleMotion.getDrivetrain().getCurrentCameraData();

    if (currentCameraData.getNumberOfTargets() > 0) {
      if (OI.rightJoystick.getTrigger()) {
        SimpleCameraPositioning.toCameraTarget(currentCameraData).start();
        foundTarget = true;
      }
    }
  }

  @Override
  protected boolean isFinished() {
    return foundTarget;
  }
}
