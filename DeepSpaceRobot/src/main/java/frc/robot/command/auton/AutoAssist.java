package frc.robot.command.auton;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import org.waltonrobotics.command.SimpleMotion;
import org.waltonrobotics.controller.Pose;
import org.waltonrobotics.motion.Path;

public class AutoAssist extends Command {

  public static boolean isValidRobotPosition(Pose robotPosition) {
    double x = robotPosition.getX();
    double y = robotPosition.getY();

    double r = Path.getRobotWidth();

    double result;
    if (x <= r) {
      result = Math.sqrt(Math.pow(r, 2) - Math.pow(x - r, 2));
    } else {
      result = r;
    }
    return result >= y;
  }

  @Override
  protected void execute() {
    boolean isValidPosition = isValidRobotPosition(SimpleMotion.getDrivetrain().getActualPosition());

    if (isValidPosition) {
      if (OI.rightJoystick.getTrigger()){

      }
    }
  }

  @Override
  protected boolean isFinished() {
    return false;
  }
}
