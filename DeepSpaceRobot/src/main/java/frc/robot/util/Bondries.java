package frc.robot.util;

import org.waltonrobotics.controller.Pose;

public class Bondries {

  public static boolean isValidRobotPosition(BoundaryFunction boundaryFunction, Pose robotPosition) {

    System.out.println(boundaryFunction.apply(-robotPosition.getX()));
    return boundaryFunction.apply(-robotPosition.getX()) <= robotPosition.getY();
  }

  public static void main(String[] args) {

    System.out.println(isValidRobotPosition(BoundaryFunction.ABS_SQRT, new Pose(0, 0)));
  }
}
