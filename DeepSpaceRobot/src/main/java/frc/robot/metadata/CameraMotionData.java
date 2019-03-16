package frc.robot.metadata;

import org.waltonrobotics.metadata.CameraData;
import org.waltonrobotics.metadata.ErrorVector;
import org.waltonrobotics.metadata.MotionData;
import org.waltonrobotics.metadata.MotionState;
import org.waltonrobotics.metadata.Pose;
import org.waltonrobotics.metadata.RobotPair;

/**
 * @author Marius Juston
 **/
public class CameraMotionData extends MotionData {

  private final CameraData cameraPosition;
  private final Pose offset;

  public CameraMotionData(Pose actual, Pose target,
      ErrorVector error, RobotPair powers, int pathNumber,
      MotionState currentMotionState, CameraData cameraPosition, Pose offset) {
    super(actual, target, error, powers, pathNumber, currentMotionState);
    this.cameraPosition = cameraPosition;
    this.offset = offset;
  }

  public CameraData getCameraPosition() {
    return cameraPosition;
  }

  public Pose getOffset() {
    return offset;
  }

  @Override
  public String toString() {
    return "CameraMotionData{" +
        "cameraPosition=" + cameraPosition +
        ", offset=" + offset +
        "} " + super.toString();
  }
}
