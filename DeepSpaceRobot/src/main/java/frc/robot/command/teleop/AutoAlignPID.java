package frc.robot.command.teleop;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDInterface;

public class AutoAlignPID implements PIDInterface {

 // public PIDController visionController = new PIDController(getP(), getI(), getD(),);

  @Override
  public void setPID(double v, double v1, double v2) {

  }

  @Override
  public double getP() {
    return 0;
  }

  @Override
  public double getI() {
    return 0;
  }

  @Override
  public double getD() {
    return 0;
  }

  @Override
  public void setSetpoint(double v) {

  }

  @Override
  public double getSetpoint() {
    return 0;
  }

  @Override
  public double getError() {
    return 0;
  }

  @Override
  public void reset() {

  }
}
