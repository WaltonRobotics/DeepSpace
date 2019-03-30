package frc.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class VictorPair {

  private final VictorSPX victorSPX1;
  private final int CANId1;
  private final VictorSPX victorSPX2;
  private final int CANId2;

  public VictorPair(int CANId1, int CANId2) {
    this.victorSPX1 = new VictorSPX(CANId1);
    this.CANId1 = CANId1;
    this.victorSPX2 = new VictorSPX(CANId2);
    this.CANId2 = CANId2;
  }

  public VictorSPX getVictorSPX1() {
    return victorSPX1;
  }

  public VictorSPX getVictorSPX2() {
    return victorSPX2;
  }

  public void set(ControlMode controlMode, double percentage) {
    victorSPX2.set(controlMode, percentage);
    victorSPX1.set(controlMode, percentage);
  }

  public void setInverted(boolean inverted) {
    victorSPX1.setInverted(inverted);
    victorSPX2.setInverted(inverted);
  }

  public void configPeakOutputForward(double value) {
    victorSPX1.configPeakOutputForward(value);
    victorSPX2.configPeakOutputForward(value);
  }

  public void configPeakOutputReverse(double value) {
    victorSPX1.configPeakOutputReverse(value);
    victorSPX2.configPeakOutputReverse(value);
  }

  public double getMotorOutputPercent() {
    return victorSPX1.getMotorOutputPercent();
  }

  public void setNeutralMode(NeutralMode neutralMode) {
    victorSPX1.setNeutralMode(neutralMode);
    victorSPX2.setNeutralMode(neutralMode);
  }
}
