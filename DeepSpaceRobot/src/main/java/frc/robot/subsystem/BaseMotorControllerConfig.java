package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.robot.LimitPair;

public abstract class BaseMotorControllerConfig {

  public abstract int getDeviceID();

  public abstract NeutralMode getNeutralMode();

  public abstract FeedbackDevice getFeedbackSensor();

  public abstract boolean getSensorPhase();

  public abstract boolean isInverted();

  public abstract StatusFrameEnhanced getStatusFramePeriod();

  public abstract double getNominalOutputForward();

  public abstract double getNominalOutputReverse();

  public abstract double getPeakOutputForward();

  public abstract double getPeakOutputReverse();

  public abstract int getProfileSlot();

  public abstract double getKP();

  public abstract double getKI();

  public abstract double getKD();

  public abstract double getKF();

  public abstract int getTimeout();

  public abstract int getPIDIdx();

  /**
   * Sensor Units per 100ms
   */
  public abstract int getMotionCruiseVelocity();

  public abstract int getMotionAcceleration();

  public abstract boolean isReverseSoftLimitEnabled();

  public abstract boolean isForwardsSoftLimitEnabled();

  public void setForwardsSoftLimitThreshold(TalonSRX talonSRX, int newForwardsSoftLimitThreshold) {
    talonSRX.configForwardSoftLimitThreshold(newForwardsSoftLimitThreshold, getTimeout());
  }


  public void setReverseSoftLimitThreshold(TalonSRX talonSRX, int newReverseSoftLimitThreshold) {
    talonSRX.configReverseSoftLimitThreshold(newReverseSoftLimitThreshold, getTimeout());
  }

  public void setLimits(TalonSRX talonSRX, LimitPair limits) {
    setForwardsSoftLimitThreshold(talonSRX, limits.getForwardsSoftLimitThreshold());
    setReverseSoftLimitThreshold(talonSRX, limits.getReverseSoftLimitThreshold());
  }

  public abstract boolean isOverrideLimitSwitchesEnabled();
}
