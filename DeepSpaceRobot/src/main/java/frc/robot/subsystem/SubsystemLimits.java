package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public abstract class SubsystemLimits {
  abstract NeutralMode getNeutralMode();
  abstract FeedbackDevice getFeedbackSensor();
  abstract boolean getSensorPhase();
  abstract boolean isInverted();

  abstract StatusFrameEnhanced getStatusFramePeriod();

  abstract double getNominalOutputForward();
  abstract double getNominalOutputReverse();
  abstract double getPeakOutputForward();
  abstract double getPeakOutputReverse();

  abstract int getProfileSlot();

  abstract double getKP();
  abstract double getKI();
  abstract double getKD();
  abstract double getKF();

  abstract int getTimeout();
  abstract int getPIDIdx();

  /**
   * Sensor Units per 100ms
   * @return
   */
  abstract int getMotionCruiseVelocity();
  abstract int getMotionAcceleration();

  abstract int getForwardsSoftLimitThreshold();
  abstract int getReverseSoftLimitThreshold();

  abstract boolean isReverseSoftLimitEnabled();
  abstract boolean isForwardsSoftLimitEnabled();

  abstract boolean isOverrideLimitSwitchesEnabled();

  public void setupTalon(TalonSRX talonSRX){
    talonSRX.setNeutralMode(getNeutralMode());
    talonSRX.configSelectedFeedbackSensor(getFeedbackSensor(), getPIDIdx(), getTimeout());
    talonSRX.setSensorPhase(getSensorPhase()); // true for comp bot; false for practice
    talonSRX.setInverted(isInverted());

    talonSRX.setStatusFramePeriod(getStatusFramePeriod(), 10, getTimeout());
//    talonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, TIMEOUT);

    talonSRX.configNominalOutputForward(getNominalOutputForward(), getTimeout());
    talonSRX.configNominalOutputReverse(getNominalOutputReverse(), getTimeout());
    talonSRX.configPeakOutputForward(getPeakOutputForward(), getTimeout());
    talonSRX.configPeakOutputReverse(getPeakOutputReverse(), getTimeout());

    talonSRX.selectProfileSlot(getProfileSlot(), getPIDIdx());

    talonSRX.config_kP(getProfileSlot(), getKP(), getTimeout());
    talonSRX.config_kI(getProfileSlot(), getKI(), getTimeout());
    talonSRX.config_kD(getProfileSlot(), getKD(), getTimeout());
    talonSRX.config_kF(getProfileSlot(), getKF(), getTimeout());

    talonSRX.configMotionCruiseVelocity(getMotionCruiseVelocity(), getTimeout());
    talonSRX.configMotionAcceleration(getMotionAcceleration(), getTimeout());

    talonSRX.configForwardSoftLimitThreshold(getForwardsSoftLimitThreshold(), getTimeout());
    talonSRX.configReverseSoftLimitThreshold(getReverseSoftLimitThreshold(), getTimeout());

    talonSRX.configForwardSoftLimitEnable(isForwardsSoftLimitEnabled(), 10);
    talonSRX.configReverseSoftLimitEnable(isReverseSoftLimitEnabled(), 10);

    /* pass false to FORCE OFF the feature.  Otherwise the enable flags above are honored */
    talonSRX.overrideLimitSwitchesEnable(isOverrideLimitSwitchesEnabled());
  }
}
