package frc.robot.robot;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.subsystem.SubsystemTargets;
import frc.robot.subsystem.BaseMotorControllerConfig;
import java.util.HashMap;
import org.waltonrobotics.util.RobotConfig;

public abstract class LimitedRobot extends RobotConfig {

  private HashMap<Enum, LimitPair> limits = new HashMap<>();

  public LimitedRobot(String robotName) {
    super(robotName);
  }

  public HashMap<Enum, LimitPair> getLimits() {
    return limits;
  }

  public abstract BaseMotorControllerConfig getCargoSubsystemLimits();

  public abstract BaseMotorControllerConfig getHatchSubsystemLimits();

  public abstract BaseMotorControllerConfig getElevatorSubsystemLimits();

  public abstract SubsystemTargets getTargets();

  public abstract void initLimits();

  public void setCargoLimit(TalonSRX talonSRX, Enum cargoType) {
    getCargoSubsystemLimits().setLimits(talonSRX, limits.get(cargoType));
  }


  public void setHatchLimit(TalonSRX talonSRX, Enum cargoType) {
    getHatchSubsystemLimits().setLimits(talonSRX, limits.get(cargoType));
  }


  public void setElevatorLimit(TalonSRX talonSRX, Enum cargoType) {
    getElevatorSubsystemLimits().setLimits(talonSRX, limits.get(cargoType));
  }


  public int getTarget(String target) {
    return getTargets().getTargets().get(target);
  }

  public void setupTalon(TalonSRX talonSRX, TalonSRXConfig talonSRXConfig, Enum limitType) {
    talonSRX.setNeutralMode(talonSRXConfig.getNeutralMode());
    talonSRX.configSelectedFeedbackSensor(talonSRXConfig.getFeedbackSensor(), talonSRXConfig.getPIDIdx(),
        talonSRXConfig.getTimeout());
    talonSRX.setSensorPhase(talonSRXConfig.getSensorPhase()); // true for comp bot; false for practice
    talonSRX.setInverted(talonSRXConfig.isInverted());

    talonSRX.setStatusFramePeriod(talonSRXConfig.getStatusFramePeriod(), 10, talonSRXConfig.getTimeout());
//    talonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, TIMEOUT);

    talonSRX.configNominalOutputForward(talonSRXConfig.getNominalOutputForward(), talonSRXConfig.getTimeout());
    talonSRX.configNominalOutputReverse(talonSRXConfig.getNominalOutputReverse(), talonSRXConfig.getTimeout());
    talonSRX.configPeakOutputForward(talonSRXConfig.getPeakOutputForward(), talonSRXConfig.getTimeout());
    talonSRX.configPeakOutputReverse(talonSRXConfig.getPeakOutputReverse(), talonSRXConfig.getTimeout());

    talonSRX.selectProfileSlot(talonSRXConfig.getProfileSlot(), talonSRXConfig.getPIDIdx());

    talonSRX.config_kP(talonSRXConfig.getProfileSlot(), talonSRXConfig.getKP(), talonSRXConfig.getTimeout());
    talonSRX.config_kI(talonSRXConfig.getProfileSlot(), talonSRXConfig.getKI(), talonSRXConfig.getTimeout());
    talonSRX.config_kD(talonSRXConfig.getProfileSlot(), talonSRXConfig.getKD(), talonSRXConfig.getTimeout());
    talonSRX.config_kF(talonSRXConfig.getProfileSlot(), talonSRXConfig.getKF(), talonSRXConfig.getTimeout());

    talonSRX.configMotionCruiseVelocity(talonSRXConfig.getMotionCruiseVelocity(), talonSRXConfig.getTimeout());
    talonSRX.configMotionAcceleration(talonSRXConfig.getMotionAcceleration(), talonSRXConfig.getTimeout());

    if (limitType != null) {
      LimitPair limitPair = getLimits().get(limitType);
      talonSRX.configForwardSoftLimitThreshold(limitPair.getForwardsSoftLimitThreshold(), talonSRXConfig.getTimeout());
      talonSRX.configReverseSoftLimitThreshold(limitPair.getReverseSoftLimitThreshold(), talonSRXConfig.getTimeout());

      talonSRX.configForwardSoftLimitEnable(talonSRXConfig.isForwardsSoftLimitEnabled(), 10);
      talonSRX.configReverseSoftLimitEnable(talonSRXConfig.isReverseSoftLimitEnabled(), 10);

      /* pass false to FORCE OFF the feature.  Otherwise the enable flags above are honored */
      talonSRX.overrideLimitSwitchesEnable(talonSRXConfig.isOverrideLimitSwitchesEnabled());
    }
  }
}
