package frc.robot.config;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import java.util.HashMap;
import org.waltonrobotics.util.RobotConfig;
import org.waltonrobotics.util.TalonConfig;

public abstract class LimitedRobot extends RobotConfig {

  private final HashMap<Enum, LimitPair> limits = new HashMap<>();

  protected LimitedRobot(String robotName) {
    super(robotName);
  }

  public HashMap<Enum, LimitPair> getLimits() {
    return limits;
  }

  public abstract BaseMotorControllerConfig getCargoSubsystemLimits();

  public abstract BaseMotorControllerConfig getHatchSubsystemLimits();

  public abstract BaseMotorControllerConfig getElevatorSubsystemLimits();

  public abstract SubsystemTargets getTargets();

  public abstract TalonConfig getLeftIntakeMotorConfig();

  public abstract TalonConfig getRightIntakeMotorConfig();

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

  public void setupController(BaseMotorController motorController, BaseMotorControllerConfig talonSRXConfig,
      Enum limitType) {
    motorController.setNeutralMode(talonSRXConfig.getNeutralMode());
    motorController.configSelectedFeedbackSensor(talonSRXConfig.getFeedbackSensor(), talonSRXConfig.getPIDIdx(),
        talonSRXConfig.getTimeout());
    motorController.setSensorPhase(talonSRXConfig.getSensorPhase()); // true for comp bot; false for practice
    motorController.setInverted(talonSRXConfig.isInverted());

    motorController.setStatusFramePeriod(talonSRXConfig.getStatusFramePeriod().value, 10, talonSRXConfig.getTimeout());
//    talonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, TIMEOUT);

    motorController.configNominalOutputForward(talonSRXConfig.getNominalOutputForward(), talonSRXConfig.getTimeout());
    motorController.configNominalOutputReverse(talonSRXConfig.getNominalOutputReverse(), talonSRXConfig.getTimeout());
    motorController.configPeakOutputForward(talonSRXConfig.getPeakOutputForward(), talonSRXConfig.getTimeout());
    motorController.configPeakOutputReverse(talonSRXConfig.getPeakOutputReverse(), talonSRXConfig.getTimeout());

    motorController.selectProfileSlot(talonSRXConfig.getProfileSlot(), talonSRXConfig.getPIDIdx());

    motorController.config_kP(talonSRXConfig.getProfileSlot(), talonSRXConfig.getKP(), talonSRXConfig.getTimeout());
    motorController.config_kI(talonSRXConfig.getProfileSlot(), talonSRXConfig.getKI(), talonSRXConfig.getTimeout());
    motorController.config_kD(talonSRXConfig.getProfileSlot(), talonSRXConfig.getKD(), talonSRXConfig.getTimeout());
    motorController.config_kF(talonSRXConfig.getProfileSlot(), talonSRXConfig.getKF(), talonSRXConfig.getTimeout());

    motorController.configMotionCruiseVelocity(talonSRXConfig.getMotionCruiseVelocity(), talonSRXConfig.getTimeout());
    motorController.configMotionAcceleration(talonSRXConfig.getMotionAcceleration(), talonSRXConfig.getTimeout());

    if (limitType != null) {
      LimitPair limitPair = getLimits().get(limitType);
      motorController
          .configForwardSoftLimitThreshold(limitPair.getForwardsSoftLimitThreshold(), talonSRXConfig.getTimeout());
      motorController
          .configReverseSoftLimitThreshold(limitPair.getReverseSoftLimitThreshold(), talonSRXConfig.getTimeout());

      motorController.configForwardSoftLimitEnable(talonSRXConfig.isForwardsSoftLimitEnabled(), 10);
      motorController.configReverseSoftLimitEnable(talonSRXConfig.isReverseSoftLimitEnabled(), 10);

      /* pass false to FORCE OFF the feature.  Otherwise the enable flags above are honored */
      motorController.overrideLimitSwitchesEnable(talonSRXConfig.isOverrideLimitSwitchesEnabled());
    }
  }
}
