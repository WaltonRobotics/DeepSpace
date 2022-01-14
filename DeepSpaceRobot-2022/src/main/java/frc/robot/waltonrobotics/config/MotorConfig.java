package frc.robot.waltonrobotics.config;

/**
 * @author Russell Newton
 **/
public interface MotorConfig {

  int getChannel();

  boolean isInverted();

  MotorParameters getMotorParameters();

}
