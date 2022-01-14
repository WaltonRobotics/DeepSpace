package frc.robot.lib.config;

/**
 * @author Russell Newton
 **/
public interface MotorConfig {

  int getChannel();

  boolean isInverted();

  MotorParameters getMotorParameters();

}
