package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import java.util.ArrayList;
import java.util.Collections;
import org.waltonrobotics.util.RobotConfig;

public class RobotBuilder {

  private ArrayList<RobotConfig> robotConfigs = new ArrayList<>();

  public RobotBuilder(ArrayList<RobotConfig> robotConfigs) {
    this.robotConfigs = robotConfigs;
  }

  public RobotBuilder(RobotConfig... robotConfigs) {
    Collections.addAll(this.robotConfigs, robotConfigs);
  }

  public ArrayList<RobotConfig> getRobotConfigs() {
    return robotConfigs;
  }

  public RobotConfig getCurrentRobotConfig() {
    return robotConfigs.stream().filter(RobotConfig::isCurrentRobot).findFirst().orElse(null);
  }

  public SendableChooser<RobotConfig> getSendableList() {
    SendableChooser<RobotConfig> robotConfigSendableChooser = new SendableChooser<>();
    for (RobotConfig robotConfig : robotConfigs) {
      robotConfigSendableChooser.addOption(robotConfig.getRobotName(), robotConfig);
    }

    RobotConfig currentRobot = getCurrentRobotConfig();
    robotConfigSendableChooser.setDefaultOption(currentRobot.getRobotName(), currentRobot);

    return robotConfigSendableChooser;

  }
}
