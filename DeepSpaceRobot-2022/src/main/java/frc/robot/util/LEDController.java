package frc.robot.util;

import static frc.robot.RobotMap.LED1;
import static frc.robot.RobotMap.LED2;

public final class LEDController {

  private LEDController() {

  }

  public static void setLEDAutoAlignMode() {
    LED1.set(true);
    LED2.set(true);
  }

  public static void setLEDFoundTargetMode() {
    LED1.set(false);
    LED2.set(true);
  }

  public static void setLEDNoTargetFoundMode() {
    LED1.set(false);
    LED2.set(false);
  }
}
