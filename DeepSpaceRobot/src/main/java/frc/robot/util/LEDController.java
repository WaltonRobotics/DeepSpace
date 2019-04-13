package frc.robot.util;

import static frc.robot.RobotMap.LED1;
import static frc.robot.RobotMap.LED2;

public class LEDController {

  private LEDController() {

  }

  public static void SetLEDAutoAlignMode() {
    LED1.set(true);
    LED2.set(true);
  }

  public static void SetLEDFoundTargetMode() {
    LED1.set(false);
    LED2.set(true);
  }

  public static void SetLEDNoTargetFoundMode() {
    LED1.set(false);
    LED1.set(false);
  }
}
