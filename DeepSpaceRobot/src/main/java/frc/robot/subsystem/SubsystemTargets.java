package frc.robot.subsystem;

import java.util.HashMap;

public abstract class SubsystemTargets {

  private final HashMap<String, Integer> targets = new HashMap<>();

  abstract void defineTargets();

  public HashMap<String, Integer> getTargets() {
    return targets;
  }
}
