package frc.robot.subsystem;

import java.util.HashMap;

public abstract class SusystemTargets {
    private HashMap<String, Double> targets = new HashMap<>();
    abstract void defineTargets();

    public HashMap<String, Double> getTargets() {
        return targets;
    }
}
