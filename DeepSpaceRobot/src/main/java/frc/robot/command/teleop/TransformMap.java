package frc.robot.command.teleop;

import java.util.ArrayList;

public class TransformMap {

    private static ArrayList<Transform> transformArrayList = new ArrayList<>();

    public static void addTransform(Transform t) {
        transformArrayList.add(t);
    }

    public static void removeTransform(Transform t) {
        transformArrayList.remove(t);
    }

    public static void removeAll() {
        transformArrayList.removeAll(transformArrayList);
    }

}
