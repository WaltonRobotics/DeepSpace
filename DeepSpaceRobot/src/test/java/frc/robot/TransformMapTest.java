package frc.robot;

import frc.robot.command.teleop.Transform;

import java.util.ArrayList;

public class TransformMapTest {

    private static ArrayList<Transform> transformArrayList = new ArrayList<>();

    public static void addTransform(Transform t) {
        transformArrayList.add(t);
    }

    public static void removeTransform(Transform t) {
        transformArrayList.remove(t);
    }

    public static void clearTransforms() {
        transformArrayList.clear();
    }

    public static double calculate(double input) {
        double lastResult = input;

        for (Transform t : transformArrayList) {
            double temp = t.transform(lastResult);
            lastResult = temp;
        }

        return lastResult;
    }

}
