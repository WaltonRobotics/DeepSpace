package frc.utils;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Date;
import java.util.UUID;

public class CrashDump {

    public static final String CRASH_DUMP_DEFAULT_FILE_PATH = "/home/lvuser/CrashDump.txt";

    private static final UUID RUN_INSTANCE_UUID = UUID.randomUUID();

    public static void logRobotStartup() {
        logMarker("Robot startup");
    }

    public static void logRobotConstruction() {
        logMarker("Robot construction");
    }

    public static void logRobotInit() {
        logMarker("Robot init");
    }

    public static void logTeleopInit() {
        logMarker("Teleop init");
    }

    public static void logAutoInit() {
        logMarker("Auto init");
    }

    public static void logDisabledInit() {
        logMarker("Disabled init");
    }

    public static void logThrowableCrash(Throwable throwable) {
        logMarker("Exception: ", throwable);
    }

    private static void logMarker(String mark) {
        logMarker(mark, null);
    }

    private static void logMarker(String mark, Throwable nullableException) {
        try (PrintWriter writer = new PrintWriter(new FileWriter(CRASH_DUMP_DEFAULT_FILE_PATH, true))) {
            writer.print(RUN_INSTANCE_UUID.toString());
            writer.print(", ");
            writer.print(mark);
            writer.print(", ");
            writer.print(new Date().toString());

            if (nullableException != null) {
                writer.print(", ");
                nullableException.printStackTrace(writer);
            }

            writer.println();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}