package frc.robot;

import edu.wpi.cscore.CameraServerJNI;
import edu.wpi.first.wpiutil.RuntimeDetector;
import edu.wpi.first.wpiutil.RuntimeLoader;
import frc.team2974.robot.subsystems.Drivetrain;
import org.junit.Before;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import java.io.File;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Objects;

public class MotorTest
{
    private static void loadNativeLibraries() {
        Path path = Paths.get(RuntimeLoader.getDefaultExtractionRoot(), RuntimeDetector.getPlatformPath());

        File file = path.toFile();

        for (File nativeLibrary : Objects.requireNonNull(file.listFiles())) {
            System.out.printf("Loading: %s from %s%n", nativeLibrary.getName(), nativeLibrary.getAbsolutePath());
            System.load(nativeLibrary.getAbsolutePath());
        }
    }
    @Before
    public void init() {
        loadNativeLibraries();
        CameraServerJNI.enumerateSinks();

    }
    @Test
    public void moveTest(){

    }
}
