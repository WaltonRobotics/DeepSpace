package frc.robot;

import static org.hamcrest.CoreMatchers.is;
import static org.opencv.highgui.HighGui.waitKey;

import edu.wpi.first.wpiutil.RuntimeDetector;
import edu.wpi.first.wpiutil.RuntimeLoader;
import frc.robot.util.ParkingLines;
import java.io.File;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Objects;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;
import org.opencv.core.Mat;
import org.opencv.highgui.HighGui;
import org.opencv.videoio.VideoCapture;

public class ParkingLineTest {

  @Before
  public void loadNativeLibraries() {
    Path path = Paths.get(RuntimeLoader.getDefaultExtractionRoot(), RuntimeDetector.getPlatformPath());

    File file = path.toFile();

    for (File nativeLibrary : Objects.requireNonNull(file.listFiles())) {
      System.out.printf("Loading: %s from %s%n", nativeLibrary.getName(), nativeLibrary.getAbsolutePath());
      System.load(nativeLibrary.getAbsolutePath());
    }
  }

  @Test
  public void testParkingLine() {
    VideoCapture camera = new VideoCapture(0);
    if (!camera.isOpened()) {
      Assert.fail();
    }
    Mat mat = new Mat();
    camera.grab();
    camera.retrieve(mat);
    camera.release();

    ParkingLines.setPercentage(0.9);
    ParkingLines.setXOffset(40);
    ParkingLines.setFocusPoint(mat.width() / 2.0, mat.height() * (2.0 / 3.0));
    ParkingLines.drawParkingLines(mat);

    HighGui.imshow("Camera", mat);
    waitKey(2500);

    Assert.assertThat(true, is(true));
  }
}
