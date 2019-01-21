package frc.robot;

import edu.wpi.first.wpiutil.RuntimeDetector;
import edu.wpi.first.wpiutil.RuntimeLoader;
import java.io.File;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Objects;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

public class CameraUndistortedTest {

  public static double[][] k = {{540.6169741181128, 0.0, 979.9714037950379},
      {0.0, 540.4923723031342, 603.6179611901534}, {0.0, 0.0, 1.0}};
  public static double[][] scaled = {{264.74866142, 0., 628.46350396},
      {0., 264.68764195, 375.72956697},
      {0., 0., 1.}};
  public static double[][] d = {{-0.04581054920752462},
      {0.00580921941468853}, {-0.0037112166109776372}, {-0.00015164559682399615}};
  public static double[][] newKK = {{360.41131608, 0., 653.3142692},
      {0., 360.3282482, 402.41197413},
      {0., 0., 1.}};

  public static void loadNativeLibraries() {
    Path path = Paths.get(RuntimeLoader.getDefaultExtractionRoot(), RuntimeDetector.getPlatformPath());

    File file = path.toFile();

    for (File nativeLibrary : Objects.requireNonNull(file.listFiles())) {
      System.out.printf("Loading: %s from %s%n", nativeLibrary.getName(), nativeLibrary.getAbsolutePath());
      System.load(nativeLibrary.getAbsolutePath());
    }
  }

  public static void fillMat(Mat mat, double[][] data) {
    mat.create(data.length, data[0].length, CvType.CV_64F);
    for (int i = 0; i < data.length; i++) {
      mat.put(i, 0, data[i]);
    }
  }

  @Before
  public void initTests() {
    loadNativeLibraries();
  }

  public double[][] getArray(Mat matrix) {
    double[][] data = new double[matrix.rows()][matrix.cols()];

    for (int r = 0; r < matrix.rows(); r++) {
      for (int c = 0; c < matrix.cols(); c++) {
        data[r][c] = matrix.get(r, c)[0];
      }
    }

    return data;
  }

  @Test
  public void kMatTest() {

    Mat k = new Mat();
    fillMat(k, CameraUndistortedTest.k);
    double[][] kData = getArray(k);

    Assert.assertArrayEquals(kData, CameraUndistortedTest.k);
  }

  @Test
  public void dMatTest() {

    Mat d = new Mat();
    fillMat(d, CameraUndistortedTest.d);
    double[][] kData = getArray(d);

    Assert.assertArrayEquals(kData, CameraUndistortedTest.d);
  }

  @Test
  public void scaledKMatTest() {

    Mat scaled = new Mat();
    fillMat(scaled, CameraUndistortedTest.scaled);
    double[][] kData = getArray(scaled);

    Assert.assertArrayEquals(kData, CameraUndistortedTest.scaled);
  }

  @Test
  public void newKMatTest() {

    Mat scaled = new Mat();
    fillMat(scaled, CameraUndistortedTest.newKK);
    double[][] kData = getArray(scaled);

    Assert.assertArrayEquals(kData, newKK);
  }

  @Test
  public void rMatTest() {

    Mat R = Mat.eye(3, 3, CvType.CV_64F);

    double[][] kData = getArray(R);

    double[][] answer = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    Assert.assertArrayEquals(kData, answer);
  }


}
