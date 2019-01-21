package frc.robot;

import static org.opencv.imgproc.Imgproc.initUndistortRectifyMap;

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
import org.opencv.core.Size;

public class CameraUndistortedTest {

  private static double[][] k = {{540.6169741181128, 0.0, 979.9714037950379},
      {0.0, 540.4923723031342, 603.6179611901534}, {0.0, 0.0, 1.0}};
  private static double[][] scaled = {{264.74866142, 0., 628.46350396},
      {0., 264.68764195, 375.72956697},
      {0., 0., 1.}};
  private static double[][] d = {{-0.04581054920752462},
      {0.00580921941468853}, {-0.0037112166109776372}, {-0.00015164559682399615}};
  private static double[][] newKK = {{360.41131608, 0., 653.3142692},
      {0., 360.3282482, 402.41197413},
      {0., 0., 1.}};

  private static void loadNativeLibraries() {
    Path path = Paths.get(RuntimeLoader.getDefaultExtractionRoot(), RuntimeDetector.getPlatformPath());

    File file = path.toFile();

    for (File nativeLibrary : Objects.requireNonNull(file.listFiles())) {
      System.out.printf("Loading: %s from %s%n", nativeLibrary.getName(), nativeLibrary.getAbsolutePath());
      System.load(nativeLibrary.getAbsolutePath());
    }
  }

  private static void fillMat(Mat mat, double[][] data) {
    mat.create(data.length, data[0].length, CvType.CV_64F);
    for (int i = 0; i < data.length; i++) {
      mat.put(i, 0, data[i]);
    }
  }

  @Before
  public void initTests() {
    loadNativeLibraries();
  }

  private double[][] getArray(Mat matrix) {
    double[][] data = new double[matrix.rows()][matrix.cols()];

    for (int r = 0; r < matrix.rows(); r++) {
      for (int c = 0; c < matrix.cols(); c++) {
        data[r][c] = matrix.get(r, c)[0];
      }
    }

    return data;
  }

  private Mat getKMat() {
    Mat mat = new Mat();
    fillMat(mat, CameraUndistortedTest.k);
    return mat;
  }

  @Test
  public void kMatTest() {

    Mat mat = getKMat();
    double[][] kData = getArray(mat);

    System.out.println(mat.dump());

    Assert.assertArrayEquals(kData, CameraUndistortedTest.k);
  }


  public Mat getDMat() {
    Mat mat = new Mat();
    fillMat(mat, CameraUndistortedTest.d);
    return mat;
  }

  @Test
  public void dMatTest() {

    Mat mat = getDMat();
    double[][] kData = getArray(mat);

    System.out.println(mat.dump());

    Assert.assertArrayEquals(kData, CameraUndistortedTest.d);
  }

  public Mat getScaledKMat() {
    Mat mat = new Mat();
    fillMat(mat, CameraUndistortedTest.scaled);
    return mat;
  }

  @Test
  public void scaledKMatTest() {

    Mat mat = getScaledKMat();
    double[][] kData = getArray(mat);

    System.out.println(mat.dump());

    Assert.assertArrayEquals(kData, CameraUndistortedTest.scaled);
  }

  public Mat getNewKMat() {
    Mat mat = new Mat();
    fillMat(mat, CameraUndistortedTest.newKK);
    return mat;
  }

  @Test
  public void newKMatTest() {

    Mat mat = getNewKMat();
    double[][] kData = getArray(mat);

    System.out.println(mat.dump());
    Assert.assertArrayEquals(kData, newKK);
  }

  Mat getRMat() {
    return Mat.eye(3, 3, CvType.CV_64F);
  }

  @Test
  public void rMatTest() {
    Mat mat = getRMat();

    double[][] kData = getArray(mat);

    double[][] answer = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

    System.out.println(mat.dump());
    Assert.assertArrayEquals(kData, answer);
  }

  @Test
  public void initUndistortRectifyMapTest() {
    Mat scaledK = getScaledKMat();
    Mat D = getDMat();
    Mat R = getRMat();
    Mat newK = getNewKMat();
    Size size = new Size(1920, 1080);

    Mat map1 = new Mat();
    Mat map2 = new Mat();
    initUndistortRectifyMap(scaledK, D, R, newK, size, CvType.CV_16SC2, map1, map2);

    Assert.assertTrue(true);
  }

}
