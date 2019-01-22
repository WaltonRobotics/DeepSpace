package frc.robot;

import static org.opencv.core.Core.BORDER_CONSTANT;
import static org.opencv.core.Core.divide;
import static org.opencv.core.Core.mean;
import static org.opencv.core.CvType.CV_32F;
import static org.opencv.highgui.HighGui.destroyAllWindows;
import static org.opencv.highgui.HighGui.imshow;
import static org.opencv.highgui.HighGui.waitKey;
import static org.opencv.imgcodecs.Imgcodecs.imread;
import static org.opencv.imgproc.Imgproc.GaussianBlur;
import static org.opencv.imgproc.Imgproc.INTER_LINEAR;
import static org.opencv.imgproc.Imgproc.initUndistortRectifyMap;
import static org.opencv.imgproc.Imgproc.remap;

import edu.wpi.first.wpiutil.RuntimeDetector;
import edu.wpi.first.wpiutil.RuntimeLoader;
import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.Objects;
import javax.xml.parsers.ParserConfigurationException;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.xml.sax.SAXException;

public class CameraUndistortedTest {

  private static double[][] k = {{540.6169741181128, 0.0, 979.9714037950379},
      {0.0, 540.4923723031342, 603.6179611901534}, {0.0, 0.0, 1.0}};
  private static double[][] scaled = {{540.6169741181128, 0.0, 979.9714037950379},
      {0.0, 540.4923723031342, 603.6179611901534},
      {0., 0., 1.}};
  private static double[][] d = {{-0.04581054920752462},
      {0.00580921941468853}, {-0.0037112166109776372}, {-0.00015164559682399615}};
  private static double[][] newKK = {{397.1229921356301, 0.0, 942.6952559372105},
      {0.0, 397.0314629237133, 563.5943504492928},
      {0., 0., 1.}};
  private static MatLoader matLoader;
  private static double[][] r = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

  public static void loadNativeLibraries() {
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
  public void initTests() throws ParserConfigurationException, SAXException, IOException {
    loadNativeLibraries();

    matLoader = new MatLoader("fisheye_undistorted.xml");
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

  private Mat getPreloadedScaledKMat() {
    return matLoader.getMat("scaledK");
  }

  private Mat getPreloadedDMat() {
    return matLoader.getMat("D");
  }

  private Mat getPreloadedRMat() {
    return matLoader.getMat("R");
  }

  private Mat getPreloadedNewKMat() {
    return matLoader.getMat("newK");
  }

  private Mat getPreloadedMap1Mat() {
    return matLoader.getMat("map1", CvType.CV_16SC2);
  }

  private Mat getPreloadedMap2Mat() {
    return matLoader.getMat("map2", CvType.CV_16UC1);
  }

  private Mat getPreloadedKMat() {
    return matLoader.getMat("K");
  }

  @Test
  public void preloadedScaledKMatTest() {
    Mat mat = getPreloadedScaledKMat();
    double[][] data = getArray(mat);

    Assert.assertArrayEquals(data, CameraUndistortedTest.scaled);
  }

  @Test
  public void preloadedRMatTest() {
    Mat mat = getPreloadedRMat();
    double[][] data = getArray(mat);

    Assert.assertArrayEquals(data, CameraUndistortedTest.r);
  }

  @Test
  public void preloadedKMatTest() {
    Mat mat = getPreloadedKMat();
    double[][] data = getArray(mat);

    Assert.assertArrayEquals(data, CameraUndistortedTest.k);
  }

  @Test
  public void preloadedNewKMatTest() {
    Mat mat = getPreloadedNewKMat();
    double[][] data = getArray(mat);

    Assert.assertArrayEquals(data, CameraUndistortedTest.newKK);
  }

  @Test
  public void preloadedDMatTest() {
    Mat mat = getPreloadedDMat();
    double[][] data = getArray(mat);

    Assert.assertArrayEquals(data, CameraUndistortedTest.d);
  }


  @Test
  public void kMatTest() {

    Mat mat = getKMat();
    double[][] kData = getArray(mat);

    System.out.println(mat.dump());

    Assert.assertArrayEquals(kData, CameraUndistortedTest.k);
  }

  private Mat getDMat() {
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

  private Mat getScaledKMat() {
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

  private Mat getNewKMat() {
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

  private Mat getRMat() {
    return Mat.eye(3, 3, CvType.CV_64F);
  }

  @Test
  public void rMatTest() {
    Mat mat = getRMat();

    double[][] kData = getArray(mat);

    System.out.println(mat.dump());
    Assert.assertArrayEquals(kData, r);
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

    Mat map1Preloaded = getPreloadedMap1Mat();
    Mat map2Preloaded = getPreloadedMap2Mat();

    System.out.println(Arrays.deepToString(getDeepArray(map1Preloaded)));
    System.out.println(Arrays.deepToString(getDeepArray(map2Preloaded)));

    Assert.assertArrayEquals(getDeepArray(map1Preloaded), getDeepArray(map1));
    Assert.assertArrayEquals(getDeepArray(map2Preloaded), getDeepArray(map2));
  }

  @Test
  public void remapTest() {
    File file = new File("src/test/java/frc/robot/testImages/undistorted.jpeg");
    System.out.println(file.exists());

    Mat undistorted = imread("src/test/java/frc/robot/testImages/undistorted.jpeg");
    Mat distorted = imread("src/test/java/frc/robot/testImages/distorted.jpeg");

    Mat map1Preloaded = getPreloadedMap1Mat();
    Mat map2Preloaded = getPreloadedMap2Mat();

    Mat dest = new Mat();
    remap(distorted, dest, map1Preloaded, map2Preloaded, INTER_LINEAR, BORDER_CONSTANT);

    Assert.assertArrayEquals(getDeepArray(distorted), getDeepArray(dest));
  }

  private double[][][] getDeepArray(Mat matrix) {
    double[][][] data = new double[matrix.rows()][matrix.cols()][matrix.channels()];

    for (int r = 0; r < matrix.rows(); r++) {
      for (int c = 0; c < matrix.cols(); c++) {
        for (int channel = 0; channel < matrix.channels(); channel++) {
          data[r][c] = matrix.get(r, c);
        }
      }
    }

    return data;
  }

  @Test
  public void testMSSIM() {
    Mat mat = imread("src/test/java/frc/robot/testImages/distorted.jpeg");
    Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2GRAY);
    Assert.assertEquals(1, getMSSIM(mat, mat), 0.0);
  }

  private double getMSSIM(Mat i1, Mat i2) {
    final double C1 = 6.5025;
    final double C2 = 58.5225;
    /***************************** INITS **********************************/
    int d = CV_32F;

    Mat I1 = new Mat();
    Mat I2 = new Mat();
    i1.convertTo(I1, d);           // cannot calculate on one byte large values
    i2.convertTo(I2, d);

    Mat I2_2 = I2.mul(I2);        // I2^2
    Mat I1_2 = I1.mul(I1);        // I1^2
    Mat I1_I2 = I1.mul(I2);        // I1 * I2

    /*************************** END INITS **********************************/

    Mat mu1 = new Mat(), mu2 = new Mat();   // PRELIMINARY COMPUTING
    GaussianBlur(I1, mu1, new Size(11, 11), 1.5);
    GaussianBlur(I2, mu2, new Size(11, 11), 1.5);

    Mat mu1_2 = mu1.mul(mu1);
    Mat mu2_2 = mu2.mul(mu2);
    Mat mu1_mu2 = mu1.mul(mu2);

    Mat sigma1_2 = new Mat(), sigma2_2 = new Mat(), sigma12 = new Mat();

    GaussianBlur(I1_2, sigma1_2, new Size(11, 11), 1.5);
    Core.subtract(sigma1_2, mu1_2, sigma1_2);

    GaussianBlur(I2_2, sigma2_2, new Size(11, 11), 1.5);
    Core.subtract(sigma2_2, mu2_2, sigma2_2);

    GaussianBlur(I1_I2, sigma12, new Size(11, 11), 1.5);
    Core.subtract(sigma12, mu1_mu2, sigma12);

    ///////////////////////////////// FORMULA ////////////////////////////////
    Mat t1 = new Mat(), t2 = new Mat(), t3 = new Mat();

    Core.multiply(mu1_mu2, new Scalar(2), t1);
    Core.add(t1, new Scalar(C1), t1);
//    t1 = 2 * mu1_mu2 + C1;

    Core.multiply(sigma12, new Scalar(2), t2);
    Core.add(t2, new Scalar(C2), t2);
//    t2 = 2 * sigma12 + C2;

    t3 = t1.mul(t2);              // t3 = ((2*mu1_mu2 + C1).*(2*sigma12 + C2))

    Core.add(mu1_2, mu2_2, t1);
    Core.add(t1, new Scalar(C1), t1);
//    t1 = mu1_2 + mu2_2 + C1;

    Core.add(sigma1_2, sigma2_2, t2);
    Core.add(t2, new Scalar(C2), t2);
//    t2 = sigma1_2 + sigma2_2 + C2;
    t1 = t1.mul(t2);               // t1 =((mu1_2 + mu2_2 + C1).*(sigma1_2 + sigma2_2 + C2))

    Mat ssim_map = new Mat();
    divide(t3, t1, ssim_map);      // ssim_map =  t3./t1;

    return mean(ssim_map).val[0];
  }
}
