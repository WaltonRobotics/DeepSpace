package frc.robot;

import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

public class MatLoader {

  private final Document doc;

  public MatLoader(String fileName) throws IOException, SAXException, ParserConfigurationException {
    File fXmlFile = new File(fileName);

    DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
    DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
    doc = dBuilder.parse(fXmlFile);

    //optional, but recommended
    //read this - http://stackoverflow.com/questions/13786607/normalization-in-dom-parsing-with-java-how-does-it-work
    doc.getDocumentElement().normalize();

  }

  public static Mat createMat(int rows, int cols, double[] data, int type) {
    Mat mat = new Mat(rows, cols, type);

    int i = 0;
    for (int r = 0; r < rows; r++) {
      for (int c = 0; c < cols; c++) {

        if (mat.channels() > 1) {
          double[] buffer = new double[mat.channels()];
          for (int y = 0; y < mat.channels(); y++) {
            buffer[y] = data[i++];
          }
          mat.put(r, c, buffer);
        } else {
          mat.put(r, c, data[i++]);
        }
      }
    }
    return mat;
  }


  public Mat getMat(String matName, int type) {
    NodeList nList = doc.getElementsByTagName(matName);

    Node nNode = nList.item(0);

    Mat mat = null;
    if (nNode.getNodeType() == Node.ELEMENT_NODE) {
      Element eElement = (Element) nNode;

      int rows = Integer.parseInt(eElement.getElementsByTagName("rows").item(0).getTextContent());
      int cols = Integer.parseInt(eElement.getElementsByTagName("cols").item(0).getTextContent());
      String dataString = eElement.getElementsByTagName("data").item(0).getTextContent();
      double[] data = Arrays.stream(dataString.trim().split("\\s+")).mapToDouble(Double::parseDouble).toArray();
      mat = createMat(rows, cols, data, type);
    }

    return mat;
  }

  public Mat getMat(String matName) {
    return getMat(matName, CvType.CV_64F);
  }
}
