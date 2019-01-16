package frc.robot.util;

import javafx.application.Application;
import javafx.scene.Group;
import javafx.scene.Scene;
import javafx.scene.layout.Pane;
import javafx.scene.shape.Arc;
import javafx.scene.shape.Circle;
import javafx.scene.shape.Line;
import javafx.stage.Stage;
import org.waltonrobotics.controller.Pose;

public class View extends Application {

  public static final int WIDTH = 800;
  public static final int HEIGHT = 200;
  public static final int dx = 1;
  public static final double SCALE = 1.0;

  public static void main(String[] args) {
    launch(args);
  }

  @Override
  public void start(Stage primaryStage) {
    Pane pane = new Pane();

    Pose robotPosition = new Pose(100, 100);

    double robotWidth = 100.0;
//    pane.getChildren().add(new Circle(robotPosition.getX(), -robotPosition.getY(), robotWidth));
    pane.getChildren().add(new Circle(3));

    Line line = new Line(robotPosition.getX(), -robotWidth / 2, robotWidth / 2, -robotWidth / 2);

    Arc arc = new Arc(robotWidth / 2, 0, 0, 0, 180, robotWidth / 2);

    pane.getChildren().addAll(line, arc);

    pane.scaleShapeProperty().set(false);

    addBoundryFunctions(pane);

    Scene scene = new Scene(pane, WIDTH, HEIGHT);
    pane.translateYProperty().set(HEIGHT);
    pane.translateXProperty().set(WIDTH / 2.0);

    primaryStage.setTitle("Boundaries");
    primaryStage.setScene(scene);
    primaryStage.show();
  }

  private void addBoundryFunctions(Pane pane) {
    for (BoundaryFunction boundaryFunction : BoundaryFunction.values()) {
      Group points = new Group();

      for (double i = -WIDTH / 2.0 * SCALE; i <= WIDTH / 2.0 * SCALE; i += dx) {
        double x = i / SCALE;

        points.getChildren().add(new Circle(x, -boundaryFunction.apply(-x) * SCALE, 1, boundaryFunction.getColor()));
      }

      pane.getChildren().add(points);
    }
  }
}
