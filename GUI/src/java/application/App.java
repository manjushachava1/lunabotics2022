package application;

import javafx.application.Application;
import javafx.fxml.FXMLLoader;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.stage.Stage;

public class App extends Application {

  @Override
  public void init() {
    System.out.println("Starting Up");
  }

  @Override
  public void start(Stage primaryStage) {
    try {
      System.setProperty("prism.lcdtext", "false");

      Parent root = FXMLLoader.load(getClass().getResource("views/GUI.fxml"));

      Scene scene = new Scene(root, 1200, 800);

      /*
      URL logoURL =
          getClass()
              .getClassLoader()
              .getResource("edu/wpi/cs3733/c21/teamE/media/BW_Hospital_Logo.png");

      Image logoImage = SwingFXUtils.toFXImage(ImageIO.read(logoURL), null);
      */

      primaryStage.setTitle("WPI Lunabotics");
      primaryStage.setScene(scene);
      //primaryStage.getIcons().add(logoImage);
      primaryStage.setMinWidth(1200);
      primaryStage.setMinHeight(800);
      primaryStage.show();
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  // Connor is cute <3

  @Override
  public void stop() {
    System.out.println("Shutting Down");
    System.exit(0); // closes any non-JavaFX threads (such as FileChooser)
  }
}
