package application.views;

import java.net.URL;
import java.util.ResourceBundle;

import javafx.fxml.FXML;
import javafx.fxml.Initializable;
import javafx.scene.media.MediaView;
import javafx.scene.media.Media;
import javafx.scene.media.MediaPlayer;
import javafx.scene.layout.AnchorPane;
import javafx.scene.web.WebEngine;
import javafx.scene.web.WebView;

public class GUIController implements Initializable {
	@FXML
	private WebView testPlay;

	private WebEngine engine;

	//private MediaPlayer player;
	//private static final String MEDIA_URL = "one_thing.mp4";

	@Override
	public void initialize(URL location, ResourceBundle resources) {
		engine = testPlay.getEngine();
		engine.loadContent("<iframe width=\"580\" height=\"480\" src=\"https://www.youtube.com/embed/RqFoqi6sDjE\" frameborder=\"0\" allow=\"accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture\" allowfullscreen></iframe>" + "", "text/html");
		//testPlay.getEngine().loadContent("<iframe width=\"560\" height=\"315\" src=\"https://www.youtube.com/embed/GPL5Hkl11IQ\" frameborder=\"0\" allow=\"accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture\" allowfullscreen></iframe>" +
		//		"", "text/html");
		//System.out.println(location.toString());
		//System.out.println(this.getClass().getResource(MEDIA_URL).toExternalForm());
		//player = new MediaPlayer(new Media(this.getClass().getResource(MEDIA_URL).toExternalForm()));
		//player.setAutoPlay(true);
		//media.setMediaPlayer(player);
	}
}
