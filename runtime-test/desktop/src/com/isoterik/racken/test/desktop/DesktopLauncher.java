package com.isoterik.racken.test.desktop;

import com.badlogic.gdx.backends.lwjgl.LwjglApplication;
import com.badlogic.gdx.backends.lwjgl.LwjglApplicationConfiguration;
import com.isoterik.racken.test.TestDriver;

public class DesktopLauncher {
	public static void main (String[] arg) {
		LwjglApplicationConfiguration config = new LwjglApplicationConfiguration();
		//config.width = 1280;
		//config.height = 720;

		new LwjglApplication(new TestDriver(), config);
	}
}
