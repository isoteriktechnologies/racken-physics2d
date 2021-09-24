package com.isoterik.racken.test;

import com.badlogic.gdx.graphics.Color;
import com.isoterik.racken.Scene;
import com.isoterik.racken.physics2d.PhysicsManager2d;
import com.isoterik.racken.physics2d.utils.Box2dUtil;

public class PhysicsManagerTest extends Scene {
    public PhysicsManagerTest() {
        setBackgroundColor(Color.BLACK);

        PhysicsManager2d physicsManager2d = PhysicsManager2d.setup(this);
        Box2dUtil.createBoundaryBox(physicsManager2d.getPhysicsWorld(), gameWorldUnits.getWorldWidth(),
                gameWorldUnits.getWorldHeight(), .2f);

        physicsManager2d.setRenderPhysicsDebugLines(true);
    }
}