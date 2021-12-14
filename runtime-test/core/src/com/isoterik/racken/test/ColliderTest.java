package com.isoterik.racken.test;

import com.badlogic.gdx.graphics.Color;
import com.isoterik.racken.GameObject;
import com.isoterik.racken.Scene;
import com.isoterik.racken.physics2d.PhysicsManager2d;
import com.isoterik.racken.physics2d.PhysicsMaterial2d;
import com.isoterik.racken.physics2d.RigidBody2d;
import com.isoterik.racken.physics2d.colliders.BoxCollider;
import com.isoterik.racken.physics2d.colliders.CircleCollider;
import com.isoterik.racken.physics2d.utils.Box2dUtil;

public class ColliderTest extends Scene {
    public ColliderTest() {
        setBackgroundColor(Color.BLACK);

        PhysicsManager2d physicsManager2d = PhysicsManager2d.setup(this);
        Box2dUtil.createBoundaryBox(physicsManager2d.getPhysicsWorld(), gameWorldUnits.getWorldWidth(),
                gameWorldUnits.getWorldHeight(), .2f);

        physicsManager2d.setRenderPhysicsDebugLines(true);

        GameObject box = GameObject.newInstance();
        box.transform.setPosition(3, 3);
        box.transform.setSize(1, 1);

        GameObject circle = GameObject.newInstance();
        circle.transform.setPosition(5, 5);
        circle.transform.setSize(1, 1);

        box.addComponent(new RigidBody2d(RigidBody2d.DynamicBody, physicsManager2d));
        box.addComponent(new BoxCollider());

        circle.addComponent(new CircleCollider());
        circle.addComponent(new RigidBody2d(RigidBody2d.DynamicBody, new PhysicsMaterial2d(.1f, .9f,
                .5f),
                physicsManager2d));

        addGameObject(box);
        addGameObject(circle);
    }
}
