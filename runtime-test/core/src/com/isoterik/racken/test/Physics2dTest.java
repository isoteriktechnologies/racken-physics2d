package com.isoterik.racken.test;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.math.MathUtils;
import com.isoterik.racken.GameObject;
import com.isoterik.racken.Scene;
import com.isoterik.racken.physics2d.*;
import com.isoterik.racken.physics2d.colliders.BoxCollider;
import com.isoterik.racken.physics2d.colliders.CircleCollider;
import com.isoterik.racken.physics2d.utils.Box2dUtil;

public class Physics2dTest extends Scene {
    public Physics2dTest() {
        setBackgroundColor(Color.BLACK);

        PhysicsManager2d physicsManager2d = PhysicsManager2d.setup(this);
        Box2dUtil.createBoundaryBox(physicsManager2d.getPhysicsWorld(), gameWorldUnits.getWorldWidth(),
                gameWorldUnits.getWorldHeight(), .3f);

        physicsManager2d.setRenderPhysicsDebugLines(true);

        for (int i = 0; i < 5; i++) {
            float width = 1, height = 1;

            GameObject box = GameObject.newInstance("Box" + (i+1));
            box.transform.setRotation(MathUtils.random(0, 180));
            box.transform.setPosition(MathUtils.random(0, gameWorldUnits.getWorldWidth() - width),
                    MathUtils.random(0, gameWorldUnits.getWorldHeight() - height));
            box.transform.setSize(width, height);

            GameObject circle = GameObject.newInstance("Circle" + (i+1));
            circle.transform.setPosition(MathUtils.random(0, gameWorldUnits.getWorldWidth() - width),
                    MathUtils.random(0, gameWorldUnits.getWorldHeight() - height));
            circle.transform.setSize(width, height);

            box.addComponent(new RigidBody2d(RigidBody2d.DynamicBody, physicsManager2d));
            box.addComponent(new BoxCollider());
            box.addComponent(new PhysicsAwareComponent());

            circle.addComponent(new CircleCollider());
            circle.addComponent(new RigidBody2d(RigidBody2d.DynamicBody, new PhysicsMaterial2d(.1f, 1f, .5f),
                    physicsManager2d));
            circle.addComponent(new PhysicsAwareComponent());

            addGameObject(box);
            addGameObject(circle);
        }
    }

    private static class PhysicsAwareComponent extends Physics2d {
        @Override
        public void onCollisionEnter2d(Collision2d collision) {
            System.out.println(gameObject.getTag() + " collided with " +
                    (collision.other != null ? collision.other.getTag() : "wall"));
        }
    }
}












