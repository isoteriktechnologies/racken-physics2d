package com.isoterik.racken.test;

import com.badlogic.gdx.Input;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.math.Vector2;
import com.isoterik.racken.GameObject;
import com.isoterik.racken.Scene;
import com.isoterik.racken.input.KeyTrigger;
import com.isoterik.racken.physics2d.PhysicsManager2d;
import com.isoterik.racken.physics2d.PhysicsMaterial2d;
import com.isoterik.racken.physics2d.RigidBody2d;
import com.isoterik.racken.physics2d.colliders.BoxCollider;
import com.isoterik.racken.physics2d.colliders.CircleCollider;
import com.isoterik.racken.physics2d.utils.Box2dUtil;

public class MultipleColliderTest extends Scene {
    public MultipleColliderTest() {
        setBackgroundColor(Color.BLACK);

        PhysicsManager2d physicsManager2d = PhysicsManager2d.setup(this);
        Box2dUtil.createBoundaryBox(physicsManager2d.getPhysicsWorld(), gameWorldUnits.getWorldWidth(),
                gameWorldUnits.getWorldHeight(), .2f);

        physicsManager2d.setRenderPhysicsDebugLines(true);

        GameObject car = GameObject.newInstance();
        float width = 3, height = 1;
        car.transform.setPosition(3, gameWorldUnits.getWorldHeight() - height);
        car.transform.setSize(width, height);

        PhysicsMaterial2d tireMaterial = new PhysicsMaterial2d(.1f, .3f, .3f);
        RigidBody2d rigidBody = new RigidBody2d(RigidBody2d.DynamicBody, physicsManager2d);

        car.addComponent(rigidBody);
        car.addComponent(new BoxCollider());
        car.addComponent(new CircleCollider(.5f, -width/2, -height/2).setMaterial(tireMaterial));
        car.addComponent(new CircleCollider(.5f, width/2, -height/2).setMaterial(tireMaterial));

        addGameObject(car);

        final float speed = 50;

        input.addKeyListener(KeyTrigger.keyDownTrigger(Input.Keys.RIGHT).setPolled(true), (mappingName, keyEventData) ->
                rigidBody.getBody().applyForceToCenter(new Vector2(speed, 0), true));

        input.addKeyListener(KeyTrigger.keyDownTrigger(Input.Keys.LEFT).setPolled(true), (mappingName, keyEventData) ->
                rigidBody.getBody().applyForceToCenter(new Vector2(-speed, 0), true));
    }
}
