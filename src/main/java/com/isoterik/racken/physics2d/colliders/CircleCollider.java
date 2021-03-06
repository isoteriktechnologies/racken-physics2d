package com.isoterik.racken.physics2d.colliders;

import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.physics.box2d.CircleShape;
import com.badlogic.gdx.physics.box2d.FixtureDef;
import com.isoterik.racken.physics2d.PhysicsMaterial2d;

/**
 * A collider that generates a circular collision boundary
 *
 * @author imranabdulmalik
 */
public class CircleCollider extends Collider {
    private float radius;
    private final Vector2 position;

    /**
     * Creates a new instance given a radius and a position on the {@link com.badlogic.gdx.physics.box2d.Body} to place the shape at
     * @param radius the radius of the circle
     * @param x the x-coordinate of the position of the circle
     * @param y the y-coordinate of the position of the circle
     */
    public CircleCollider(float radius, float x, float y) {
        position = new Vector2(x, y);
        this.radius = radius;
    }

    /**
     * Creates a new instance given a radius. Position defaults to (0, 0)
     * @param radius the radius of the circle
     */
    public CircleCollider(float radius)
    { this(radius, 0, 0); }

    /**
     * Creates a new instance with no radius. If no valid radius is set before this collider gets used by a
     * {@link com.isoterik.racken.physics2d.RigidBody2d}, the collider will assume the radius of
     * the host game object
     */
    public CircleCollider()
    { this(-1f); }

    @Override
    public CircleCollider setMaterial(PhysicsMaterial2d material) {
        super.setMaterial(material);
        return this;
    }

    /**
     * Sets the radius of the circle.
     * <strong>Note:</strong> this has no effect if this collider has already been used by a
     * {@link com.isoterik.racken.physics2d.RigidBody2d}
     * @param radius the radius of the circle
     */
    public void setRadius(float radius)
    { this.radius = radius; }

    /**
     *
     * @return the radius of the circle
     */
    public float getRadius()
    { return radius; }

    @Override
    public FixtureDef __getFixtureDef() {
        // Assumes the radius of the host game object if the radius is <= 0
        if (radius <= 0)
            radius = gameObject.transform.size.x * .5f;

        shape = new CircleShape();
        shape.setRadius(radius);
        ((CircleShape)shape).setPosition(position);

        FixtureDef fdef = new FixtureDef();
        fdef.shape = shape;

        return fdef;
    }
}