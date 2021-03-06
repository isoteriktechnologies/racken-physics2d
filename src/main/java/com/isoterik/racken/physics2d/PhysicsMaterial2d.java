package com.isoterik.racken.physics2d;

/**
 * Defines the physical properties of a 2D physics body.
 *
 * @author isoteriksoftware
 */
public class PhysicsMaterial2d {
    /**
     * The friction coefficient, usually in the range [0,1].
     */
    public float friction;

    /**
     * The restitution (elasticity) usually in the range [0,1]. The higher the value, the more energy conserved by the body.
     */
    public float bounciness;

    /**
     * The density, usually in kg/m^3.
     */
    public float density;

    /**
     * Creates a new instance given the friction, bounciness and density
     * @param friction The friction coefficient, usually in the range [0,1].
     * @param bounciness The restitution (elasticity) usually in the range [0,1]. The higher the value, the more energy conserved by the body.
     * @param density The density, usually in kg/m^2.
     */
    public PhysicsMaterial2d(float friction, float bounciness,
                             float density) {
        this.friction   = friction;
        this.bounciness = bounciness;
        this.density    = density;
    }

    /**
     * Creates an instance with default values. friction defaults to 0.4f, bounciness defaults to 0.1f and density defaults to 1f
     */
    public PhysicsMaterial2d() {
        this(0.4f, 0.1f, 1f);
    }
}