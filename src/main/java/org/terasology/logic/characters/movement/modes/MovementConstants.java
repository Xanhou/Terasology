package org.terasology.logic.characters.movement.modes;

/**
 *
 * @author Rednax
 */
public class MovementConstants {
    
    /**
     * The amount of vertical penetration to allow.
     */
    public static final float VERTICAL_PENETRATION = 0.04f;
    /**
     * The amount of extra distance added to vertical movement to allow for penetration.
     */
    public static final float VERTICAL_PENETRATION_LEEWAY = 0.05f;
    /**
     * The amount of horizontal penetration to allow.
     */
    public static final float HORIZONTAL_PENETRATION = 0.03f;
    /**
     * The amount of extra distance added to horizontal movement to allow for penentration.
     */
    public static final float HORIZONTAL_PENETRATION_LEEWAY = 0.04f;
    
    public static final float CHECK_FORWARD_DIST = 0.05f;
    public static final float CLIMB_GRAVITY = 0f;
    public static final float GHOST_INERTIA = 4f;
    public static final float GRAVITY = 28.0f;
    /**
     * Players are not allowed to fall faster than this distance per second:
     */
    public static final float TERMINAL_VELOCITY = 64.0f;
    public static final float UNDERWATER_GRAVITY = 0.25f;
    public static final float UNDERWATER_INERTIA = 2.0f;
    
    private MovementConstants(){};
}
