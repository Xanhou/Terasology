/*
 * Copyright 2013 MovingBlocks
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package org.terasology.logic.characters.movement;

import org.terasology.physics.SweepCallback;
import com.bulletphysics.linearmath.QuaternionUtil;
import javax.vecmath.AxisAngle4f;
import javax.vecmath.Vector3f;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.terasology.entitySystem.EntityRef;
import org.terasology.entitySystem.event.Event;
import org.terasology.logic.characters.CharacterMoveInputEvent;
import org.terasology.logic.characters.CharacterMoveInputEvent;
import org.terasology.logic.characters.movement.CharacterMovementComponent;
import org.terasology.logic.characters.movement.CharacterMover;
import org.terasology.logic.characters.CharacterStateEvent;
import org.terasology.logic.characters.CharacterStateEvent;
import org.terasology.logic.characters.movement.MovementMode;
import static org.terasology.logic.characters.movement.MovementMode.CLIMBING;
import static org.terasology.logic.characters.movement.MovementMode.GHOSTING;
import static org.terasology.logic.characters.movement.MovementMode.SWIMMING;
import static org.terasology.logic.characters.movement.MovementMode.WALKING;
import org.terasology.logic.characters.events.FootstepEvent;
import org.terasology.logic.characters.events.HorizontalCollisionEvent;
import org.terasology.logic.characters.events.JumpEvent;
import org.terasology.logic.characters.events.OnEnterLiquidEvent;
import org.terasology.logic.characters.events.OnLeaveLiquidEvent;
import org.terasology.logic.characters.events.SwimStrokeEvent;
import org.terasology.logic.characters.events.VerticalCollisionEvent;
import org.terasology.math.TeraMath;
import org.terasology.math.Vector3fUtil;
import org.terasology.physics.PhysicsEngine;
import org.terasology.physics.CharacterCollider;
import org.terasology.physics.events.MovedEvent;
import org.terasology.world.WorldProvider;

/**
 * The KineticCharacterMover generalises the character movement to a physics engine independent class. The physics engine will then only
 * have to fill in several smaller method calls.
 * 
 * TODO: Refactor to allow additional movement modes.
 *
 * @author Immortius
 */
public class KinematicCharacterMover implements CharacterMover {

    private static final float CHECK_FORWARD_DIST = 0.05f;
    public static final float CLIMB_GRAVITY = 0f;
    public static final float GHOST_INERTIA = 4f;
    public static final float GRAVITY = 28.0f;
    /**
     * The amount of horizontal penetration to allow.
     */
    private static final float HORIZONTAL_PENETRATION = 0.03f;
    /**
     * The amount of extra distance added to horizontal movement to allow for penentration.
     */
    private static final float HORIZONTAL_PENETRATION_LEEWAY = 0.04f;
    /**
     * Players are not allowed to fall faster than this distance per second:
     */
    public static final float TERMINAL_VELOCITY = 64.0f;
    public static final float UNDERWATER_GRAVITY = 0.25f;
    public static final float UNDERWATER_INERTIA = 2.0f;
    /**
     * The amount of vertical penetration to allow.
     */
    private static final float VERTICAL_PENETRATION = 0.04f;
    /**
     * The amount of extra distance added to vertical movement to allow for penetration.
     */
    private static final float VERTICAL_PENETRATION_LEEWAY = 0.05f;
    
    private final Logger logger = LoggerFactory.getLogger(KinematicCharacterMover.class);
    private WorldProvider worldProvider;
    private PhysicsEngine physics;

    public KinematicCharacterMover(WorldProvider wp, PhysicsEngine physicsEngine) {
        this.worldProvider = wp;
        physics = physicsEngine;
    }

    /**
     * Updates whether a character should change movement mode (from being underwater or in a ladder). A higher and lower point of the
     * character is tested for being in water, only if both points are in water does the character count as swimming. <br> <br>
     *
     * Sends the OnEnterLiquidEvent and OnLeaveLiquidEvent events.
     *
     * @param movementComp The movement component of the character.
     * @param state The current state of the character.
     */
    private void checkMode(final CharacterMovementComponent movementComp, final CharacterStateEvent state,
            final CharacterStateEvent oldState, EntityRef entity, boolean firstRun) {
        //If we are ghosting, the mode cannot be changed.
        if (state.getMode() == MovementMode.GHOSTING) {
            return;
        }

        Vector3f worldPos = state.getPosition();
        Vector3f top = new Vector3f(worldPos);
        Vector3f bottom = new Vector3f(worldPos);
        top.y += 0.25f * movementComp.height;
        bottom.y -= 0.25f * movementComp.height;

        final boolean topUnderwater = worldProvider.getBlock(top).isLiquid();
        final boolean bottomUnderwater = worldProvider.getBlock(bottom).isLiquid();

        final boolean newSwimming = topUnderwater && bottomUnderwater;
        boolean newClimbing = false;

        //TODO: refactor this knot of if-else statements into something easy to read. Some sub-methods and switch statements would be nice.
        if (!newSwimming) {
            Vector3f[] sides = {new Vector3f(worldPos), new Vector3f(worldPos), new Vector3f(worldPos), new Vector3f(
                worldPos), new Vector3f(worldPos)};
            float factor = 0.18f;
            sides[0].x += factor * movementComp.radius;
            sides[1].x -= factor * movementComp.radius;
            sides[2].z += factor * movementComp.radius;
            sides[3].z -= factor * movementComp.radius;
            sides[4].y -= movementComp.height;
            for (Vector3f side : sides) {
                if (worldProvider.getBlock(side).isClimbable()) {
                    //If any of our sides are near a climbable block, climb!
                    newClimbing = true;
                    break;
                }
            }
        }

        if (newSwimming) {
            //Note that you cannot climb under water!
            if (state.getMode() != MovementMode.SWIMMING) {
                if (firstRun) {
                    entity.send(new OnEnterLiquidEvent(worldProvider.getBlock(state.getPosition())));
                }
                state.setMode(MovementMode.SWIMMING);
            }
        } else if (state.getMode() == MovementMode.SWIMMING) {
            if (firstRun) {
                entity.send(new OnLeaveLiquidEvent(worldProvider.getBlock(oldState.getPosition())));
            }
            if (newClimbing) {
                state.setMode(MovementMode.CLIMBING);
                state.getVelocity().y = 0;
            } else {
                if (state.getVelocity().y > 0) {
                    state.getVelocity().y += 8;
                }
                state.setMode(MovementMode.WALKING);
            }
        } else if (newClimbing != (state.getMode() == MovementMode.CLIMBING)) {
            //We need to toggle the climbing mode
            state.getVelocity().y = 0;
            state.setMode((newClimbing) ? MovementMode.CLIMBING : MovementMode.WALKING);
        }
    }

    private Vector3f extractResidualMovement(Vector3f hitNormal, Vector3f direction) {
        return extractResidualMovement(hitNormal, direction, 1f);
    }

    private Vector3f extractResidualMovement(Vector3f hitNormal, Vector3f direction, float normalMag) {
        float movementLength = direction.length();
        if (movementLength > physics.getEpsilon()) {
            direction.normalize();
            Vector3f reflectDir = Vector3fUtil.reflect(direction, hitNormal, new Vector3f());
            reflectDir.normalize();
            Vector3f perpendicularDir = Vector3fUtil.getPerpendicularComponent(reflectDir, hitNormal, new Vector3f());
            if (normalMag != 0.0f) {
                Vector3f perpComponent = new Vector3f();
                perpComponent.scale(normalMag * movementLength, perpendicularDir);
                direction.set(perpComponent);
            }
        }
        return direction;
    }
    
    private void ghost(final CharacterMovementComponent movementComp, final CharacterStateEvent state,
            CharacterMoveInputEvent input, EntityRef entity) {
        setVelocity(movementComp, state, input, entity);
        // No collision, so just do the move
        Vector3f deltaPos = new Vector3f(state.getVelocity());
        deltaPos.scale(input.getDelta());
        state.getPosition().add(deltaPos);
        if (input.isFirstRun() && deltaPos.length() > 0) {
            entity.send(new MovedEvent(deltaPos, state.getPosition()));
        }
    }

    private MoveResult move(final Vector3f startPosition, final Vector3f moveDelta, final float stepHeight,
            final float slopeFactor, final CharacterCollider collider) {
        Vector3f position = new Vector3f(startPosition);
        boolean hitTop = false;
        boolean hitBottom = false;
        boolean hitSide = false;
        // Actual upwards movement
        if (moveDelta.y > 0) {
            float actualUpDist = moveUp(moveDelta.y, collider, position);
            position.y += actualUpDist;
            hitTop = moveDelta.y - actualUpDist > physics.getEpsilon();
        }
        HorizontalMoveResult horizontalResult = moveHorizontal(new Vector3f(moveDelta.x, 0, moveDelta.z), collider, position, slopeFactor, stepHeight);
        hitSide = horizontalResult.hitSide;
        float steppedUpDist = horizontalResult.steppedUpDist;
        
        /**
         * The moveDelta.y &lt 0 part is easy to understand here, we are moving down, so we need to calculate downwards movement.
         * The steppedUpDist requires a bit of explaining.
         * 
         */
        if (moveDelta.y < 0 || steppedUpDist > 0) {
            float dist = (moveDelta.y < 0) ? moveDelta.y : 0;
            dist -= steppedUpDist;
            hitBottom = moveDown(dist, slopeFactor, collider, position);
        }
        if (!hitBottom && stepHeight > 0) {
            Vector3f tempPos = new Vector3f(position);
            hitBottom = moveDown(-stepHeight, slopeFactor, collider, tempPos);
            // Don't apply step down if nothing to step onto
            if (hitBottom) {
                position.set(tempPos);
            }
        }
        return new MoveResult(position, hitSide, hitBottom, hitTop);
    }

    private boolean moveDown(float dist, float slopeFactor, CharacterCollider collider, Vector3f position) {
        float remainingDist = -dist;
        Vector3f targetPos = new Vector3f(position);
        targetPos.y -= remainingDist + VERTICAL_PENETRATION_LEEWAY;
        Vector3f normalizedDir = new Vector3f(0, -1, 0);
        boolean hit = false;
        int iteration = 0;
        while (remainingDist > physics.getEpsilon() && iteration++ < 10) {
            SweepCallback callback = collider.sweep(position, targetPos, VERTICAL_PENETRATION, -1.0f);
            float actualDist = Math.max(0,
                    (remainingDist + VERTICAL_PENETRATION_LEEWAY) * callback.getClosestHitFraction() - VERTICAL_PENETRATION_LEEWAY);
            Vector3f expectedMove = new Vector3f(targetPos);
            expectedMove.sub(position);
            if (expectedMove.lengthSquared() > physics.getEpsilon()) {
                expectedMove.normalize();
                expectedMove.scale(actualDist);
                position.add(expectedMove);
            }
            remainingDist -= actualDist;
            if (remainingDist < physics.getEpsilon()) {
                break;
            } else if (callback.hasHit()) {
                float originalSlope = callback.getHitNormalWorld().dot(new Vector3f(0, 1, 0));
                if (originalSlope < slopeFactor) {
                    float slope = callback.calculateSafeSlope(originalSlope, CHECK_FORWARD_DIST);
                    if (slope < slopeFactor) {
                        remainingDist -= actualDist;
                        expectedMove.set(targetPos);
                        expectedMove.sub(position);
                        extractResidualMovement(callback.getHitNormalWorld(), expectedMove);
                        float sqrDist = expectedMove.lengthSquared();
                        if (sqrDist > physics.getEpsilon()) {
                            expectedMove.normalize();
                            if (expectedMove.dot(normalizedDir) <= 0.0f) {
                                hit = true;
                                break;
                            }
                        } else {
                            hit = true;
                            break;
                        }
                        if (expectedMove.y > -physics.getEpsilon()) {
                            hit = true;
                            break;
                        }
                        normalizedDir.set(expectedMove);
                        expectedMove.scale(-remainingDist / expectedMove.y + HORIZONTAL_PENETRATION_LEEWAY);
                        targetPos.add(position, expectedMove);
                    } else {
                        hit = true;
                        break;
                    }
                } else {
                    hit = true;
                    break;
                }
            } else {
                break;
            }
        }
        if (iteration >= 10) {
            hit = true;
        }
        return hit;
    }

    /**
     * If distance to move (horixMove.length()) is smaller than the epsilon value of the physics engine, no action is taken by this method.
     * @param horizMove
     * @param collider
     * @param position
     * @param slopeFactor
     * @param stepHeight
     * @return 
     */
    private HorizontalMoveResult moveHorizontal(Vector3f horizMove, CharacterCollider collider, Vector3f position, float slopeFactor,
            float stepHeight) {
        float dist = horizMove.length();
        if (dist < physics.getEpsilon()) {
            return new HorizontalMoveResult();
        }
        
        float steppedUpDist = 0f;
        boolean steppedThisMethodCall = false;
        float remainingFraction = 1.0f;
        
        boolean horizontalHit = false;
        Vector3f normalizedDir = Vector3fUtil.safeNormalize(horizMove, new Vector3f());
        Vector3f targetPos = new Vector3f(normalizedDir);
        targetPos.scale(dist + HORIZONTAL_PENETRATION_LEEWAY);
        targetPos.add(position);
        int iteration = 0;
        Vector3f lastHitNormal = new Vector3f(0, 1, 0);
        while (remainingFraction >= 0.01f && iteration++ < 10) {
            SweepCallback callback = collider.sweep(position, targetPos, HORIZONTAL_PENETRATION, slopeFactor);
            /* Note: this isn't quite correct (after the first iteration the closestHitFraction is only for part of the moment)
             but probably close enough */
            float actualDist = Math.max(0,
                    (dist + HORIZONTAL_PENETRATION_LEEWAY) * callback.getClosestHitFraction() - HORIZONTAL_PENETRATION_LEEWAY);
            if (actualDist != 0) {
                remainingFraction -= actualDist / dist;
            }
            if (callback.hasHit()) {
                if (actualDist > physics.getEpsilon()) {
                    Vector3f actualMove = new Vector3f(normalizedDir);
                    actualMove.scale(actualDist);
                    position.add(actualMove);
                }
                dist -= actualDist;
                Vector3f newDir = new Vector3f(normalizedDir);
                newDir.scale(dist);
                float slope = callback.getHitNormalWorld().dot(new Vector3f(0, 1, 0));
                // We step up if we're hitting a big slope, or if we're grazing 
                // the ground, otherwise we move up a shallow slope.
                if (slope < slopeFactor || 1 - slope < physics.getEpsilon()) {
                    boolean steppedThisIterarion = false;
                    if (!steppedThisMethodCall) {
                        steppedThisMethodCall = true;
                        steppedThisIterarion = true;
                        boolean moveUpStep = callback.checkForStep(newDir, stepHeight, slopeFactor, CHECK_FORWARD_DIST);
                        if (moveUpStep) {
                            steppedUpDist = moveUp(stepHeight, collider, position);
                            position.y += steppedUpDist;
                        }
                    }
                    if (!steppedThisIterarion) {
                        horizontalHit = true;
                        Vector3f newHorizDir = new Vector3f(newDir.x, 0, newDir.z);
                        Vector3f horizNormal = new Vector3f(callback.getHitNormalWorld().x, 0,
                                callback.getHitNormalWorld().z);
                        if (horizNormal.lengthSquared() > physics.getEpsilon()) {
                            horizNormal.normalize();
                            if (lastHitNormal.dot(horizNormal) > physics.getEpsilon()) {
                                break;
                            }
                            lastHitNormal.set(horizNormal);
                            extractResidualMovement(horizNormal, newHorizDir);
                        }
                        newDir.set(newHorizDir);
                    }
                } else {
                    // Hitting a shallow slope, move up it
                    Vector3f newHorizDir = new Vector3f(newDir.x, 0, newDir.z);
                    extractResidualMovement(callback.getHitNormalWorld(), newDir);
                    Vector3f modHorizDir = new Vector3f(newDir);
                    modHorizDir.y = 0;
                    newDir.scale(newHorizDir.length() / modHorizDir.length());
                }
                float sqrDist = newDir.lengthSquared();
                if (sqrDist > physics.getEpsilon()) {
                    newDir.normalize();
                    if (newDir.dot(normalizedDir) <= 0.0f) {
                        break;
                    }
                } else {
                    break;
                }
                dist = (float) Math.sqrt(sqrDist);
                normalizedDir.set(newDir);
                targetPos.set(normalizedDir);
                targetPos.scale(dist + HORIZONTAL_PENETRATION_LEEWAY);
                targetPos.add(position);
            } else {
                normalizedDir.scale(dist);
                position.add(normalizedDir);
                break;
            }
        }
        return new HorizontalMoveResult(horizontalHit, steppedUpDist);
    }

    /**
     * Determines how far the given collider can move up and returns this value, which is bound by riseAmount.
     * This method does not alter any state information.
     * @param riseAmount the maximum amount to test for
     * @param collider the collider to test for
     * @param position the position the collider is on when testing
     * @return the amount the collider can move up. 0 &le return &ge riseAmount.
     */
    private float moveUp(float riseAmount, CharacterCollider collider, Vector3f position) {
        Vector3f to = new Vector3f(position.x, position.y + riseAmount + VERTICAL_PENETRATION_LEEWAY, position.z);
        SweepCallback callback = collider.sweep(position, to, VERTICAL_PENETRATION_LEEWAY, -1f);
        if (callback.hasHit()) {
            float actualDist = Math.max(0,
                    ((riseAmount + VERTICAL_PENETRATION_LEEWAY) * callback.getClosestHitFraction()) - VERTICAL_PENETRATION_LEEWAY);
            return actualDist;
        }
        return riseAmount;
    }

    @Override
    public CharacterStateEvent step(CharacterStateEvent initial, CharacterMoveInputEvent input, EntityRef entity) {
        CharacterMovementComponent characterMovementComponent = entity.getComponent(CharacterMovementComponent.class);
        CharacterStateEvent result = new CharacterStateEvent(initial);
        result.setSequenceNumber(input.getSequenceNumber());
        //Prevent movement in unloaded chunks. Likely to happen when teleporting.
        boolean canMove = worldProvider.isBlockRelevant(initial.getPosition());
        if (canMove) {
            updatePosition(characterMovementComponent, result, input, entity);
        }
        //Rotation is allowed in unloaded chunks:
        result.setTime(initial.getTime() + input.getDeltaMs());
        updateRotation(characterMovementComponent, result, input);
        result.setPitch(input.getPitch());
        result.setYaw(input.getYaw());
        
        if(canMove) {
            checkMode(characterMovementComponent, result, initial, entity, input.isFirstRun());
        }
        
        input.runComplete();
        return result;
    }

    private void updatePosition(final CharacterMovementComponent movementComp, final CharacterStateEvent state,
            CharacterMoveInputEvent input, EntityRef entity) {
        switch (state.getMode()) {
            case GHOSTING:
                ghost(movementComp, state, input, entity);
                break;
            case SWIMMING:
            case WALKING:
            case CLIMBING:
                walk(movementComp, state, input, entity);
                break;
            default:
                walk(movementComp, state, input, entity);
                break;
        }
    }

    @SuppressWarnings(value = "SuspiciousNameCombination")
    private void updateRotation(CharacterMovementComponent movementComp, CharacterStateEvent result,
            CharacterMoveInputEvent input) {
        if (movementComp.faceMovementDirection && result.getVelocity().lengthSquared() > 0.01f) {
            float yaw = (float) Math.atan2(result.getVelocity().x, result.getVelocity().z);
            AxisAngle4f axisAngle = new AxisAngle4f(0, 1, 0, yaw);
            result.getRotation().set(axisAngle);
        } else {
            QuaternionUtil.setEuler(result.getRotation(), TeraMath.DEG_TO_RAD * input.getYaw(), 0, 0);
        }
    }

    private void walk(final CharacterMovementComponent movementComp, final CharacterStateEvent state,
            CharacterMoveInputEvent input, EntityRef entity) {
        Vector3f initialVelocity = new Vector3f(state.getVelocity());
        setVelocity(movementComp, state, input, entity);
        Vector3f endVelocity = state.getVelocity();
        
        Vector3f position = state.getPosition();
        Vector3f moveDelta = new Vector3f(endVelocity);
        moveDelta.scale(input.getDelta());
        float stepHeight = movementComp.getStepHeight(state.isGrounded());
        float slopeFactor = movementComp.getSlopeFactor();
        CharacterCollider collider = physics.getCharacterCollider(entity);
        
        MoveResult moveResult = move(position, moveDelta,  stepHeight, slopeFactor, collider);
        Vector3f distanceMoved = new Vector3f(moveResult.getFinalPosition());
        distanceMoved.sub(state.getPosition());
        state.getPosition().set(moveResult.getFinalPosition());
        
        if (moveResult.isBottomHit()) {
            //Send VerticalCollisionEvent
            if (!state.isGrounded()) {
                if (input.isFirstRun()) {
                    Vector3f landVelocity = new Vector3f(initialVelocity);
                    landVelocity.y += (distanceMoved.y / moveDelta.y) * (endVelocity.y - initialVelocity.y);
                    logger.debug("Landed at " + landVelocity);
                    entity.send(new VerticalCollisionEvent(state.getPosition(), landVelocity));
                }
                state.setGrounded(true);
            }
            endVelocity.y = 0;
            // Jumping is only possible, if the entity is standing on ground
            if (input.isJumpRequested()) {
                state.setGrounded(false);
                endVelocity.y += movementComp.jumpSpeed;
                if (input.isFirstRun()) {
                    entity.send(new JumpEvent());
                }
            }
        } else {
            //Apply head bumping feedback
            if (moveResult.isTopHit() && endVelocity.y > 0) {
                endVelocity.y = getVerticalCollisionFeedbackFactor(state.getMode()) * endVelocity.y;
            }
            state.setGrounded(false);
        }
        
        //Check footsteps and send movement event
        if (input.isFirstRun() && distanceMoved.length() > 0) {
            entity.send(new MovedEvent(distanceMoved, state.getPosition()));
            
            if (shouldSetFootStepDelta(state.getMode(), state.isGrounded())) {
                state.setFootstepDelta(
                        state.getFootstepDelta() + distanceMoved.length() / movementComp.distanceBetweenFootsteps);
                if (state.getFootstepDelta() > 1) {
                    state.setFootstepDelta(state.getFootstepDelta() - 1);
                    if (input.isFirstRun()) {
                        Event footStepEvent = getFootStepEvent(state.getMode(), state);
                        if(footStepEvent != null) {
                            entity.send(footStepEvent);
                        }
                    }
                }
            }

        }
        
        if (input.isFirstRun() && moveResult.isHorizontalHit()) {
            Event event = getHorrizontalCollisionEvent(state.getMode(), state);
            if(event != null) {
                entity.send(event);
            }
        }
    }
    
    /**
     * Determines the new velocity and sets it with state.getVelocity().set(...)
     * @param movementComp
     * @param state
     * @param input
     * @param entity 
     */
    private void setVelocity(final CharacterMovementComponent movementComp, final CharacterStateEvent state,
            CharacterMoveInputEvent input, EntityRef entity) {
        
        Vector3f desiredVelocity = new Vector3f(input.getMovementDirection());
        
        float lengthSquared = desiredVelocity.lengthSquared();
        //Allow smaller velocity for analog inputs, not bigger velocity:
        if (lengthSquared > 1) {
            desiredVelocity.normalize();
        }
        
        float maxSpeed = movementComp.getMaxSpeed();
        
        if (input.isRunning()) {
            maxSpeed *= movementComp.runFactor;
        }
        
        if(state.getMode() == MovementMode.WALKING) {
            // As we can't use it, remove the y component of desired movement while maintaining speed.
            if (desiredVelocity.y != 0) {
                float speed = desiredVelocity.length();
                desiredVelocity.y = 0;
                if (desiredVelocity.x != 0 || desiredVelocity.z != 0) {
                    desiredVelocity.normalize();
                    desiredVelocity.scale(speed);
                }
            }
        }
        
        desiredVelocity.scale(maxSpeed);
        desiredVelocity.y -= getGravity(state.getMode());
        
        // Modify velocity towards desired, up to the maximum rate determined by friction
        Vector3f velocityDiff = new Vector3f(desiredVelocity);
        velocityDiff.sub(state.getVelocity());
        float inertia = getInertia(state.getMode(), movementComp);
        velocityDiff.scale(Math.min(inertia * input.getDelta(), 1.0f));
        Vector3f endVelocity = new Vector3f(state.getVelocity());
        endVelocity.add(velocityDiff);
        
        //Determine special adjustments:
        switch (state.getMode()) {
            case SWIMMING:
                // Slow down due to friction
                float speed = endVelocity.length();
                if (speed > maxSpeed) {
                    endVelocity.scale((speed - 4 * (speed - movementComp.getMaxSpeed()) * input.getDelta()) / speed);
                }
                break;
            case WALKING:
                endVelocity.y = Math.max(-TERMINAL_VELOCITY, state.getVelocity().y - GRAVITY * input.getDelta());
                break;
            default:
                //No special adjustments for other modes
                break;
        }
        
        //Lets not forget to actually set the velocity:
        state.getVelocity().set(endVelocity);
    }
    
    //TODO move the getter methods to the CharacterMovementComponent
    private float getGravity(MovementMode mode) {
        switch (mode) {
            case GHOSTING:
                return 0f;
            case SWIMMING:
                return UNDERWATER_GRAVITY;
            case WALKING:
                return GRAVITY;
            case CLIMBING:
                return CLIMB_GRAVITY;
            default:
                return GRAVITY;
        }
    }

    private float getInertia(MovementMode mode, CharacterMovementComponent movementComp) {
        switch (mode) {
            case GHOSTING:
                return GHOST_INERTIA;
            case SWIMMING:
                return UNDERWATER_INERTIA;
            case WALKING:
                return movementComp.groundFriction;
            case CLIMBING:
                return movementComp.groundFriction;
            default:
                return GRAVITY;
        }
    }

    private boolean shouldSetFootStepDelta(MovementMode mode, boolean isGrounded) {
        switch (mode) {
            case GHOSTING:
                return false;
            case SWIMMING:
                return true;
            case WALKING:
                return isGrounded;
            case CLIMBING:
                return isGrounded;
            default:
                return false;
        }
    }
    
    private Event getFootStepEvent(MovementMode mode, CharacterStateEvent state) {
        switch (mode) {
            case GHOSTING:
                return null;
            case SWIMMING:
                return new SwimStrokeEvent(worldProvider.getBlock(state.getPosition()));
            case WALKING:
                return new FootstepEvent();
            case CLIMBING:
                return null;
            default:
                return null;
        }
    }
    
    /**
     * When the character bumps into something while moving up, the vertical velocity of the character is multiplied by this value.
     * @param mode
     * @return 
     */
    private float getVerticalCollisionFeedbackFactor(MovementMode mode) {
        switch(mode) {
            case GHOSTING:
                return 0f;
            case SWIMMING:
                return 0f;
            case WALKING:
                return -.5f;
            case CLIMBING:
                return 0f;
            default:
                return 0f;
        }
    }
    
    private Event getHorrizontalCollisionEvent(MovementMode mode, CharacterStateEvent state) {
        switch (mode) {
            case GHOSTING:
                return null;
            case SWIMMING:
                return null;
            case WALKING:
                return new HorizontalCollisionEvent(state.getPosition(), state.getVelocity());
            case CLIMBING:
                return null;
            default:
                return null;
        }
    }
    
    /**
     * Holds the result of some movement.
     */
    public class MoveResult {

        private Vector3f finalPosition;
        private boolean horizontalHit;
        private boolean bottomHit;
        private boolean topHit;

        public MoveResult(Vector3f finalPosition, boolean hitHorizontal, boolean hitBottom, boolean hitTop) {
            this.finalPosition = finalPosition;
            this.horizontalHit = hitHorizontal;
            this.bottomHit = hitBottom;
            this.topHit = hitTop;
        }

        public Vector3f getFinalPosition() {
            return finalPosition;
        }

        public boolean isHorizontalHit() {
            return horizontalHit;
        }

        public boolean isBottomHit() {
            return bottomHit;
        }

        public boolean isTopHit() {
            return topHit;
        }
    }
    
    private static class HorizontalMoveResult {
        public final boolean hitSide;
        public final float steppedUpDist;
        
        public HorizontalMoveResult(boolean hitSide, float steppedUpDist) {
            this.steppedUpDist = steppedUpDist;
            this.hitSide = hitSide;
        }
        
        public HorizontalMoveResult() {
            hitSide = false;
            steppedUpDist = 0f;
        }
    }
}
