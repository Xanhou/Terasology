package org.terasology.logic.characters.movement.modes;

import com.bulletphysics.linearmath.QuaternionUtil;
import javax.vecmath.AxisAngle4f;
import javax.vecmath.Vector3f;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.terasology.engine.CoreRegistry;
import org.terasology.entitySystem.EntityRef;
import org.terasology.entitySystem.event.Event;
import org.terasology.logic.characters.CharacterMoveInputEvent;
import org.terasology.logic.characters.CharacterStateEvent;
import org.terasology.logic.characters.events.JumpEvent;
import org.terasology.logic.characters.events.VerticalCollisionEvent;
import org.terasology.logic.characters.movement.CharacterMovementComponent;
import org.terasology.math.TeraMath;
import org.terasology.physics.CharacterCollider;
import org.terasology.physics.PhysicsEngine;
import org.terasology.physics.SweepCallback;
import org.terasology.physics.events.MovedEvent;
import org.terasology.world.WorldProvider;
import static org.terasology.logic.characters.movement.modes.MovementConstants.*;
import org.terasology.math.Vector3fUtil;

/**
 * TODO: redo the documenting, since a lot has changed.
 * @author XanHou
 */
public abstract class AbstractMovementMode implements MovementMode {

    protected WorldProvider worldProvider = null;
    protected PhysicsEngine physics = null;
    private final Logger logger = LoggerFactory.getLogger(AbstractMovementMode.class);

    @Override
    public void updateState(EntityRef character, CharacterStateEvent result,
            CharacterMoveInputEvent input) {
        if (worldProvider == null) {
            worldProvider = CoreRegistry.get(WorldProvider.class);
        }
        if (physics == null) {
            physics = CoreRegistry.get(PhysicsEngine.class);
        }

        if (worldProvider == null || physics == null) {
            logger.error("WorldProvider or PhysicsEngine is null while trying to update position of entity. Entity: {}", character);
        }

        CharacterMovementComponent characterMovementComponent = character.getComponent(CharacterMovementComponent.class);
        boolean canMove = worldProvider.isBlockRelevant(result.getPosition());
        if (canMove) {
            updatePosition(characterMovementComponent, result, input, character);
        }
        //Rotation is allowed in unloaded chunks:
        updateRotation(characterMovementComponent, result, input);
    }

    protected void updatePosition(final CharacterMovementComponent movementComp, final CharacterStateEvent state,
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

        MoveResult moveResult = move(position, moveDelta, stepHeight, slopeFactor, collider);
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
                endVelocity.y = getVerticalCollisionFeedbackFactor(entity)* endVelocity.y;
            }
            state.setGrounded(false);
        }

        //Check footsteps and send movement event
        if (input.isFirstRun() && distanceMoved.length() > 0) {
            entity.send(new MovedEvent(distanceMoved, state.getPosition()));

            if (shouldSetFootStepDelta(state.isGrounded(), entity)) {
                state.setFootstepDelta(
                        state.getFootstepDelta() + distanceMoved.length() / movementComp.distanceBetweenFootsteps);
                if (state.getFootstepDelta() > 1) {
                    state.setFootstepDelta(state.getFootstepDelta() - 1);
                    if (input.isFirstRun()) {
                        Event footStepEvent = getFootStepEvent(position, entity);
                        if (footStepEvent != null) {
                            entity.send(footStepEvent);
                        }
                    }
                }
            }

        }

        if (input.isFirstRun() && moveResult.isHorizontalHit()) {
            Event event = getHorrizontalCollisionEvent(position, endVelocity, entity);
            if (event != null) {
                entity.send(event);
            }
        }
    }

    /**
     * Determines the new velocity and sets it with state.getVelocity().set(...)
     *
     * @param movementComp
     * @param state
     * @param input
     * @param entity
     */
    protected void setVelocity(final CharacterMovementComponent movementComp, final CharacterStateEvent state,
            CharacterMoveInputEvent input, EntityRef entity) {

        //TODO: make mode independant
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

        if (!userBasedYMovement(entity)) {
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
        desiredVelocity.y -= getGravity(entity);

        // Modify velocity towards desired, up to the maximum rate determined by friction
        Vector3f velocityDiff = new Vector3f(desiredVelocity);
        velocityDiff.sub(state.getVelocity());
        float inertia = getInertia(entity);
        velocityDiff.scale(Math.min(inertia * input.getDelta(), 1.0f));
        Vector3f endVelocity = new Vector3f(state.getVelocity());
        endVelocity.add(velocityDiff);
        
        float speed = endVelocity.length();
        endVelocity.scale(getFinalVelocityScalar(entity, speed));
        endVelocity.y = Math.max(-getMaxDownwardSpeed(entity), state.getVelocity().y - getGravity(entity) * input.getDelta());

        //Lets not forget to actually set the velocity:
        state.getVelocity().set(endVelocity);
    }

    protected MoveResult move(final Vector3f startPosition, final Vector3f moveDelta, final float stepHeight,
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
         * The moveDelta.y &lt 0 part is easy to understand here, we are moving
         * down, so we need to calculate downwards movement. The steppedUpDist
         * requires a bit of explaining.
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

    protected boolean moveDown(float dist, float slopeFactor, CharacterCollider collider, Vector3f position) {
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
     * If distance to move (horixMove.length()) is smaller than the epsilon
     * value of the physics engine, no action is taken by this method.
     *
     * @param horizMove
     * @param collider
     * @param position
     * @param slopeFactor
     * @param stepHeight
     * @return
     */
    protected HorizontalMoveResult moveHorizontal(Vector3f horizMove, CharacterCollider collider, Vector3f position, float slopeFactor,
            float stepHeight) {
        //TODO: divide in smaller sub methods and document everything.
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
     * Determines how far the given collider can move up and returns this value,
     * which is bound by riseAmount. This method does not alter any state
     * information.
     *
     * @param riseAmount the maximum amount to test for
     * @param collider the collider to test for
     * @param position the position the collider is on when testing
     * @return the amount the collider can move up. 0 &le return &ge riseAmount.
     */
    protected float moveUp(float riseAmount, CharacterCollider collider, Vector3f position) {
        Vector3f to = new Vector3f(position.x, position.y + riseAmount + VERTICAL_PENETRATION_LEEWAY, position.z);
        SweepCallback callback = collider.sweep(position, to, VERTICAL_PENETRATION_LEEWAY, -1f);
        if (callback.hasHit()) {
            float actualDist = Math.max(0,
                    ((riseAmount + VERTICAL_PENETRATION_LEEWAY) * callback.getClosestHitFraction()) - VERTICAL_PENETRATION_LEEWAY);
            return actualDist;
        }
        return riseAmount;
    }

    @SuppressWarnings(value = "SuspiciousNameCombination")
    protected void updateRotation(CharacterMovementComponent movementComp, CharacterStateEvent result,
            CharacterMoveInputEvent input) {
        if (movementComp.faceMovementDirection && result.getVelocity().lengthSquared() > 0.01f) {
            float yaw = (float) Math.atan2(result.getVelocity().x, result.getVelocity().z);
            AxisAngle4f axisAngle = new AxisAngle4f(0, 1, 0, yaw);
            result.getRotation().set(axisAngle);
        } else {
            QuaternionUtil.setEuler(result.getRotation(), TeraMath.DEG_TO_RAD * input.getYaw(), 0, 0);
        }
    }

    protected Vector3f extractResidualMovement(Vector3f hitNormal, Vector3f direction) {
        return extractResidualMovement(hitNormal, direction, 1f);
    }

    protected Vector3f extractResidualMovement(Vector3f hitNormal, Vector3f direction, float normalMag) {
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

    //TODO make abstract and only return TERMINAL_VEL when walking, otherwise return float.minvalue
    protected abstract float getMaxDownwardSpeed(EntityRef entity);/* {
        if(mode == WALKING) {
        return TERMINAL_VELOCITY;
        } else {
            return Float.POSITIVE_INFINITY;
        }
    }*/

    /**
     * The velocity will be scaled with this factor before being applied.
     * @param entity
     * @param finalSpeed the final spee before applying this scalar.
     * @return
     */
    protected abstract float getFinalVelocityScalar(EntityRef entity, float finalSpeed);/* {
        //TODO make abstractand only return this when swimming, return 1 otherwise:
        // Slow down due to friction
        if (speed > maxSpeed && mode == SWIMMING) {
            return (speed - 4 * (speed - movementComp.getMaxSpeed()) * input.getDelta()) / speed;
        } else {
            return 1f;
        }
    }*/
    
    /**
     * Returns the gravity for this movement mode.
     * @param mode
     * @return 
     */
    protected abstract float getGravity(EntityRef entity);

    /**
     * Returns the inertia for this movement mode.
     * @param mode
     * @param movementComp
     * @return 
     */
    protected abstract float getInertia(EntityRef entity);

    /**
     * Returns true if the footstep delta should be changed. In other words, it
     * should be true if the entity is doing something the generates footstep
     * events, such as walking while being on the ground (not jumping) or
     * swimming (SwimStrokeEvent)
     *
     * @param isGrounded whether or not the entity is grounded.
     * @return true if footsteps should be calculated, false otherwise.
     */
    protected abstract boolean shouldSetFootStepDelta(boolean isGrounded, EntityRef entity);

    /**
     * Returns the event to fire when an entity sets a footstep, swimstroke or
     * any other event that is generated based on the footstep mechanism.
     * Returning null results in no event being fired and is considered valid
     * behaviour.
     *
     * @param state 
     * @return
     */
    protected abstract Event getFootStepEvent(Vector3f newPosition, EntityRef entity);

    /**
     * When the character bumps into something while moving up, the vertical
     * velocity of the character is multiplied by this value. It is usual to 
     * make this value zero or negative, but not required.
     *
     * @param entity The entity that collided while being moved
     * @return
     */
    protected abstract float getVerticalCollisionFeedbackFactor(EntityRef entity);

    /**
     * Returns the event to fire when an entity has a horizontal collision.
     * Returning null results in no event being fired and is considered valid
     * behaviour.
     *
     * @param position the position where the collision happened
     * @param velocity the velocity of impact.
     * @param entity the entity that collided while being moved.
     * @return The event to fire in case of a horrizontal collision or null.
     */
    protected abstract Event getHorrizontalCollisionEvent(Vector3f position, Vector3f velocity, EntityRef entity);
    
    /**
     * @param entity
     * @return true if the user input can have influence on the vertical
     * movement of the character. False otherwise.
     */
    protected abstract boolean userBasedYMovement(EntityRef entity);
    
    /**
     * the moveHorizontal(...) method needed to return multiple values, this
     * class holds these values.
     */
    protected static class HorizontalMoveResult {

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
