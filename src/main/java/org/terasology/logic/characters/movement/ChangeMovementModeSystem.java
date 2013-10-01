package org.terasology.logic.characters.movement;

import javax.vecmath.Vector3f;
import org.terasology.entitySystem.EntityRef;
import org.terasology.entitySystem.RegisterMode;
import org.terasology.entitySystem.event.ReceiveEvent;
import org.terasology.entitySystem.systems.ComponentSystem;
import org.terasology.entitySystem.systems.In;
import org.terasology.entitySystem.systems.RegisterSystem;
import org.terasology.logic.characters.CharacterStateEvent;
import org.terasology.logic.characters.events.OnEnterLiquidEvent;
import org.terasology.logic.characters.events.OnLeaveLiquidEvent;
import org.terasology.world.WorldProvider;

/**
 *
 * @author XanHou
 */
@RegisterSystem(RegisterMode.ALWAYS)
public class ChangeMovementModeSystem implements ComponentSystem{

    @In
    private WorldProvider worldProvider;
    
    @Override
    public void initialise() {
        //TODO register movement modes with the mover
    }

    @Override
    public void shutdown() {
    }
    
    @ReceiveEvent()
    public void onUpdateMode(UpdateMovementModeEvent event, EntityRef character) {
        updateMode(event.getResultState(), event.getInitialState(), character, event.isFirstRun());
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
    private void updateMode(final CharacterStateEvent state,
            final CharacterStateEvent oldState, EntityRef entity, boolean firstRun) {
        CharacterMovementComponent movementComp = entity.getComponent(CharacterMovementComponent.class);
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
}
