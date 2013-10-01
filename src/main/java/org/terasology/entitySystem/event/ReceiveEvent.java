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
package org.terasology.entitySystem.event;

import org.terasology.entitySystem.Component;
import org.terasology.entitySystem.RegisterMode;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * This annotation is used to mark up methods that can be registered to receive events through the EventSystem
 * <p/>
 * These methods should have the form
 * <code>public void handlerMethod(EventType event, EntityRef entity)</code>
 *
 * @author Immortius <immortius@gmail.com>
 */
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.METHOD)
public @interface ReceiveEvent {
    /**
     * What components that the entity must have for this method to be invoked
     */
    Class<? extends Component>[] components() default {};

    
    /**
     * The netFilter allows you to set when to register for events.
     * The default is to always register. For the different options, see the 
     * RegisterMode enum.
     */
    RegisterMode netFilter() default RegisterMode.ALWAYS;

    /**
     * Sets the priority for receiving events.
     * </p>
     * While you can use integers numbers, it is common practice to use the
     * EventPriority constants for this. The default is
     * EventPriority.PRIORITY_NORMAL.
     * </p>
     * When using integer numbers, higher numbers mean higher priority, meaning
     * you receive the event before systems with a lower priority value.
     */
    int priority() default EventPriority.PRIORITY_NORMAL;
}
