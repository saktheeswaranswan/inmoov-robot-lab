package org.myrobotlab.planning;

import ch.ethz.idsc.owl.rrts.core.Transition;
import ch.ethz.idsc.owl.rrts.core.TransitionRegionQuery;
import ch.ethz.idsc.tensor.Tensor;
import org.myrobotlab.kinematics.DHRobotArm;


/**
 * This is the class that determines whether a given transition
 * is valid, it's the heart of the problem description.
 * <p>
 * Right now it supports the right arm of InMoov. The joint
 * angles are given in the state tensor ordered as:
 * <ul>
 *     <li>Shoulder piston (right omoplate)</li>
 *     <li>Shoulder rotation (right shoulder)</li>
 *     <li>Bicep rotation (right rotate)</li>
 *     <li>Bicep piston (right bicep)</li>
 * </ul>
 */
public class InMoovTransitionRegionQuery implements TransitionRegionQuery {
    private final DHRobotArm arm;

    public InMoovTransitionRegionQuery(DHRobotArm arm) {
        this.arm = arm;
    }





    @Override
    public boolean isDisjoint(Transition transition) {
        // Check if angles within bounds
        Tensor goalState = transition.end();
        for (int i = 0; i < goalState.length(); i++) {
            boolean inRange = arm.getLink(i).rotate(goalState.Get(i).number().doubleValue());
            if (!inRange) {
                return false;
            }
        }
        return true;
    }
}
