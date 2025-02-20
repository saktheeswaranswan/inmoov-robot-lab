package org.myrobotlab.planning;

import ch.ethz.idsc.owl.rrts.core.Transition;
import ch.ethz.idsc.owl.rrts.core.TransitionRegionQuery;
import ch.ethz.idsc.tensor.Tensor;
import org.myrobotlab.kinematics.DHLink;
import org.myrobotlab.kinematics.DHRobotArm;
import org.myrobotlab.kinematics.Point;
import org.myrobotlab.service.JMonkeyEngine;


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
    private final JMonkeyEngine jme;
    public boolean enableCollision = false;

    public InMoovTransitionRegionQuery(DHRobotArm arm, JMonkeyEngine jme) {
        this.arm = arm;
        this.jme = jme;
    }





    @Override
    public boolean isDisjoint(Transition transition) {
        // Check if angles within bounds
        Tensor goalState = transition.end();
        Point shoulderPos = arm.getJointPosition(0);


        for (int i = 0; i < goalState.length(); i++) {
            DHLink link = arm.getLink(i);
            double goalAngle = goalState.Get(i).number().doubleValue();
            double min = Math.toDegrees(link.getMin()) + link.offset - 0.001;
            double max = Math.toDegrees(link.getMax()) + link.offset + 0.001;
            boolean inRange = goalAngle > min && goalAngle < max;
            if (!inRange) {
                System.err.println("Goal angle " + goalAngle + " out of range [" + min + ", " + max + "]");
                return false;
            }

//            Point diff = currentPos.subtract(shoulderPos);
//            if (diff.getX() > 0 && diff.getY() < 0) {
//                return false;
//            }
        }

        Point currentPos = arm.getPalmPosition();


        if (collides(currentPos)) {
            return false;
        }

        currentPos = arm.getJointPosition(3);

        if (collides(currentPos)) {
            return false;
        }

        currentPos = arm.getJointPosition(2);
        if (collides(currentPos)) {
            return false;
        }

        currentPos = arm.getJointPosition(1);
        if (collides(currentPos)) {
            return false;
        }

        currentPos = arm.getJointPosition(0);
        if (collides(currentPos)) {
            return false;
        }

        currentPos = arm.getPalmPosition().subtract(arm.getJointPosition(0));
        currentPos = arm.getJointPosition(0).add(new Point(currentPos.getX()/2, currentPos.getY()/2, currentPos.getZ()/2));
        if (collides(currentPos)) {
            return false;
        }

        return true;
    }

    public boolean collides(Point point) {
        double x = point.getX();
        double y = point.getY();
        double z = point.getZ();
        return enableCollision && x > 1060 && x < 1573 && y > -141 && y < -43 && z > 315 && z < 392;

    }
}
