package org.myrobotlab.service.config;

import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.Scalar;

public class MotionPlanningConfig extends ServiceConfig {

    public int resolutionNumerator = 1;
    public int resolutionDenominator = 10;

    public Scalar resolution() {
        return RationalScalar.of(resolutionNumerator, resolutionDenominator);
    }
}
