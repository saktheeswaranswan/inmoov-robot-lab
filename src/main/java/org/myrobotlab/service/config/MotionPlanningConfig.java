package org.myrobotlab.service.config;

import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

public class MotionPlanningConfig extends ServiceConfig {

    public int resolutionNumerator = 1;
    public int resolutionDenominator = 10;

    public Tensor lowerBounds = Tensors.vector(0);
    public Tensor upperBounds = Tensors.vector(1);

    public Scalar resolution() {
        return RationalScalar.of(resolutionNumerator, resolutionDenominator);
    }
}
