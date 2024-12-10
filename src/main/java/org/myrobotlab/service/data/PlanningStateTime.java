package org.myrobotlab.service.data;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

public class PlanningStateTime {
    public Tensor state;
    public Scalar time;
    public PlanningStateTime(Tensor state, Scalar time) {
        this.state = state;
        this.time = time;
    }

    public double get(int index) {
        return state.Get(index).number().doubleValue();
    }

}
