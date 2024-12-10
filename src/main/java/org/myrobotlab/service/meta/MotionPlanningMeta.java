package org.myrobotlab.service.meta;

import org.myrobotlab.logging.LoggerFactory;
import org.myrobotlab.service.meta.abstracts.MetaData;
import org.slf4j.Logger;

public class MotionPlanningMeta extends MetaData {
  private static final long serialVersionUID = 1L;
  public final static Logger log = LoggerFactory.getLogger(MotionPlanningMeta.class);

  /**
   * This class is contains all the meta data details of a service. It's peers,
   * dependencies, and all other meta data related to the service.
   */
  public MotionPlanningMeta() {
    addDescription("N-dimensional motion planner with several algorithms, including RRT*");
    // add dependency if necessary
    addDependency("ch.ethz.idsc", "owl", "0.5.9");
    addCategory("robotics");

  }

}
