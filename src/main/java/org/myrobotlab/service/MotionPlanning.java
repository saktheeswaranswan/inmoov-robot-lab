package org.myrobotlab.service;

import ch.ethz.idsc.owl.bot.rn.RnTransitionSpace;
import ch.ethz.idsc.owl.bot.rn.rrts.RnRrtsFlow;
import ch.ethz.idsc.owl.bot.rn.rrts.RnRrtsNodeCollection;
import ch.ethz.idsc.owl.bot.se2.Se2StateSpaceModel;
import ch.ethz.idsc.owl.data.tree.Expand;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owl.math.state.TrajectorySample;
import ch.ethz.idsc.owl.rrts.DefaultRrtsPlannerServer;
import ch.ethz.idsc.owl.rrts.RrtsPlannerServer;
import ch.ethz.idsc.owl.rrts.adapter.EmptyTransitionRegionQuery;
import ch.ethz.idsc.owl.rrts.adapter.LengthCostFunction;
import ch.ethz.idsc.owl.rrts.core.RrtsNodeCollection;
import ch.ethz.idsc.owl.rrts.core.TransitionRegionQuery;
import ch.ethz.idsc.sophus.math.sample.BoxRandomSample;
import ch.ethz.idsc.sophus.math.sample.ConstantRandomSample;
import ch.ethz.idsc.sophus.math.sample.RandomSampleInterface;
import ch.ethz.idsc.tensor.*;
import org.myrobotlab.framework.Service;
import org.myrobotlab.framework.interfaces.ServiceInterface;
import org.myrobotlab.kinematics.DHRobotArm;
import org.myrobotlab.logging.Level;
import org.myrobotlab.logging.LoggerFactory;
import org.myrobotlab.logging.LoggingFactory;
import org.myrobotlab.planning.InMoovTransitionRegionQuery;
import org.myrobotlab.service.config.MotionPlanningConfig;
import org.myrobotlab.service.data.PlanningStateTime;
import org.slf4j.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

public class MotionPlanning extends Service<MotionPlanningConfig>
{

  private static final long serialVersionUID = 1L;

  public final static Logger log = LoggerFactory.getLogger(MotionPlanning.class);
  private RrtsPlannerServer server;
  private TransitionRegionQuery region = EmptyTransitionRegionQuery.INSTANCE;


  public void setGoal(Tensor goal) {
    server.setGoal(goal);
  }

  public void setRegion(TransitionRegionQuery region) {
    this.region = region;
    apply(config);
  }

  public List<PlanningStateTime> publishPlan(List<PlanningStateTime> plan) {
    for (PlanningStateTime sample : plan) {
        log.info("{}: {}", sample.state.toString(), sample.time.toString());

    }
    return plan;
  }


  public void plan(Tensor state) {
    StateTime stateTime = new StateTime(state, RealScalar.ZERO);

    server.insertRoot(stateTime);
    new Expand<>(server).steps(400);
    List<TrajectorySample> trajectory = server.getTrajectory().orElse(new ArrayList<>());
    List<PlanningStateTime> plan = trajectory.stream().map(
            sample -> new PlanningStateTime(sample.stateTime().state(), sample.stateTime().time())
    ).collect(Collectors.toList());
    invoke("publishPlan", plan);

  }

  public MotionPlanning(String n, String id) {
    super(n, id);

  }

  @Override
  public MotionPlanningConfig apply(MotionPlanningConfig c) {
    super.apply(c);

    Tensor lbounds = Tensors.vector(-4, -4, -4, -4);
    Tensor ubounds = Tensors.vector(4, 4, 4, 4);
    // ---
    server = new DefaultRrtsPlannerServer( //
            RnTransitionSpace.INSTANCE, //
            region, //
            config.resolution(), //
            Se2StateSpaceModel.INSTANCE, //
            LengthCostFunction.INSTANCE) {
      @Override
      protected RrtsNodeCollection rrtsNodeCollection() {
        return new RnRrtsNodeCollection(lbounds, ubounds);
      }

      @Override
      protected RandomSampleInterface spaceSampler(Tensor state) {
        return BoxRandomSample.of(lbounds, ubounds);
      }

      @Override
      protected RandomSampleInterface goalSampler(Tensor goal) {
        return new ConstantRandomSample(goal);
      }

      @Override
      protected Tensor uBetween(StateTime orig, StateTime dest) {
        return RnRrtsFlow.uBetween(orig, dest);
      }
    };

    return config;
  }

  public static void main(String[] args) {
    try {

      LoggingFactory.init(Level.INFO);

      MotionPlanning planner = (MotionPlanning)Runtime.start("planner", "MotionPlanning");
      assert planner != null;

      InMoov2 i01 = (InMoov2)Runtime.start("i01", "InMoov2");

      assert i01 != null;
      i01.setAllVirtual(true);
      ServiceInterface arm = i01.startPeer("rightArm");
      ServiceInterface simulator = i01.startPeer("simulator");
      DHRobotArm dhRobotArm = InMoov2Arm.getDHRobotArm(i01.getName(), "right");

      InMoovTransitionRegionQuery region = new InMoovTransitionRegionQuery(dhRobotArm);
      sleep(1000);

      planner.config.resolutionDenominator = 15;
      planner.setRegion(region);
      arm.subscribe(planner, "publishPlan");
//      Runtime.start("webgui", "WebGui");


//      Tensor state = Tensors.vector(-Math.PI/2, Math.PI/2, 0, Math.PI);
//      dhRobotArm.centerAllJoints();
      dhRobotArm.getLink(0).rotate(dhRobotArm.getLink(0).getMax());
      dhRobotArm.getLink(1).rotate(dhRobotArm.getLink(1).getMax());

      dhRobotArm.getLink(2).rotate(dhRobotArm.getLink(2).getMax());

      dhRobotArm.getLink(3).rotate(dhRobotArm.getLink(3).getMax());
      Tensor state = Tensors.vector(
              dhRobotArm.getLink(0).getTheta(),
              dhRobotArm.getLink(1).getTheta(),
              dhRobotArm.getLink(2).getTheta(),
              dhRobotArm.getLink(3).getTheta());
      dhRobotArm.getLink(0).incrRotate(-Math.PI/4);
      dhRobotArm.getLink(1).incrRotate(-Math.PI/4);
      dhRobotArm.getLink(2).incrRotate(-Math.PI/4);
//      dhRobotArm.getLink(0).rotate(dhRobotArm.getLink(0).getMin());
//      dhRobotArm.getLink(1).rotate(dhRobotArm.getLink(1).getMin());
//
//      dhRobotArm.getLink(2).rotate(dhRobotArm.getLink(2).getMin());

      dhRobotArm.getLink(3).rotate(dhRobotArm.getLink(3).getMin());
      Tensor goal = Tensors.vector(
              dhRobotArm.getLink(0).getTheta(),
              dhRobotArm.getLink(1).getTheta(),
              dhRobotArm.getLink(2).getTheta(),
              dhRobotArm.getLink(3).getTheta());

//      Tensor goal = Tensors.vector(-Math.PI/2, Math.PI/2, Math.PI/2, Math.PI);


      planner.setGoal(goal);
      planner.plan(state);


    } catch (Exception e) {
      log.error("main threw", e);
    }
  }
}
