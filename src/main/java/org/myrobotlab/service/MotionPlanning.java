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
import com.jme3.math.Vector3f;
import com.jme3.scene.Node;
import org.myrobotlab.framework.Service;
import org.myrobotlab.framework.interfaces.ServiceInterface;
import org.myrobotlab.kinematics.DHRobotArm;
import org.myrobotlab.kinematics.Point;
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

  public static class JMEToIK {
    private final Point scales;
    private final Point offset;
    public JMEToIK(Point scales, Point offset) {
      this.scales = scales;
      this.offset = offset;
    }

    public Point toIK(Point jme) {
      Point scaled = new Point(jme.getX() * scales.getX(), jme.getY() * scales.getY(), jme.getZ() * scales.getZ());
      return scaled.add(offset);

    }

    public Point toIK(Vector3f jme) {
      Point scaled = new Point(jme.getX() * scales.getX(), jme.getZ() * scales.getY(), jme.getY() * scales.getZ());
      return scaled.add(offset);

    }

    public Vector3f toJME(Point ik) {
      Point translated = ik.subtract(offset);
      return new Vector3f((float) (translated.getX()/ scales.getX()), (float) (translated.getY()/ scales.getY()), (float) (translated.getZ()/ scales.getZ()));
    }
  }

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


  public Tensor plan(Tensor state) {
    StateTime stateTime = new StateTime(state, RealScalar.ZERO);

    server.insertRoot(stateTime);
    new Expand<>(server).steps(1000);
    List<TrajectorySample> trajectory = server.getTrajectory().orElse(new ArrayList<>());
    List<PlanningStateTime> plan = trajectory.stream().map(
            sample -> new PlanningStateTime(sample.stateTime().state(), sample.stateTime().time())
    ).collect(Collectors.toList());
    invoke("publishPlan", plan);
    return plan.get(plan.size()-1).state;
  }

  public MotionPlanning(String n, String id) {
    super(n, id);

  }

  @Override
  public MotionPlanningConfig apply(MotionPlanningConfig c) {
    super.apply(c);


    // ---
    server = new DefaultRrtsPlannerServer( //
            RnTransitionSpace.INSTANCE, //
            region, //
            config.resolution(), //
            Se2StateSpaceModel.INSTANCE, //
            LengthCostFunction.INSTANCE) {
      @Override
      protected RrtsNodeCollection rrtsNodeCollection() {
        return new RnRrtsNodeCollection(config.lowerBounds, config.upperBounds);
      }

      @Override
      protected RandomSampleInterface spaceSampler(Tensor state) {
        return BoxRandomSample.of(config.lowerBounds, config.upperBounds);
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

//      Runtime.start("webgui", "WebGui");


      MotionPlanning planner = (MotionPlanning)Runtime.start("planner", "MotionPlanning");
      assert planner != null;
//      LoggingFactory.init(Level.DEBUG);
      InMoov2 i01 = (InMoov2)Runtime.start("i01", "InMoov2");

      assert i01 != null;
      i01.setAllVirtual(true);

      InMoov2Arm arm = (InMoov2Arm)i01.startPeer("rightArm");
      arm.setSpeed(500.0, 500.0, 500.0, 500.0);
      JMonkeyEngine simulator = (JMonkeyEngine)i01.startPeer("simulator");
      simulator.setDisplayStatView(true);
      double boxSize = 0.2;
      Node box = simulator.addBox("box", boxSize, boxSize, boxSize, "0xff0000", false);

      simulator.moveTo("box", -0.3, 2.75, 0.5);

      sleep(500);

      DHRobotArm dhRobotArm = InMoov2Arm.getDHRobotArm(i01.getName(), "right");

      InMoovTransitionRegionQuery region = new InMoovTransitionRegionQuery(dhRobotArm, simulator);

      planner.config.resolutionDenominator = 15;
      planner.config.lowerBounds = Tensors.vector(dhRobotArm.getLinks().stream().map(link -> Math.toDegrees(link.getMin()) + link.offset).toArray(Double[]::new));
      planner.config.upperBounds = Tensors.vector(dhRobotArm.getLinks().stream().map(link -> Math.toDegrees(link.getMax()) + link.offset).toArray(Double[]::new));
      planner.setRegion(region);
      arm.subscribe(planner, "publishPlan");


      // First, get the arm's starting position
      dhRobotArm.centerAllJoints();
      Tensor state = Tensors.vector(dhRobotArm.getLinks().stream().map(link -> link.getThetaDegrees() + link.offset).toArray(Double[]::new));

      arm.moveToBlocking(
              state.Get(0).number().doubleValue(),
              state.Get(1).number().doubleValue(),
              state.Get(2).number().doubleValue(),
              state.Get(3).number().doubleValue());

      sleep(1000);
      Point startPos = dhRobotArm.getPalmPosition();
      Point initialPos = new Point(startPos);
      Node wrist = simulator.getNode("i01.rightHand.wrist");
      Node bicep = simulator.getNode("i01.rightArm.bicep");
      Node rotate = simulator.getNode("i01.rightArm.rotate");
      Node shoulder = simulator.getNode("i01.rightArm.shoulder");
      Node omoplate = simulator.getNode("i01.rightArm.omoplate");
      log.info("Wrist location in JME: {}", wrist.getWorldTransform().getTranslation());
      log.info("Bicep location in JME: {}", bicep.getWorldTransform().getTranslation());
      log.info("Rotate location in JME: {}", rotate.getWorldTransform().getTranslation());
      log.info("Shoulder location in JME: {}", shoulder.getWorldTransform().getTranslation());
      log.info("Omoplate location in JME: {}", omoplate.getWorldTransform().getTranslation());
      Vector3f wristInitialPosVec = wrist.getWorldTransform().getTranslation();
      Point wristInitialPos = new Point(wristInitialPosVec.x, wristInitialPosVec.y, wristInitialPosVec.z);
      Point initialDifferences = initialPos.subtract(wristInitialPos);
      log.info(initialDifferences.toString());

//      Tensor state = Tensors.vector(-Math.PI/2, Math.PI/2, 0, Math.PI);
//      dhRobotArm.centerAllJoints();


      dhRobotArm.getLink(0).rotate(dhRobotArm.getLink(0).getMax()-Math.PI/4);
      dhRobotArm.getLink(1).rotate(dhRobotArm.getLink(1).getMax()-Math.PI/4);

      dhRobotArm.getLink(2).rotate(dhRobotArm.getLink(2).getMax()-Math.PI/4);

      dhRobotArm.getLink(3).rotate(dhRobotArm.getLink(3).getMax()-Math.PI/4);
      state = Tensors.vector(dhRobotArm.getLinks().stream().map(link -> link.getThetaDegrees() + link.offset).toArray(Double[]::new));

      arm.moveToBlocking(
              state.Get(0).number().doubleValue(),
              state.Get(1).number().doubleValue(),
              state.Get(2).number().doubleValue(),
              state.Get(3).number().doubleValue());
      sleep(1000);
      startPos = dhRobotArm.getPalmPosition();

      // Now we need to find how far it moved
      Vector3f wristEndPosVec = wrist.getWorldTransform().getTranslation();
      Point wristEndPos = new Point(wristEndPosVec.x, wristEndPosVec.y, wristEndPosVec.z);
      Point ikDifference = startPos.subtract(initialPos);
      Point wristDifference = wristEndPos.subtract(wristInitialPos);
      Point scales = new Point(ikDifference.getX() / wristDifference.getX(),ikDifference.getY() / -wristDifference.getZ(), ikDifference.getZ() / wristDifference.getY());
      // ik = scales * wrist +

      log.info(scales.toString());
      Point estimatedInitial = new Point(wristInitialPos.getX() * scales.getX(), wristInitialPos.getZ() * scales.getY(), wristInitialPos.getY() * scales.getZ());
      Point estimatedInitialError = initialPos.subtract(estimatedInitial);
      Point estimatedEnd = new Point(wristEndPos.getX() * scales.getX(), wristEndPos.getZ() * scales.getY(), wristEndPos.getY() * scales.getZ());
      estimatedEnd = estimatedEnd.add(estimatedInitialError);
      log.info(estimatedEnd.toString());

      JMEToIK converter = new JMEToIK(scales, estimatedInitialError);
      Vector3f boxCenterVec = new Vector3f(box.getWorldTransform().getTranslation());
      Point boxCenterIK = converter.toIK(boxCenterVec);

      Point boxBottomLeftCorner = converter.toIK(boxCenterVec.add((float) (-boxSize/2), (float) (-boxSize/2), (float) (-boxSize/2)));
      Point boxTopRightCorner = converter.toIK(boxCenterVec.add((float) (boxSize/2), (float) (boxSize/2), (float) (boxSize/2)));
      log.info("Box bottom left: {}", boxBottomLeftCorner.toString());
      log.info("Box top right:{}", boxTopRightCorner.toString());
      log.info("Box center:{}", boxCenterIK.toString());


      log.info(startPos.toString());

      Point endPos = new Point(100, 0, 460);
      region.enableCollision = true;
      double destBoxSize = 0.1f;
      Node destBox = simulator.addBox("destBox", destBoxSize, destBoxSize, destBoxSize, "0000ff", true);
      Vector3f destBoxCoords = converter.toJME(endPos);
      simulator.moveTo("destBox", destBoxCoords.x, destBoxCoords.y, destBoxCoords.z);
      log.info("Destination box bottom left corner: {}", converter.toIK(destBoxCoords.subtract(0.05f, 0.05f, 0.05f)));
      log.info("Destination box top right corner: {}", converter.toIK(destBoxCoords.add(0.05f, 0.05f, 0.05f)));
      log.info("Destination box center: {}", converter.toIK(destBoxCoords));


      sleep(1000);
      dhRobotArm.moveToGoal(endPos);

      Tensor goal = Tensors.vector(dhRobotArm.getLinks().stream().map(link -> link.getThetaDegrees() + link.offset).toArray(Double[]::new));

//      Tensor goal = Tensors.vector(-Math.PI/2, Math.PI/2, Math.PI/2, Math.PI);


      planner.setGoal(goal);

      Tensor newState = planner.plan(state);

//      endPos = converter.toIK(new Point(0.0f, 3.0f, 0.4));
//
//      sleep(7000);
//      wristEndPosVec = wrist.getWorldTransform().getTranslation();
//      log.info("Final wrist position in JME: {}", wristEndPosVec);
//      log.info("Final wrist position in IK: {}", converter.toIK(wristEndPosVec));
//      log.info("IK joint angles: {}", dhRobotArm.getLinks().stream().map(link -> link.getThetaDegrees() + link.offset).collect(Collectors.toList()));
//
//      wrist = simulator.getNode("i01.rightHand.wrist");
//      bicep = simulator.getNode("i01.rightArm.bicep");
//      rotate = simulator.getNode("i01.rightArm.rotate");
//      shoulder = simulator.getNode("i01.rightArm.shoulder");
//      omoplate = simulator.getNode("i01.rightArm.omoplate");
//      log.info("Wrist location in JME: {}", wrist.getWorldTransform().getTranslation());
//      log.info("Bicep location in JME: {}", bicep.getWorldTransform().getTranslation());
//      log.info("Rotate location in JME: {}", rotate.getWorldTransform().getTranslation());
//      log.info("Shoulder location in JME: {}", shoulder.getWorldTransform().getTranslation());
//      log.info("Omoplate location in JME: {}", omoplate.getWorldTransform().getTranslation());
//
//      planner.apply(planner.config);
//      dhRobotArm.moveToGoal(endPos);
//
//      goal = Tensors.vector(dhRobotArm.getLinks().stream().map(link -> link.getThetaDegrees() + link.offset).toArray(Double[]::new));

//      planner.setGoal(goal);
//      newState = planner.plan(newState);

      endPos = converter.toIK(boxCenterVec.add(0.25f, 0.0f, 0.25f));


      sleep(6000);
      planner.apply(planner.config);
      dhRobotArm.moveToGoal(endPos);

      goal = Tensors.vector(dhRobotArm.getLinks().stream().map(link -> link.getThetaDegrees() + link.offset).toArray(Double[]::new));
      planner.setGoal(goal);
      newState = planner.plan(newState);
    } catch (Exception e) {
      log.error("main threw", e);
    }
  }
}
