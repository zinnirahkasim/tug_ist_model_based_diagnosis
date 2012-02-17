import org.ros.actionlib.ActionSpec;
import org.ros.actionlib.server.ActionServerCallbacks;
import org.ros.actionlib.server.SimpleActionServerCallbacks;
import org.ros.exception.RosException;

import org.ros.message.diagnosis_msgs.DiagnosisRepairAction;
import org.ros.message.diagnosis_msgs.DiagnosisRepairActionFeedback;
import org.ros.message.diagnosis_msgs.DiagnosisRepairActionGoal;
import org.ros.message.diagnosis_msgs.DiagnosisRepairActionResult;
import org.ros.message.diagnosis_msgs.DiagnosisRepairFeedback;
import org.ros.message.diagnosis_msgs.DiagnosisRepairGoal;
import org.ros.message.diagnosis_msgs.DiagnosisRepairResult;


public class DiagnosisRepairActionNodeSpec
    extends
    ActionSpec<DiagnosisRepairAction, DiagnosisRepairActionFeedback, DiagnosisRepairActionGoal, DiagnosisRepairActionResult, DiagnosisRepairFeedback, DiagnosisRepairGoal, DiagnosisRepairResult> {

  public DiagnosisRepairActionNodeSpec() throws RosException {
    super(DiagnosisRepairAction.class, "repair/DiagnosisRepairAction",
        "repair/DiagnosisRepairActionFeedback", "repair/DiagnosisRepairActionGoal",
        "repair/DiagnosisRepairActionResult", "repair/DiagnosisRepairFeedback",
        "repair/DiagnosisRepairGoal", "repair/DiagnosisRepairResult");
  }

  @Override
  public DiagnosisRepairActionNodeClient buildActionClient(String nameSpace) {

    DiagnosisRepairActionNodeClient ac = null;
    try {
      ac = new DiagnosisRepairActionNodeClient(nameSpace, this);
    } catch (RosException e) {
      e.printStackTrace();
    }
    return ac;

  }

  @Override
  public DiagnosisRepairActionNodeSimpleClient buildSimpleActionClient(String nameSpace) {

    DiagnosisRepairActionNodeSimpleClient sac = null;
    try {
      return new DiagnosisRepairActionNodeSimpleClient(nameSpace, this);
    } catch (RosException e) {
      e.printStackTrace();
    }
    return sac;

  }

  @Override
  public
      DiagnosisRepairActionNodeServer
      buildActionServer(
          String nameSpace,
          ActionServerCallbacks<DiagnosisRepairActionFeedback, DiagnosisRepairActionGoal, DiagnosisRepairActionResult, DiagnosisRepairFeedback, DiagnosisRepairGoal, DiagnosisRepairResult> callbacks) {

    return new DiagnosisRepairActionNodeServer(nameSpace, this, callbacks);

  }

  @Override
  public
      DiagnosisRepairActionNodeSimpleServer
      buildSimpleActionServer(
          String nameSpace,
          SimpleActionServerCallbacks<DiagnosisRepairActionFeedback, DiagnosisRepairActionGoal, DiagnosisRepairActionResult, DiagnosisRepairFeedback, DiagnosisRepairGoal, DiagnosisRepairResult> callbacks,
          boolean useBlockingGoalCallback) {

    return new DiagnosisRepairActionNodeSimpleServer(nameSpace, this, callbacks, useBlockingGoalCallback);

  }

}
