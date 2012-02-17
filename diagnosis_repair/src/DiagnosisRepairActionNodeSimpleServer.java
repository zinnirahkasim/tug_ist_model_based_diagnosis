import org.ros.actionlib.ActionSpec;
import org.ros.actionlib.server.DefaultSimpleActionServer;
import org.ros.actionlib.server.SimpleActionServerCallbacks;
import org.ros.message.diagnosis_msgs.DiagnosisRepairAction;
import org.ros.message.diagnosis_msgs.DiagnosisRepairActionFeedback;
import org.ros.message.diagnosis_msgs.DiagnosisRepairActionGoal;
import org.ros.message.diagnosis_msgs.DiagnosisRepairActionResult;
import org.ros.message.diagnosis_msgs.DiagnosisRepairFeedback;
import org.ros.message.diagnosis_msgs.DiagnosisRepairGoal;
import org.ros.message.diagnosis_msgs.DiagnosisRepairResult;


public class DiagnosisRepairActionNodeSimpleServer
    extends
    DefaultSimpleActionServer<DiagnosisRepairActionFeedback, DiagnosisRepairActionGoal, DiagnosisRepairActionResult, DiagnosisRepairFeedback, DiagnosisRepairGoal, DiagnosisRepairResult> {

  public DiagnosisRepairActionNodeSimpleServer(
      String nameSpace,
      ActionSpec<?, DiagnosisRepairActionFeedback, DiagnosisRepairActionGoal, DiagnosisRepairActionResult, DiagnosisRepairFeedback, DiagnosisRepairGoal, DiagnosisRepairResult> spec,
      SimpleActionServerCallbacks<DiagnosisRepairActionFeedback, DiagnosisRepairActionGoal, DiagnosisRepairActionResult, DiagnosisRepairFeedback, DiagnosisRepairGoal, DiagnosisRepairResult> callbacks,
      boolean useBlockingGoalCallback) {
    super(nameSpace, spec, callbacks, useBlockingGoalCallback);
  }

}
