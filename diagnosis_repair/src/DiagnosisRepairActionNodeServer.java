import org.ros.actionlib.ActionSpec;
import org.ros.actionlib.server.ActionServerCallbacks;
import org.ros.actionlib.server.DefaultActionServer;
import org.ros.message.diagnosis_msgs.DiagnosisRepairActionFeedback;
import org.ros.message.diagnosis_msgs.DiagnosisRepairActionGoal;
import org.ros.message.diagnosis_msgs.DiagnosisRepairActionResult;
import org.ros.message.diagnosis_msgs.DiagnosisRepairFeedback;
import org.ros.message.diagnosis_msgs.DiagnosisRepairGoal;
import org.ros.message.diagnosis_msgs.DiagnosisRepairResult;

public class DiagnosisRepairActionNodeServer
    extends
    DefaultActionServer<DiagnosisRepairActionFeedback, DiagnosisRepairActionGoal, DiagnosisRepairActionResult, DiagnosisRepairFeedback, 
    DiagnosisRepairGoal,  DiagnosisRepairResult> {

      public DiagnosisRepairActionNodeServer(
      String nameSpace,
      ActionSpec<?, DiagnosisRepairActionFeedback, DiagnosisRepairActionGoal, DiagnosisRepairActionResult, DiagnosisRepairFeedback, 
      DiagnosisRepairGoal, DiagnosisRepairResult> spec,
      ActionServerCallbacks<DiagnosisRepairActionFeedback, DiagnosisRepairActionGoal, DiagnosisRepairActionResult, DiagnosisRepairFeedback, 
      DiagnosisRepairGoal, DiagnosisRepairResult> callbacks) {

    super(nameSpace, spec, callbacks);

  }
}
