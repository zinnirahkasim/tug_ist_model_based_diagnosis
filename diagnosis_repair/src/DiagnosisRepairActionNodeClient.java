import org.ros.actionlib.client.ActionClient;
import org.ros.exception.RosException;
import org.ros.message.diagnosis_msgs.DiagnosisRepairActionFeedback;
import org.ros.message.diagnosis_msgs.DiagnosisRepairActionGoal;
import org.ros.message.diagnosis_msgs.DiagnosisRepairActionResult;
import org.ros.message.diagnosis_msgs.DiagnosisRepairFeedback;
import org.ros.message.diagnosis_msgs.DiagnosisRepairGoal;
import org.ros.message.diagnosis_msgs.DiagnosisRepairResult;

public class DiagnosisRepairActionNodeClient
    extends
    ActionClient<DiagnosisRepairActionFeedback, DiagnosisRepairActionGoal, DiagnosisRepairActionResult, DiagnosisRepairFeedback, DiagnosisRepairGoal, DiagnosisRepairResult> {

  public DiagnosisRepairActionNodeClient(String nameSpace, DiagnosisRepairActionNodeSpec spec) throws RosException {
    super(nameSpace, spec);
  }
}
