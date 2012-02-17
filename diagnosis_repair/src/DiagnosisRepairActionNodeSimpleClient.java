import org.ros.actionlib.client.SimpleActionClient;
import org.ros.exception.RosException;
import org.ros.message.diagnosis_msgs.DiagnosisRepairAction;
import org.ros.message.diagnosis_msgs.DiagnosisRepairActionFeedback;
import org.ros.message.diagnosis_msgs.DiagnosisRepairActionGoal;
import org.ros.message.diagnosis_msgs.DiagnosisRepairActionResult;
import org.ros.message.diagnosis_msgs.DiagnosisRepairFeedback;
import org.ros.message.diagnosis_msgs.DiagnosisRepairGoal;
import org.ros.message.diagnosis_msgs.DiagnosisRepairResult;


public class DiagnosisRepairActionNodeSimpleClient
    extends
    SimpleActionClient<DiagnosisRepairActionFeedback, DiagnosisRepairActionGoal, DiagnosisRepairActionResult, DiagnosisRepairFeedback, DiagnosisRepairGoal, DiagnosisRepairResult> {

  public DiagnosisRepairActionNodeSimpleClient(String nameSpace, DiagnosisRepairActionNodeSpec spec)
      throws RosException {
    super(nameSpace, spec);
  }

}
