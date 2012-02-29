import org.ros.actionlib.server.SimpleActionServer;
import org.ros.actionlib.server.SimpleActionServerCallbacks;
import org.ros.message.diagnosis_msgs.DiagnosisRepairActionFeedback;
import org.ros.message.diagnosis_msgs.DiagnosisRepairActionGoal;
import org.ros.message.diagnosis_msgs.DiagnosisRepairActionResult;
import org.ros.message.diagnosis_msgs.DiagnosisRepairFeedback;
import org.ros.message.diagnosis_msgs.DiagnosisRepairGoal;
import org.ros.message.diagnosis_msgs.DiagnosisRepairResult;

import java.util.ArrayList;

public class DiagnosisRepairActionNodeCallbacks
    implements
    SimpleActionServerCallbacks<DiagnosisRepairActionFeedback, DiagnosisRepairActionGoal, DiagnosisRepairActionResult, DiagnosisRepairFeedback, 
    DiagnosisRepairGoal, DiagnosisRepairResult> {

 @Override
  public
      void
      blockingGoalCallback(
          DiagnosisRepairGoal goal,
          SimpleActionServer<DiagnosisRepairActionFeedback, DiagnosisRepairActionGoal, DiagnosisRepairActionResult, DiagnosisRepairFeedback, 
          DiagnosisRepairGoal, DiagnosisRepairResult> actionServer) {

    String[] params = (String[]) goal.parameter.toArray(new String[0]);   
    
    
    System.out.println("BLOCKING GOAL CALLBACK");
    System.out.print("Parameters Received:");
    for(int i=0; i<params.length; i++)
       System.out.print(" "+params[i]+" ");
    System.out.println();
    DiagnosisRepairResult result = new DiagnosisRepairResult();
    result.result = 0;
    actionServer.setSucceeded(result, "");

  }

  @Override
  public
      void
      goalCallback(
          SimpleActionServer<DiagnosisRepairActionFeedback, DiagnosisRepairActionGoal, DiagnosisRepairActionResult, DiagnosisRepairFeedback, DiagnosisRepairGoal, DiagnosisRepairResult> actionServer) {
    System.out.println("GOAL CALLBACK");
  }

  @Override
  public
      void
      preemptCallback(
          SimpleActionServer<DiagnosisRepairActionFeedback, DiagnosisRepairActionGoal, DiagnosisRepairActionResult, DiagnosisRepairFeedback, DiagnosisRepairGoal, DiagnosisRepairResult> actionServer) {
    System.out.println("PREEMPT CALLBACK");
  }

  private void snore() {

    try {
      Thread.sleep(1000);
    } catch (InterruptedException e) {
    }

  }

  private
      void
      publishFeedback(
          int[] seq,
          SimpleActionServer<DiagnosisRepairActionFeedback, DiagnosisRepairActionGoal, DiagnosisRepairActionResult, DiagnosisRepairFeedback, DiagnosisRepairGoal, DiagnosisRepairResult> actionServer) {
    DiagnosisRepairFeedback feedback = new DiagnosisRepairFeedback();
    feedback.feedback = 1;
    actionServer.publishFeedback(feedback);


  }

}

