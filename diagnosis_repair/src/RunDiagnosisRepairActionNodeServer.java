import org.ros.node.DefaultNodeRunner;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.NodeRunner;


public class RunDiagnosisRepairActionNodeServer {

  public static void main(String[] args) {
    main();
  }

  public static void main() {
    try {
      // user code implementing the SimpleActionServerCallbacks interface
      DiagnosisRepairActionNodeCallbacks impl = new DiagnosisRepairActionNodeCallbacks();

      DiagnosisRepairActionNodeSpec spec = new DiagnosisRepairActionNodeSpec();
      final DiagnosisRepairActionNodeSimpleServer sas =
          spec.buildSimpleActionServer("diagnosis_repair_server_node", impl, true);

      NodeConfiguration configuration = NodeConfiguration.newPrivate();
      configuration.setNodeName("RepairActionNodeServer");
      NodeRunner runner = DefaultNodeRunner.newDefault();

      runner.run(new NodeMain() {

        @Override
        public void onStart(Node node) {
          sas.addClientPubSub(node);
        }
        
        @Override
        public void onShutdown(Node node) {
        }

      }, configuration);
    } catch (Exception e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }

}
