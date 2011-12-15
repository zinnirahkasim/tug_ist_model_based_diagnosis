package ATPInterface;
import com.google.common.base.Preconditions;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;

/**
 * This is a simple rosjava {@link Publisher} {@link Node}. It assumes an
 * external roscore is already running.
 * 
 * @author ethan.rublee@gmail.com (Ethan Rublee)
 * @author damonkohler@google.com (Damon Kohler)
 */
public class ATPTalker implements NodeMain {

  private Node node;

  @Override
  public void main(Node node) {
    Preconditions.checkState(this.node == null);
    this.node = node;
    try {
      Publisher<org.ros.message.std_msgs.String> publisher =
          node.newPublisher("chatter", "std_msgs/String");
      int seq = 0;
      while (true) {
        org.ros.message.std_msgs.String str = new org.ros.message.std_msgs.String();
        str.data = "BissMillah! " + seq;
        //str.data = "Hello world! " + seq;
        publisher.publish(str);
        node.getLog().info("BissMillah! " + seq);
        //node.getLog().info("Hello, world! " + seq);
        seq++;
        Thread.sleep(1000);
      }
    } catch (Exception e) {
      if (node != null) {
        node.getLog().fatal(e);
      } else {
        e.printStackTrace();
      }
    }
  }

  @Override
  public void shutdown() {
    node.shutdown();
    node = null;
  }
}

