import java.io.File;
import java.util.Properties;
import pddl4j.ErrorManager;
import pddl4j.PDDLObject;
import pddl4j.ErrorManager.Message;
import pddl4j.Parser;
import com.google.common.base.Preconditions;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import org.ros.message.diagnosis_msgs.Diagnosis;

public class planner1 implements NodeMain{
		private Node node;
    @Override
    public void onShutdown(Node node) {
     node.shutdown();
     node = null;
    }
    @Override
    public void onStart(Node node){ 
    try{
        main_planner(node);
        }catch(Exception e){}
    }
    public void main_planner(Node node) {
        try {
            Preconditions.checkState(this.node == null);
            this.node = node;
						String[] args= new String[2];
            args[0] = "/home/szaman/my_electric_pkgs/model_based_diagnosis/diagnosis_repair/repair_domain.pddl";
            args[1] = "/home/szaman/my_electric_pkgs/model_based_diagnosis/diagnosis_repair/repair_problem.pddl";
            if (args.length != 2) {
                Graphplan.printUsage();
            } else {
                Properties options = Graphplan.getParserOptions();
                if (!new File(args[0]).exists()) {
                    System.out.println("domain file " + args[0] + " does not exist");
                    System.exit(0);
                }
                if (!new File(args[1]).exists()) {
                    System.out.println("problem file " + args[0] + " does not exist");
                    System.exit(0);
                }
                Parser parser = new Parser(options);
                PDDLObject domain = parser.parse(new File(args[0]));
                PDDLObject problem = parser.parse(new File(args[1]));
                PDDLObject pb = null;
                if (domain != null && problem != null) {
                    pb = parser.link(domain, problem);
                }
                ErrorManager mgr = parser.getErrorManager();
                if (mgr.contains(Message.ERROR)) {
                    mgr.print(Message.ALL);
                }
                else {
                    mgr.print(Message.WARNING);
                    //System.out.println("\nParsing domain \"" + domain.getDomainName() + "\" done successfully ...");
                    //System.out.println("Parsing problem \"" + problem.getProblemName() + "\" done successfully ...\n");
                    Graphplan gplan = new Graphplan(pb);
                    gplan.problem = pb;
                    gplan.preprocessing();
                    Plan plan = gplan.solve();
                    if (plan != Plan.FAILURE) {
                        //System.out.println("\nfound plan as follows:\n");
                        Graphplan.printPlan(plan);
                    } else {
                        System.out.println("\nno solution plan found\n");
                    }
                        System.out.printf("\n Nos of ACTIONS : %12d \n", gplan.a_tried);
                        //System.out.printf("\nnumber of actions tried : %12d \n", gplan.a_tried);
                        //System.out.printf("number of noops tried   : %12d \n\n", gplan.noop_tried);
               }
            }
        } catch (Throwable t) {
            System.err.println(t.getMessage());
            t.printStackTrace(System.err);
        }
    }

}
