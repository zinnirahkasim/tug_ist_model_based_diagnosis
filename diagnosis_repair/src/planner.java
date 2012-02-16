import java.util.*;
import java.io.*;
import java.util.Properties;
import pddl4j.ErrorManager;
import pddl4j.PDDLObject;
import pddl4j.ErrorManager.Message;
import pddl4j.Parser;

import pddl4j.exp.term.Term;
import pddl4j.exp.AtomicFormula;
import com.google.common.base.Preconditions;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.message.MessageListener;
import org.ros.message.diagnosis_msgs.Diagnosis;
import org.ros.message.diagnosis_msgs.DiagnosisResults;
import org.ros.message.diagnosis_msgs.Observations;

public class planner implements NodeMain{
    private String problem;
		private Node node;
    private ArrayList<String> msg_list = new ArrayList<String>();

    public planner()
    {
     problem = "define (problem prob)(:domain repair_domain)(:requirements :strips)(:objects ";
    }
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
         Preconditions.checkState(this.node == null);
         this.node = node;
         int o_length=-1;

         node.newSubscriber("/Diagnosis", "diagnosis_msgs/Diagnosis",
          new MessageListener<org.ros.message.diagnosis_msgs.Diagnosis>() {
            @Override
            public void onNewMessage(org.ros.message.diagnosis_msgs.Diagnosis diag_msg) {
         try {
            org.ros.message.diagnosis_msgs.DiagnosisResults diag_r =  new org.ros.message.diagnosis_msgs.DiagnosisResults();
            ArrayList<org.ros.message.diagnosis_msgs.DiagnosisResults> diag = new ArrayList<org.ros.message.diagnosis_msgs.DiagnosisResults>();
            diag = diag_msg.diag;
            for(int d=0; d<diag.size(); d++)
            {
            diag_r = diag.get(d);
            String[] good = (String[]) diag_r.good.toArray(new String[0]);
            String[] bad = (String[]) diag_r.bad.toArray(new String[0]);
            
            String co_problem="";
            String goal = "(:goal (and ";
            String init="(:init ";
            for(int i=0;i<good.length;i++)
               {
               co_problem = co_problem + good[i] + " ";
               init = init + "(component "+good[i]+")" + "(nab "+good[i]+")";
               goal=goal+"(nab "+good[i]+")";
               }
            for(int i=0;i<bad.length;i++)
               {
               co_problem = co_problem + bad[i] + " ";
               init = init + "(component "+bad[i]+")" + "(ab "+bad[i]+")";
               goal=goal+"(nab "+bad[i]+")";
               }
            for(int i=0;i<msg_list.size();i++)
               {
                 String topic = msg_list.get(i).substring(msg_list.get(i).indexOf("(")+1,msg_list.get(i).indexOf(")"));
                 co_problem = co_problem + topic + " ";            
                 if(msg_list.get(i).charAt(0)=='o')
                      init = init + "(topic "+topic+")" + "(ok "+topic+")";
                 else
                      init = init + "(topic "+topic+")" + "(not_ok "+topic+")";               
							 }
            co_problem = co_problem + ")";
            init = init + ")";
            goal=goal+"))";
            String prob = "(" + problem + co_problem + init + goal + ")";
            //System.out.println(prob);
            BufferedWriter out=new BufferedWriter(new FileWriter("prob.pddl"));
            out.write(prob);
            out.close();
						String repair_domain = "/home/szaman/my_electric_pkgs/model_based_diagnosis/diagnosis_repair/repair_domain.pddl";
            Properties options = Graphplan.getParserOptions();
                if (!new File(repair_domain).exists()) {
                    System.out.println("domain file " + repair_domain + " does not exist");
                    System.exit(0);
                }
                Parser parser = new Parser(options);
                PDDLObject domain = parser.parse(new File(repair_domain));
                PDDLObject problem = parser.parse(new File("prob.pddl"));
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
                    Graphplan gplan = new Graphplan(pb);
                    gplan.problem = pb;
                    gplan.preprocessing();
                    Plan plan = gplan.solve();
                    if (plan != Plan.FAILURE) {
                        System.out.println("ACTIONS for DIAGNOSIS :"+(d+1));
                        Graphplan.printPlan(plan);
												/*for (Set<AtomicFormula> layer : plan)
                             for (AtomicFormula action : layer)
											         { 
                                System.out.print(action.getPredicate().toUpperCase());
								                for (Term parameter : action) 
                                  {
                                     System.out.print(" " + parameter.getImage().toUpperCase());
                                  }
															  System.out.println();
                                }*/
                    } else {
                        System.out.println("\nno solution plan found\n");
                    }
                        System.out.printf("\n Nos of ACTIONS : %12d \n", gplan.a_tried);
                    gplan = null;
                    plan = null;

               }
               
            }// for d
        } catch (Throwable t) {
            System.err.println(t.getMessage());
            t.printStackTrace(System.err);
        }
         } //block
       });

node.newSubscriber("/Diagnostic_Observation", "diagnosis_msgs/Observations",
          new MessageListener<org.ros.message.diagnosis_msgs.Observations>() {
            @Override
            public void onNewMessage(org.ros.message.diagnosis_msgs.Observations msg) {
              //if(!processObs)
              //{
							String[] obs_msg = (String[]) msg.obs.toArray(new String[0]);
               for(int m=0; m<obs_msg.length; m++)
                 { String s = obs_msg[m];
                  boolean found = false;
                  /*if(s.charAt(0)=='~')
									  s = neg_prefix + s.substring(1);
       						*/for(String st : msg_list)
                   if(s.equals(st))
             					{
               					found = true;
               					break;
              				}
                    
      						if(!found)
         						{ String sub_str="";
           						String str = s;
          						int p;
          						p = str.indexOf('(') - 1;
          						while(str.charAt(p)!='_')
            						sub_str = sub_str + str.charAt(++p);
          						String ostr = "ok" + sub_str + "Frequency)" ;
          						//String nstr = neg_prefix + "ok" + sub_str + "Frequency)";
                      String nstr = "~ok" + sub_str + "Frequency)";
          						for(int i=0;i<msg_list.size();i++)
            							if(ostr.equals(msg_list.get(i)) || nstr.equals(msg_list.get(i)) )
                						{
                              msg_list.remove(i);
                 							continue;
                						}
												msg_list.add(str);
												                 
         							} // if(!found)
                     
                } // for int m
             /*for(int i=0;i<msg_list.size();i++)
                 {
                   System.out.println(msg_list.get(i).toString());
                 }*/
            //} // if(!processObs)
           } //onMessage   
          });

    }// main

}// class
