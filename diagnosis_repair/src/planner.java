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
import org.ros.actionlib.client.ActionClient;
import org.ros.message.diagnosis_msgs.DiagnosisRepairGoal;

import org.ros.actionlib.client.SimpleActionClient;
import org.ros.actionlib.client.SimpleActionClientCallbacks;
import org.ros.actionlib.state.SimpleClientGoalState;
import org.ros.exception.RosException;

import org.ros.message.diagnosis_msgs.DiagnosisRepairActionFeedback;
import org.ros.message.diagnosis_msgs.DiagnosisRepairActionGoal;
import org.ros.message.diagnosis_msgs.DiagnosisRepairActionResult;
import org.ros.message.diagnosis_msgs.DiagnosisRepairFeedback;
import org.ros.message.diagnosis_msgs.DiagnosisRepairGoal;
import org.ros.message.diagnosis_msgs.DiagnosisRepairResult;

import org.ros.node.DefaultNodeRunner;
import java.util.concurrent.TimeUnit;

import pddl4j.exp.action.Action;
import pddl4j.exp.action.ActionDef;
import pddl4j.exp.action.ActionID;


import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.Set;


public class planner implements NodeMain{
    private String problem;
		private Node node;
    private ArrayList<String> msg_list = new ArrayList<String>();
    Map<String,Object> mp;

    public planner()
    {
     problem = "define (problem prob)(:domain test_repair_domain)(:requirements :strips :typing :negative-preconditions)(:objects ";
     mp = new HashMap<String,Object>();
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
				 
         try{
         
         DiagnosisRepairActionNodeSpec spec = new DiagnosisRepairActionNodeSpec();
         

       mp.put("start_node",spec.buildSimpleActionClient("start_node"));
       mp.put("stop_node",spec.buildSimpleActionClient("stop_node"));
       mp.put("shutdown",spec.buildSimpleActionClient("shutdown"));
       mp.put("power_up",spec.buildSimpleActionClient("power_up"));
 
        Set s=mp.entrySet();

        Iterator it=s.iterator();

        while(it.hasNext())
        {
            Map.Entry m =(Map.Entry)it.next();
            DiagnosisRepairActionNodeSimpleClient obj = (DiagnosisRepairActionNodeSimpleClient)m.getValue();
            obj.addClientPubSub(node);
            
        }

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
            
            for(int i=0;i<msg_list.size();i++)
               {
                 String parameter = msg_list.get(i).substring(msg_list.get(i).indexOf("(")+1,msg_list.get(i).indexOf(")"));
                 co_problem = co_problem + parameter + " ";            
                 if(msg_list.get(i).charAt(0)=='~')
                    {
                      String predicate = msg_list.get(i).substring(1,msg_list.get(i).indexOf("("));
                      init = init + "(component "+parameter+")" + "(not_"+predicate+" "+parameter+")";
                    }
                 else
                    {
                      String predicate = msg_list.get(i).substring(0,msg_list.get(i).indexOf("("));
                      init = init + "(component "+parameter+")" + "("+predicate+" "+parameter+")";               
                    }               
							 }
            for(int i=0;i<good.length;i++)
               {
               //co_problem = co_problem + good[i] + " ";
               //init = init + "(component "+good[i]+")" + "(nab "+good[i]+")";
							 init = init + "(nab "+good[i]+")";
               goal=goal+"(nab "+good[i]+")";
               }
            for(int i=0;i<bad.length;i++)
               {
               //co_problem = co_problem + bad[i] + " ";
               //init = init + "(component "+bad[i]+")" + "(ab "+bad[i]+")";
							 init = init + "(ab "+bad[i]+")";
               goal=goal+"(nab "+bad[i]+")";
               }
            co_problem = co_problem + ")";
            init = init + ")";
            goal=goal+"))";
            String prob = "(" + problem + co_problem + init + goal + ")";
            System.out.println(prob);
            BufferedWriter out=new BufferedWriter(new FileWriter("prob.pddl"));
            out.write(prob);
            out.close();

					String repair_domain = "/home/szaman/my_electric_pkgs/model_based_diagnosis/diagnosis_repair/test_repair_domain.pddl";
  	      Properties options = Graphplan.getParserOptions();
          
					if (!new File(repair_domain).exists()) {
            System.out.println("domain file " + repair_domain + " does not exist");
            System.exit(0);
          }

					Parser parser = new Parser(options);

          PDDLObject domain = parser.parse(new File(repair_domain));
    			PDDLObject problem = parser.parse(new File("prob.pddl"));

  //PDDLObject problem = parser.parse(new File("/home/szaman/my_electric_pkgs/model_based_diagnosis/diagnosis_repair/test_repair_problem.pddl"));
         	/*Iterator<ActionDef> i = domain.actionsIterator();
       				while (i.hasNext()) {
          				ActionDef a = i.next();
           				Action ac = (Action) a;
           				System.out.print("="+ac.getName());
         					mp.put(ac.getName(),spec.buildSimpleActionClient(ac.getName()));
              
            }*/
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
                    
			
                        org.ros.message.diagnosis_msgs.DiagnosisRepairGoal repairGoal =  new org.ros.message.diagnosis_msgs.DiagnosisRepairGoal();
                        ArrayList<String> params = new ArrayList<String>();
                        String actionServer=null;
												for (Set<AtomicFormula> layer : plan)
                             for (AtomicFormula action : layer)
											         { 
                                actionServer = action.getPredicate().toUpperCase();
                                for (Term parameter : action) 
                                  {  
                                     params.add(parameter.getImage());
                                     
                                  }
                                 actionServer = action.getPredicate();
                                 repairGoal.parameter = params;
															   System.out.print("Call Action Server: "+actionServer+" for parameters:");

                                 for(int j=0; j < params.size(); ++j)
                 									{
                  									System.out.print(params.get(j).toString()+",");
                 									}
                                  System.out.println();
                                  if(mp.containsKey(actionServer)) 
                                  {
                                   DiagnosisRepairActionNodeSimpleClient sac = (DiagnosisRepairActionNodeSimpleClient)mp.get(actionServer);
      
                                   repairGoal.parameter = params;
        
                                   sac.sendGoal(repairGoal, new SimpleActionClientCallbacks<DiagnosisRepairFeedback, DiagnosisRepairResult>() {
                                   @Override
                                   public void feedbackCallback(DiagnosisRepairFeedback feedback) {
                                    System.out.print("Client feedback\n\t");
                                    System.out.println();
        													 }

        												  @Override
        												  public void doneCallback(SimpleClientGoalState state, DiagnosisRepairResult result) {
                    											System.out.println("Client done " + state);
                                  }
            
        		        						  @Override
        						        		  public void activeCallback() {
          								          System.out.println("Client active");
        								          }
                               });
                              } // if mp.contians()
                                 else
                                   System.out.print("No action serever ["+actionServer+"] exists.");

      												// wait for the action to return
      												/*System.out.println("[Test] Waiting for result.");
      												boolean finished_before_timeout = sac.waitForResult(100, TimeUnit.SECONDS);

      												if (finished_before_timeout) {
        											SimpleClientGoalState state = sac.getState();
        											System.out.println("[Test] Action finished: " + state.toString());

                              DiagnosisRepairResult res = sac.getResult();	
        
                              System.out.println("Result Obtained Back. ");
        						          System.out.println();
      								        } else {
        									      System.out.println("[Test] Action did not finish before the time out");
      								          }*/
                               //params.clear();
                            } // for action
                          // }// d>0
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
              String[] obs_msg = (String[]) msg.obs.toArray(new String[0]);
               for(int m=0; m<obs_msg.length; m++)
                 { 
                   boolean found = false;
                   String s = obs_msg[m];
                   
                   if(!msg_list.contains(s))
         						{
                      String ns = "@";  // Negating or oposite strings
                   		if(s.charAt(0)=='~')
                       		ns = s.substring(1);
                   		else
                       		ns = "~" + s;
											int k = msg_list.indexOf(ns);
            							if(k!=-1)
                						{
                              msg_list.set(k,s);
                              found = true;
                 							break;
                						}
                      
                     if(!found)
											 msg_list.add(s);
                     }
                } // for int m
             //System.out.println("END");
           } //onMessage   
          });

 
  } catch (RosException e) {
        e.printStackTrace();
      } 
 }// main

}// class
