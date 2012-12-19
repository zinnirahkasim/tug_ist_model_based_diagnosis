/*
* Copyright (c).2012. OWNER: Institute for Software Technology TU-Graz Austria.
* Authors: Safdar Zaman, Gerald Steinbauer. (szaman@ist.tugraz.at, steinbauer@ist.tugraz.at)
* All rights reserved.
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* 1. Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation 
* and/or other materials provided with the distribution.
* 3. Neither the name of the <ORGANIZATION> nor the names of its contributors
* may be used to endorse or promote products derived from this software without
* specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSE-
* QUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
* GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
* OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
* DAMAGE.
*/

import java.util.*;
import java.io.*;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.ReentrantLock;

import java.util.Properties;
import pddl4j.ErrorManager;
import pddl4j.PDDLObject;
import pddl4j.ErrorManager.Message;
import pddl4j.Parser;
import pddl4j.exp.term.Term;
import pddl4j.exp.AtomicFormula;

import org.apache.commons.logging.Log;
import com.google.common.base.Preconditions;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.NodeConfiguration;
import org.ros.node.parameter.ParameterTree;
import org.ros.namespace.GraphName;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.internal.loader.CommandLineLoader;
import org.ros.namespace.GraphName;


import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.message.MessageListener;
/*
import org.ros.message.MessageListener;
import org.ros.message.diagnosis_msgs.Diagnosis;
import org.ros.message.diagnosis_msgs.DiagnosisResults;
import org.ros.message.diagnosis_msgs.Observations;

import org.ros.message.diagnosis_msgs.DiagnosisRepairGoal;

import org.ros.actionlib.client.SimpleActionClient;
import org.ros.actionlib.ActionSpec;
import org.ros.actionlib.client.SimpleActionClientCallbacks;
import org.ros.actionlib.state.SimpleClientGoalState;


import org.ros.message.diagnosis_msgs.DiagnosisRepairAction;
import org.ros.message.diagnosis_msgs.DiagnosisRepairActionFeedback;
import org.ros.message.diagnosis_msgs.DiagnosisRepairActionGoal;
import org.ros.message.diagnosis_msgs.DiagnosisRepairActionResult;
import org.ros.message.diagnosis_msgs.DiagnosisRepairFeedback;
import org.ros.message.diagnosis_msgs.DiagnosisRepairGoal;
import org.ros.message.diagnosis_msgs.DiagnosisRepairResult;


import org.ros.actionlib.client.ActionClient;
*/
import org.ros.exception.RosException;
//import org.ros.node.DefaultNodeRunner;
import java.util.concurrent.TimeUnit;
import org.ros.time.WallTimeProvider;

import pddl4j.exp.action.Action;
import pddl4j.exp.action.ActionDef;
import pddl4j.exp.action.ActionID;


import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.Set;
import java.util.List;

/**
 * This class implements the  planner and Repair Actions for the Model based Diagnosis.
 * 
 * @version 1.0
 */
public class plannerRepair implements NodeMain{
    
    private String[] excluded_nodes;
    
    /**
     * The parser reference.
     */
    private Parser _parser;

    /**
     * The options properties for the parser.
     */
    private Properties _options;

    /**
     * The domain reference for PDDL.
     */
    private PDDLObject _domain;

    /**
     * The current ros node handler.
     */
		private ConnectedNode _node;

    /**
     * The list of the observations.
     */
    private ArrayList<String> obs_list = new ArrayList<String>();
    
    /**
     * The MAP mapping the name of action with action client Object.
     */
    private Map<String,SimpleActionClient> mp;

    /**
     * The Simple Cloient Object for the action client.
     */
    private DiagnosisRepairActionNodeSimpleClient sac;

    /**
     * The Specification reference for the Repair Action.
     */
    private DiagnosisRepairActionNodeSpec spec;

    /**
     * The Current Time Provider.
     */
    private WallTimeProvider t;

    /**
     * The log reference for logging.
     */
		private Log log;

    /**
     * The PDDL domain file reference.
     */
    private File domain_file;
		private String domain_file_name;

    /**
     * The cuncurrency control object.
     */
    private Object threadSync = new Object();

    /**
     * The lock reference for Locking.
     */
    protected ReentrantLock lock = new ReentrantLock(true);

    /**
     * The condition reference for locking.
     */
    protected Condition c = lock.newCondition(); 

    /**
     * The execution control variables.
     */
    private static boolean executing_plan, goal_processed;

    /**
     * new plannerRepair creation to get the plan and repair actions.
     */
    public plannerRepair(){
     executing_plan = false;
     goal_processed = false;
     mp = new HashMap<String,SimpleActionClient>();
     t = new WallTimeProvider();
     try{
         spec = new DiagnosisRepairActionNodeSpec();
     }catch(RosException e){}
    }
      
    /**
     * Overridden method onShutdown of rosjava node.
		 * called when node is killed.
     * 
     * @param node the ros node handler.
     */

    @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("diagnosis_engine/diagnosis_engine");
  }

  @Override
  public void onShutdownComplete(Node node) {
  }

  @Override
  public void onError(Node node, Throwable throwable) {
  }

    @Override
    public void onShutdown(Node node) {
     node.shutdown();
     node = null;
    }

    /**
     * Overridden method onStart of rosjava node.
     * 
     * @param node the ros node handler.
     */
    @Override
    public void onStart(ConnectedNode node){
	 ParameterTree prm = node.newParameterTree();
	 //NodeConfiguration nodeConfiguration = node.nodeConfiguration.getNodeName();
	 domain_file_name = prm.getString("file","repair_domain.pddl");
	 String param_str = prm.getString("excluded_nodes","node");
	 excluded_nodes = param_str.split(",");
	 //System.out.println("param1="+excluded_nodes+"\n"+domain_file_name);
         //System.out.println(node.getName());
         _node = node;
         main_runner();

        
    }

    /**
     * main running method.
     * It contains subscribers and all neccessary main controlling code
		 *
     * @param node the ros node handler.
     */
    public void main_runner() {
	log = _node.getLog();
        _options = Graphplan.getParserOptions();
	_parser = new Parser(_options);
      try{
            domain_file = new File(domain_file_name);
            if (!domain_file.exists()) {
            					System.out.println("domain file does not exist");
            					System.exit(0);
            }
    		   _domain = _parser.parse(domain_file);
					 
      }catch(FileNotFoundException e){
           			  System.err.println(e.getMessage());
           				e.printStackTrace(System.err);
					 		}
				    				
      	connectActionServers();

	      /**
  	    * Subscriber for the /diagnosis topic.
  	    */
          
        Subscriber<diagnosis_msgs.Diagnosis> subscriber = _node.newSubscriber("/diagnosis", diagnosis_msgs.Diagnosis._TYPE);
        subscriber.addMessageListener(new MessageListener<diagnosis_msgs.Diagnosis>(){
              @Override
              public void onNewMessage(diagnosis_msgs.Diagnosis diag_msg) {

		              diagnosis_msgs.DiagnosisResults diag_r; // =  new org.ros.message.diagnosis_msgs.DiagnosisResults();
		              List<diagnosis_msgs.DiagnosisResults> diag;// = new ArrayList<org.ros.message.diagnosis_msgs.DiagnosisResults>();
    		          diag = diag_msg.getDiag();
    		          diag_r = diag.get(0);
			List<String> good = diag_r.getGood();
                        List<String> bad =  diag_r.getBad();
			for(int i=0;i<excluded_nodes.length;i++)
                  	if(bad.contains(excluded_nodes[i]))
                        {
                         bad.remove(excluded_nodes[i]);
                         System.out.println(excluded_nodes[i]+" excluded.");}
                 									
									if(bad.size()>0 && !executing_plan){
										 executing_plan = true;
										 make_probPDDL_file(good,bad);
                   	 get_plan();
                  }
             }
   			}); //newSubscriber

	      /**
  	    * Subscriber for the /observations topic.
  	    */
         Subscriber<diagnosis_msgs.Observations> sub_Obs = _node.newSubscriber("/observations", diagnosis_msgs.Observations._TYPE);
          sub_Obs.addMessageListener(new MessageListener<diagnosis_msgs.Observations>(){
            @Override
            public void onNewMessage(diagnosis_msgs.Observations msg) {
              try{
              List<String> obs_msg = msg.getObs();
		synchronized (threadSync){
                Iterator it = obs_msg.iterator();
		while(it.hasNext()) {
                //for(int m=0; m<obs_msg.length; m++) {
                   String s = (String) it.next();
									 if(!obs_list.contains(s)) {
                      String ns = "@";  
                   		if(s.charAt(0)=='~')
                       		ns = s.substring(1);
                   		else
                       		ns = "~" + s;
											int k = obs_list.indexOf(ns);
            					if(k!=-1)
                				 obs_list.set(k,s);
                      else
												 obs_list.add(s);
                   } //if
                 } // while
							}// synchronized
					}catch(Exception e){
              System.out.println("Error while updating the observation list.");
           }	
  			} // On New Mssage
     		}); // Subscriber
        
    }//main_runner



    /**
     * Extracts the repair actions from the PDDL domain.
     * Makes action client with each action for action server.
     * in the planning graph because it is easier to find a solution for it.
     */
     public void connectActionServers(){
			 try{
            Iterator<ActionDef> i = _domain.actionsIterator();
        		while (i.hasNext()) {
            		ActionDef a = i.next();
            		Action op = (Action) a;
								//System.out.println(op.getName().toString());
                sac = spec.buildSimpleActionClient(op.getName().toString());
								sac.addClientPubSub(_node);
								//mp.put(op.getName().toString(),sac);
        		}
            System.out.println("Planner is waiting for Action Server to start......!");
      			sac.waitForServer();
      			System.out.println("Connected with Server and Planner is Up....");
       }catch(Exception e){}
     }

    /**
     * Creates PDDL Problem file.
		 *
     * @param list good (NAB) diagnosis and list of bad (AB) diagnosis.
     */
     public void make_probPDDL_file(List<String> good, List<String> bad) {
					 synchronized (threadSync){
							 ArrayList<String> list = new ArrayList<String>();
							 lock.lock();
							 c.signalAll();
							 list = obs_list;
               lock.unlock();
							 String prob_text = "define (problem repair_goal)(:domain repair_domain)(:requirements :strips :equality :typing :negative-preconditions)(:objects ";
							 String co_problem="";
						   String goal = "(:goal (and ";
						   String init="(:init ";
							 for(int i=0;i<list.size();i++) {
                  		String parameter = list.get(i).substring(list.get(i).indexOf("(")+1,list.get(i).indexOf(")"));
											parameter = parameter.replace(",","_");
                  		co_problem = co_problem + parameter + " ";
                  		if(list.get(i).charAt(0)=='~') {
                      		String predicate = list.get(i).substring(1,list.get(i).indexOf("("));
                          //if(!predicate.equals("ok"))
													  init = init + "(not_"+predicate+" "+parameter+")";
                  		} else {
                      		String predicate = list.get(i).substring(0,list.get(i).indexOf("("));
                          //if(!predicate.equals("ok"))
                            init = init + "("+predicate+" "+parameter+")";               
                    		}               
							 }
							 Iterator good_it=good.iterator();
							 while(good_it.hasNext()){
											String good_str=(String)good_it.next();
               				init = init + "(good "+good_str+")";
               				goal=goal+"(good "+good_str+")";
                      //co_problem = co_problem + good[i] + " ";
               }
							 Iterator bad_it=bad.iterator();
            	 while(bad_it.hasNext()){
                      String bad_str=(String)bad_it.next();
               				init = init + "(bad "+bad_str+")";
                  		goal=goal+"(good "+bad_str+")";
                      //co_problem = co_problem + bad[i] + " ";
               }
               co_problem = co_problem + ")";
               init = init + ")";
               goal=goal+"))";
               String prob = "(" + prob_text + co_problem + init + goal + ")";
							 System.out.println(prob);
						   try{
               		     BufferedWriter out=new BufferedWriter(new FileWriter("repair_goal.pddl"));
               		     out.write(prob);
               		     out.close();
               }catch(IOException e){}
				 }//synchronized
     } //make_probPDDL_file


     /**
     * Gets plan of repair actions from the planner.
		 *
     */
     public void get_plan(){
           try{  
									log.info("Planner called.");
									PDDLObject problem = _parser.parse(new File("repair_goal.pddl"));
									PDDLObject pb = null;
    		      		if (_domain != null && problem != null){
                    pb = _parser.link(_domain, problem);
              		}
              		ErrorManager mgr = _parser.getErrorManager();
              		if (mgr.contains(Message.ERROR))
                    mgr.print(Message.ALL);
              		else {
                      		mgr.print(Message.WARNING);
                      		Graphplan gplan = new Graphplan(pb);
                      		gplan.preprocessing();
													Plan plan = gplan.solve();
												  if (plan != Plan.FAILURE) {
                          		String actionServer=null;
															String param = null;
															for (Set<AtomicFormula> layer : plan) {
																	for (AtomicFormula action : layer) {
																			actionServer = action.getPredicate();
																			ArrayList<String> params = new ArrayList<String>();
																			for (Term parameter : action) 
                                     				params.add(parameter.getImage());
																			goal_processed = false;
                                      while(!goal_processed){
                                          log.info("Repair called: "+actionServer+"("+params.get(0).toString()+")");
                                			    execute_plan(actionServer, params);
																			}
																	}
															}
															
															log.info("Planner finished.");
															Thread.currentThread().sleep(3000);
															plan.actions.clear();
															executing_plan = false;
															plan = null;
                     			}else
                            		log.info("\nno solution plan found\n");
										}//else
            }catch(Throwable t){
									System.err.println(t.getMessage());
           				t.printStackTrace(System.err);
           			  executing_plan = false;
					 		}
   } //get_plan


    /**
    * Repair actions take place here.
		* Executes the plan of actions for repair.
		*
    * @param name of Action Server and list of its parameters if any.
    */
   	public void execute_plan(String actionServer, ArrayList<String> params){
       try{ 
            sac = spec.buildSimpleActionClient(actionServer);
						sac.addClientPubSub(_node);
   		diagnosis_msgs.DiagnosisRepairGoal repairGoal; //= spec.createGoalMessage();
	        repairGoal.setParameter(params);
            
                sac.sendGoal(repairGoal);
	        SimpleClientGoalState state = sac.getState();
          
	          boolean finished_before_timeout = sac.waitForResult(10, TimeUnit.SECONDS);
						if (finished_before_timeout) {
              diagnosis_msgs.DiagnosisRepairResult res = (diagnosis_msgs.DiagnosisRepairResult) sac.getResult();
              state = sac.getState();
              log.info("Repair finished with state "+state.toString()+".");
              goal_processed = true;
						}else{
                  log.info("Repair Action did not finish before the time out");	
									goal_processed = false;		
				        }
            
        }catch(Throwable t){
           			  System.err.println(t.getMessage());
           				t.printStackTrace(System.err);
									goal_processed = true;
         }
  }// execute_plan

}//class
