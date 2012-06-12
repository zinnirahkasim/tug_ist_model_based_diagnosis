/*
* Copyright (c).
* May-2012. Institute from Software Technology, TU Graz, Austria,
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
import java.text.*;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.ReentrantLock;

import com.google.common.base.Preconditions;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import java.lang.System;
import org.ros.message.diagnosis_msgs.Diagnosis;
import org.ros.message.diagnosis_msgs.Observations;
import org.ros.message.diagnostic_msgs.DiagnosticArray;

import org.ros.actionlib.client.SimpleActionClient;
import org.ros.actionlib.ActionSpec;
import org.ros.message.diagnosis_msgs.SystemModelAction;
import org.ros.message.diagnosis_msgs.SystemModelActionFeedback;
import org.ros.message.diagnosis_msgs.SystemModelActionGoal;
import org.ros.message.diagnosis_msgs.SystemModelActionResult;
import org.ros.message.diagnosis_msgs.SystemModelFeedback;
import org.ros.message.diagnosis_msgs.SystemModelGoal;
import org.ros.message.diagnosis_msgs.SystemModelResult;
import java.util.concurrent.TimeUnit;
import org.ros.exception.RosException;


import hittingsetalg.*;
import theoremprover.*;
import utils.*;
import dfengine.*;

/**
 * This is a diagnosic_engine class of the Model Based Diagnostics. It assumes an
 * external roscore is already running.
 * @authors Safdar Zaman, Gerald Steinbauer. (szaman@ist.tugraz.at, steinbauer@ist.tugraz.at)
 */
class IllegalUserInput extends Exception {

    public IllegalUserInput(String msg) {
        super(msg);
    }

}

public class diagnosis_engine extends Thread implements NodeMain {
 
  private String PROP;
  private String SD;
  private String OBS;
  private String AB;
  private String NAB;
  private String neg_prefix;
  private Node node;
  private ArrayList<String> msg_list = new ArrayList<String>();
  private boolean processObs;
  private boolean threadRunning;
 	private Publisher<org.ros.message.diagnosis_msgs.Diagnosis> publisher;
  private Publisher<org.ros.message.diagnostic_msgs.DiagnosticArray> d_pub;
  private Calendar now = Calendar.getInstance();
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



	public diagnosis_engine()
  {
       super("FetchResult"+1);
       AB = "AB";
       NAB = "NAB";
       neg_prefix = "not_";
			 processObs = false;
			 threadRunning =  false;
       
  }



  @Override
  public void onStart(Node node){ 
    try{
        main_engine(node);
    }catch(Exception e){}
  }

  public void main_engine(Node node) throws ParseError, IllegalUserInput{
      try {
      this.node = node;
      String path=null;
      final Log log = node.getLog();
      System.out.println("Diagnosis Engine is up.......");
      get_sys_model(node);
      publisher = node.newPublisher("/diagnosis", "diagnosis_msgs/Diagnosis");
      d_pub = node.newPublisher("/diagnostics", "diagnostic_msgs/DiagnosticArray"); 

      /**
      * Subscriber for the /observations topic.
      */
			node.newSubscriber("/observations", "diagnosis_msgs/Observations",
          new MessageListener<org.ros.message.diagnosis_msgs.Observations>(){
            @Override
            public void onNewMessage(org.ros.message.diagnosis_msgs.Observations msg) {
              String[] obs_msg = (String[]) msg.obs.toArray(new String[0]);
              synchronized (threadSync){  
               	for(int m=0; m<obs_msg.length; m++){
									 boolean found = false;
                   String s = obs_msg[m];
                   String ns = "@";
                   if(s.charAt(0)=='~'){ 
                       ns = s.substring(1);
                       s = neg_prefix + s.substring(1);
                   }else
                       ns = neg_prefix + s;
                  if(!msg_list.contains(s)){
                       int k = msg_list.indexOf(ns);
            					 if(k!=-1)
                              msg_list.set(k,s);
                       else 
                              msg_list.add(s);
                  }
              }// for loop
            }// synchronized             

           } //OnNewMessage   
           } //MessageListener
           );//newSubscriber

   
     }catch (Exception e) {
          if (node != null) {
              System.out.println(e);
              node.getLog().fatal(e);
          }else 
              e.printStackTrace();
      } // catch

  } //main

  public void get_sys_model(Node node){
        try
           {
     				ActionSpec<SystemModelAction, SystemModelActionFeedback, SystemModelActionGoal, SystemModelActionResult, SystemModelFeedback, 	SystemModelGoal, SystemModelResult> spec = new ActionSpec(SystemModelAction.class, "diagnosis_msgs/SystemModelAction", "diagnosis_msgs/SystemModelActionFeedback", "diagnosis_msgs/SystemModelActionGoal","diagnosis_msgs/SystemModelActionResult", "diagnosis_msgs/SystemModelFeedback",
        "diagnosis_msgs/SystemModelGoal", "diagnosis_msgs/SystemModelResult");


     SimpleActionClient<SystemModelActionFeedback, SystemModelActionGoal, SystemModelActionResult, SystemModelFeedback, SystemModelGoal, SystemModelResult> sac = new SimpleActionClient("diagnosis_model_server", spec);

            
           System.out.println("[Test] Waiting for system model action server to start!");
           sac.addClientPubSub(node);
           sac.waitForServer();
           System.out.println("System Model Server started!!!!");
           SystemModelGoal repairGoal= spec.createGoalMessage();
           repairGoal.goal = 1;
					 sac.sendGoal(repairGoal);
          boolean finished_before_timeout = sac.waitForResult(10, TimeUnit.SECONDS);
					if (finished_before_timeout) {
              SystemModelResult res = (org.ros.message.diagnosis_msgs.SystemModelResult) sac.getResult();
							String[] rules = (String[]) res.rules.toArray(new String[0]);
              String[] props = (String[]) res.props.toArray(new String[0]);
              String s="";
              for(int m=0; m<rules.length-1; m++){
                   s = s + rules[m] + ".\r\n";
              }
              s = s + rules[rules.length-1] + ".\r\n\r\n\r\n";
              SD = s;
              s="";
              for(int m=0; m<props.length-1; m++){
                  s = s + props[m] + "\r\n";
              }
							s = s + props[props.length-1] + "\r\n\r\n\r\n";
              PROP = s;
              AB = res.ab;
              NAB = res.nab;
              neg_prefix = res.neg_prefix;
				
              System.out.println("Result is back in time is.ab="+res.ab+",nab="+res.nab+",neg="+res.neg_prefix+"rules:"+rules[0]);
              start();
					}else{
                System.out.println("[Test] Action did not finish before the time out");				
			     }
       }catch (RosException e){
                  System.err.println(e.getMessage());
                  e.printStackTrace(System.err);;
        }catch(Throwable t){
           			  System.err.println(t.getMessage());
           				t.printStackTrace(System.err);
         }
  } // get_sys_model
	void find_diagnosis(){   
    final Log log = node.getLog();
    try{
        ArrayList<String> list = new ArrayList<String>();
        lock.lock();
				c.signalAll();
				list = msg_list;
        lock.unlock();
    		OBS="";
    		for(int j=0; j <  list.size(); ++j)
        		OBS = OBS +  list.get(j).toString() + ".";
       
    
    		LSentence sd = parseSD();
    		LSentence obs = parseOBS();
    		LSentence indepModel = new LSentence();
    		indepModel.addRules(sd);
    		indepModel.addRules(obs);

    		ArrayList result;
    
    		int id = 0;
				ABTheoremProver thrmp = new ABTheoremProver();
    		thrmp = indepModel.asABPropositionalSentence(thrmp);
    		Iterator it = thrmp.getAssumptions().iterator();
    		ArrayList<String> components = new ArrayList<String>();     
    		while(it.hasNext()){
            Assumption a = (Assumption)it.next();
            String ass = (String)a.identifier;
            boolean ass_ab = true;
            String compName = null;
            if (ass.matches(AB + "\\(" + "[a-zA-Z_0-9]+" + "\\)")) {
                compName = ass.substring(AB.length() + 1, ass.length() - 1);
                ass_ab = true;
            } else if (ass.matches(NAB + "\\(" + "[a-zA-Z_0-9]+" + "\\)")) {
                      compName = ass.substring(NAB.length() + 1,
                                               ass.length() - 1);
                			ass_ab = false;
             				} //else if
             boolean present = false;
             for (int i = 0; i < components.size(); ++i) {    
                if(compName.equals(components.get(i).toString()))              
                    present = true;
             }
             if(!present)
               components.add(compName);
         }//while
      final Diagnosis dmsg = node.getMessageFactory().newMessage("diagnosis_msgs/Diagnosis");

      ArrayList<org.ros.message.diagnosis_msgs.DiagnosisResults>	diagArr = new                    				     				 ArrayList<org.ros.message.diagnosis_msgs.DiagnosisResults>();

        boolean consistent = checkConsistency(indepModel);
        String gd="",bd="";
        if (consistent) {
            org.ros.message.diagnosis_msgs.DiagnosisResults diag_result_c =  new org.ros.message.diagnosis_msgs.DiagnosisResults();
						ArrayList<String> good = new ArrayList<String>();  
            ArrayList<String> bad = new ArrayList<String>();
            log.info("Consistent.");
            for(int j=0; j < components.size(); ++j) {
                  gd= gd+"'"+components.get(j).toString()+"'";
                  good.add(components.get(j).toString());
            }
            diag_result_c.good = good;
         		diag_result_c.bad  = bad;
         		dmsg.o_time =  now.getTimeInMillis();
         		diagArr.add(diag_result_c);
         		dmsg.diag = diagArr;
            publisher.publish(dmsg);

        } else {
							 log.info("Start Diagnosis.");
               ArrayList diagnoses = new ArrayList();
			         ArrayList conflictSets = new ArrayList();  
			         MinHittingSetsFM  hsFM=null;
			         try {
		               hsFM = new MinHittingSetsFM(false, thrmp, AB, NAB);
		           }catch (Exception e) {
	               System.out.println("Illigal assumptions");
			         }
    		      int computationResult = hsFM.compute(10, 100);
			        boolean hasMoreDiags = (computationResult != MinHittingSets.CS_ALL_MIN_DIAGS_COMPUTED);
			    		ArrayList minHittingSetsAsAss = hsFM.getMinHS();
			    		ArrayList conflictsAsAss = hsFM.getConflictsAsAss();
        
        			for (int i = 0; i < minHittingSetsAsAss.size(); ++i) { 
							 		org.ros.message.diagnosis_msgs.DiagnosisResults diag_result_ic =  new org.ros.message.diagnosis_msgs.DiagnosisResults();           
               		ArrayList<String> good1 = new ArrayList<String>();  
        			 		ArrayList<String> bad1 = new ArrayList<String>();
							 		gd="";bd="";
							 		String res = minHittingSetsAsAss.get(i).toString();
									log.info(res);
               		for(int j=0; j < components.size(); ++j) {
                  		String comp = components.get(j).toString();
											boolean inComponents = res.indexOf("("+comp+")") > 0;
											if(!inComponents)
                      		good1.add(comp);
													
                      else
                   				bad1.add(comp);
                  } // for j
									log.info("Found Diagnosis.");
									diag_result_ic.good = good1;
          				diag_result_ic.bad  = bad1;
         					diagArr.add(diag_result_ic);	       
       				}// for i
         
         			dmsg.o_time =  now.getTimeInMillis();
         			dmsg.diag = diagArr;
         			publisher.publish(dmsg);
        			final DiagnosticArray d_arr_msg = node.getMessageFactory().newMessage("diagnostic_msgs/DiagnosticArray");
			  			d_arr_msg.header.frame_id = "Engine";
        			d_pub.publish(d_arr_msg);
        
        } // else inconsistent

      } catch (Exception e) {
              System.out.println("ERROR!"+e);
        }
       
	}// find_diagnosis()

   
	public void run() {
  	try{
       final Log log = node.getLog();
       while(true){
				 find_diagnosis();
         Thread.currentThread().sleep(900);
       }
   	}catch(Exception e){
           System.out.println(e);
     } 
	}//run

   protected ArrayList composeRepairCandidateResult(RepairCandidates rcs) {
        ArrayList result = new ArrayList(rcs.size());
        
        Iterator itCands = rcs.iterator();
        while (itCands.hasNext()) {
            RepairCandidate rc = (RepairCandidate)itCands.next();
            result.add(rc.toString());
        }

        return result;
    }

 protected LSentence parseSD() throws ParseError,
        IllegalUserInput{

        LogicParser parser = new LogicParser();
        LSentence allRules = new LSentence();

        generatePropNegationAxioms(PROP, parser, allRules);
        parseLogSentences(SD, parser, allRules);
  
        return allRules;
    }

 protected LSentence parseOBS() throws ParseError,
        IllegalUserInput{
        LogicParser parser = new LogicParser();
        LSentence allRules = new LSentence();

        parseLogSentences(OBS, parser, allRules);

        
        return allRules;
    }

protected void generatePropNegationAxioms(String text, LogicParser parser, 
                                              LSentence allRules) throws ParseError,
        IllegalUserInput
        {

        StringTokenizer tokenizer = new StringTokenizer(text, "\n");

        String negationPrefix = neg_prefix;

        while(tokenizer.hasMoreTokens()) {
            String prop = tokenizer.nextToken().trim();
            
            if (prop.length() > 0) { 
                String line = prop + ", " + negationPrefix + prop + " -> false.";
                
                if (parser.parse(line)) {
                    allRules.addRules((LSentence)parser.result());
                } else throw new ParseError(prop);
            }
        }
    }


  protected void parseLogSentences(String text, LogicParser parser, LSentence allRules) throws ParseError,
        IllegalUserInput
        {

        StringTokenizer tokenizer = new StringTokenizer(text, "\n");
        
        while(tokenizer.hasMoreTokens()) {
            String line = tokenizer.nextToken().trim();
            
            if (line.length() > 0) {                
                if (parser.parse(line)) {   
                    allRules.addRules((LSentence)parser.result());
                } else throw new ParseError(line);
            }
            
        } 
    }

  protected boolean checkConsistency(LSentence allRules) throws ParseError,
        IllegalUserInput {
        
        ABTheoremProver theoremProver = new ABTheoremProver();
        theoremProver = allRules.asABPropositionalSentence(theoremProver);
        if (theoremProver == null) {
            throw new ParseError("[unknown]");
        }

        boolean consistent;
        boolean useFaultModelsCB = true;

        if (useFaultModelsCB==true) {
            String assAB = AB; 
            String assNAB = NAB; 

            if (assAB == null) {
                throw new IllegalUserInput("Invalid AB assumption defined!");
            } else if (assNAB == null) {
                throw new IllegalUserInput("Invalid NAB assumption defined!");
            }

            ArrayList posAssPrefixes = new ArrayList();
            posAssPrefixes.add(assNAB);
            consistent = theoremProver.checkConsistency(posAssPrefixes);
        } else {
            consistent = theoremProver.checkConsistency();
        }

        // print conflict set to console

        ArrayList conflict = theoremProver.contradiction().collectAssumptions();
        printConflictSet(conflict);

        return consistent;
    }

  protected void printConflictSet(ArrayList conflict) {        

        String cs = "";
        Iterator itC = conflict.iterator();
        int i=0;
        while (itC.hasNext()) {
            Assumption a = (Assumption)itC.next();
            if (cs.length() == 0) cs += "-";
            else cs += " \\/ -";
            cs += (String)a.identifier;
            i = i + 1;
        }
      if(i!=0)
        System.out.println("\n\nConflict set returned by theorem prover: " + cs + "\n\n");
    }

  @Override
  public void onShutdown(Node node) {
    node.shutdown();
    node = null;
  }

}
