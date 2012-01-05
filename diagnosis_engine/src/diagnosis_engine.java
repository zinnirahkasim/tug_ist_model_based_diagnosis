import com.google.common.base.Preconditions;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import java.util.*;
import java.io.*;
import java.text.*;
import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import java.lang.System;
import org.ros.message.diagnosis_msgs.Diagnosis;
import org.ros.message.diagnosis_msgs.Observations;

import hittingsetalg.*;
import theoremprover.*;
import utils.*;
import dfengine.*;

/**
 * This is a diagnosic_engine class of the Model Based Diagnostics. It assumes an
 * external roscore is already running.
 * 
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
  private boolean processObs = false;
  //private boolean gotSD = false;

 	private Publisher<org.ros.message.diagnosis_msgs.Diagnosis> publisher;
  private Calendar now = Calendar.getInstance();
  public diagnosis_engine()
  {
       super("FetchResult"+1);
       AB = "";
       NAB = "";
       neg_prefix = "";
       
  }
  @Override
  public void onStart(Node node){ 
    try{
        main_engine(node);
        }catch(Exception e){}
  }
  public void main_engine(Node node) throws ParseError,
        IllegalUserInput{
  try {
      this.node = node;
      String path=null;
      final Log log = node.getLog();
      
      System.out.println("Diagnosis Engine is up.......");
      publisher = node.newPublisher("/Diagnosis", "diagnosis_msgs/Diagnosis");

      node.newSubscriber("/Diagnostic_Model", "diagnosis_msgs/SystemDescription",
          new MessageListener<org.ros.message.diagnosis_msgs.SystemDescription>() {
            @Override
            public void onNewMessage(org.ros.message.diagnosis_msgs.SystemDescription sd_msg) {
            try { 
                 String[] rules = (String[]) sd_msg.rules.toArray(new String[0]);
                 String[] props = (String[]) sd_msg.props.toArray(new String[0]);
                 //System.out.println("Subscriber got up");
                 //System.out.println("Time="+sd_msg.out_time+",AB="+sd_msg.AB+",NAB="+sd_msg.NAB+",Neg_Prefix="+sd_msg.neg_prefix+", No of  				 Rules:"+rules.length+", No of Props:"+props.length);
                  String s="";
                  for(int m=0; m<rules.length; m++)
                  {  s = s + rules[m];
                  }
                  SD = s;
                 //System.out.println("Rules:"+s);
                 s="";
                 for(int m=0; m<props.length; m++)
                  {  s = s + props[m];
                  }
                 //System.out.println("Propos:"+s);
                 PROP = s;
                 String AB1 = sd_msg.AB;
                 String NAB1 = sd_msg.NAB;
                 String neg_prefix1 = sd_msg.neg_prefix;
                 //System.out.println("AB="+AB1+",NAB="+NAB1+",Neg_Prefix="+neg_prefix1+", Rules:"+SD+", Props:"+ PROP);
                 start();
                }catch(Exception e) 
                 {System.out.println("Error");System.out.println(e);
                  }
          
      }
      }
      );


node.newSubscriber("/Diagnostic_Observation", "diagnosis_msgs/Observations",
          new MessageListener<org.ros.message.diagnosis_msgs.Observations>() {
            @Override
            public void onNewMessage(org.ros.message.diagnosis_msgs.Observations msg) {
              if(!processObs)
              {
               String[] obs_msg = (String[]) msg.obs.toArray(new String[0]);
               for(int m=0; m<obs_msg.length; m++)
                 { String s = obs_msg[m];
                  boolean found = false;
       						for(String st : msg_list)
                   if(s.equals(st))
             					{
               					found = true;
               					break;
              				}
                    
      						if(!found)
         						{ String sub_str="";
           						String str = obs_msg[m];
          						int p;
          						if(str.charAt(2)=='(')
            							p = 1;
          						else
            							p = 3;
          						while(str.charAt(p)!='_')
            						sub_str = sub_str + str.charAt(++p);
          						String ostr = "ok" + sub_str + "Frequency)" ;
          						String nstr = "n_ok" + sub_str + "Frequency)";
          						String ndstr = "n_ok" + sub_str + "Node)";
          						for(int i=0;i<msg_list.size();i++)
            							if(ostr.equals(msg_list.get(i)) || nstr.equals(msg_list.get(i)) || ndstr.equals(msg_list.get(i)))
                						{
                              msg_list.remove(i);
                 							continue;
                						}
         								msg_list.add(str);
                      
         							} // if(!found)
     
                } // for int i
             
            } // if(!processObs)
            //else System.out.println("Halted");
           } //onMessage   
          });

   
    } catch (Exception e) {
      if (node != null) {
        System.out.println(e);
        node.getLog().fatal(e);
        
      } else {
        e.printStackTrace();
      }
    }
        

  } //main


void find_diag(org.ros.message.std_msgs.String message)
    {   

    try{ 
     OBS = new String(message.data);
          
    

    LSentence sd = parseSD();
    LSentence obs = parseOBS();
    LSentence indepModel = new LSentence();
    indepModel.addRules(sd);
    indepModel.addRules(obs);

    ArrayList result;

        boolean consistent = checkConsistency(indepModel);
        if (consistent) {
            result = new ArrayList();
            result.add("Consistent!");
            System.out.println("Consistent!");

        } else {  // inconsistent

          ABTheoremProver thrmp = new ABTheoremProver();
          thrmp = indepModel.asABPropositionalSentence(thrmp);
          System.out.println("Not consistent!");
			    ArrayList diagnoses = new ArrayList();  // list of String
			    ArrayList conflictSets = new ArrayList();  // list of String
			    MinHittingSetsFM  hsFM=null;
			    try {
		         hsFM = new MinHittingSetsFM(false, thrmp, "AB", "NAB");
		         }catch (Exception e) {
	               System.out.println("Illigal assumptions");
			         }
		      int computationResult = hsFM.compute(10, 100);
			    boolean hasMoreDiags = (computationResult != MinHittingSets.CS_ALL_MIN_DIAGS_COMPUTED);
			    ArrayList minHittingSetsAsAss = hsFM.getMinHS();
			    ArrayList conflictsAsAss = hsFM.getConflictsAsAss();
           
         
         
         org.ros.message.diagnosis_msgs.DiagnosisResults diag_result =  new org.ros.message.diagnosis_msgs.DiagnosisResults();
         ArrayList<org.ros.message.diagnosis_msgs.DiagnosisResults>	diagArr = new                    				     ArrayList<org.ros.message.diagnosis_msgs.DiagnosisResults>();
         
         

         
        final Diagnosis dmsg = node.getMessageFactory().newMessage("diagnosis_msgs/Diagnosis");
        ArrayList<String> good = new ArrayList<String>();  
        ArrayList<String> bad = new ArrayList<String>();
        for (int i = 0; i < minHittingSetsAsAss.size(); ++i) 
			       {                  
                bad.add(minHittingSetsAsAss.get(i).toString());
                for(int j=0; j < minHittingSetsAsAss.size(); ++j)
                  if(j!=i)
                   good.add(minHittingSetsAsAss.get(j).toString());
                
                
                System.out.println(minHittingSetsAsAss.get(i));
			       }
         
         diag_result.good = good;
         diag_result.bad  = bad;
         dmsg.o_time =  now.getTimeInMillis();
         diagArr.add(diag_result);
         dmsg.diag = diagArr;
         publisher.publish(dmsg);
			    
        } // else inconsistent

     }catch (Exception e) {
       System.out.println("File Read Error!"+e);
       }
       
}// function

   
public void run() {
  try{
       while(true) {
         org.ros.message.std_msgs.String str = new org.ros.message.std_msgs.String();
         Thread.currentThread().sleep(1000);
         processObs = true;
         Thread.currentThread().sleep(1000);
         for(int i=0;i<msg_list.size();i++)
                 { str.data = str.data + msg_list.get(i) + ".";
                  }
         //System.out.println(str.data);
         find_diag(str);
         processObs = false;
       }
   }
   catch(Exception e) {System.out.println(e);
   } 
 } // run

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

        String negationPrefix = neg_prefix; //"n_";

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

        //String text = textArea.getText();
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
        
        // prepare theorem prover, then check consistency

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