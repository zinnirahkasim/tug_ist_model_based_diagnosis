/* Diagnosis Planner TCP Server 

# Copyright (c). 2012. OWNER: Institute for Software Technology TU-Graz Austria.
# Authors: Safdar Zaman, Gerald Steinbauer. (szaman@ist.tugraz.at, steinbauer@ist.tugraz.at)
# All rights reserved.
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation 
# and/or other materials provided with the distribution.
# 3. Neither the name of the <ORGANIZATION> nor the names of its contributors
# may be used to endorse or promote products derived from this software without
# specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSE-
# QUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
# GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
# OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE.
##
# Diagnosis Planner is a TCP protocol based Server that interacts with diagnosis_repair for repair.

*/
import java.net.*;
import java.io.IOException;
import java.util.*;
import pddl4j.Parser;
import pddl4j.PDDLObject;
import java.io.*;
import java.util.Set;
import pddl4j.exp.AtomicFormula;
import java.util.List;
import pddl4j.exp.term.Term;

class Diagnosis_planner {
private final int CODE_OK = 100;
private final int CODE_ERROR_COMMAND = 200;
private final int CODE_ERROR_FILENAME = 300;
private final int CODE_RESULT = 400;
private final int CODE_ERROR_RESULT = 500;
private final int CODE_EXIT = 600;

private final String MSG_OK = "OK";
private final String MSG_ILLEGAL_FIRST_KEYWORD = "Illigal first command word";
private final String MSG_ILLEGAL_SECOND_KEYWORD = "Illigal second command word";
private final String MSG_ERROR_RESULT = "Nil result, domain and/or problem file error";
private final String MSG_ILLEGAL_FILENAME = "File not exists";
private final String MSG_EXIT = "Server shutdown";

private final String COMMAND_UPLOAD = "UPLOAD";
private final String COMMAND_DOMAIN = "DOMAIN_FILE";
private final String COMMAND_PROBLEM = "PROBLEM_FILE";
private final String COMMAND_RESULT = "RESULT";
private final String COMMAND_EXIT = "EXIT";

private Properties _options;
private Parser _parser;
private PDDLObject _domain;
private PDDLObject _problem;
private PDDLObject _pb;
protected InetAddress ip;
protected int port;
protected String header,body;
protected DataOutputStream outputStream;
private String domain_file, problem_file_path;
    Diagnosis_planner(String domain_file){
      this.domain_file = domain_file;
      problem_file_path = this.getClass().getProtectionDomain().getCodeSource().getLocation().toString().substring(5);
      System.out.println("you gave path:"+this.domain_file);
      _options = Graphplan.getParserOptions();
      _parser = new Parser(_options);
    }
    public void run() {
        ServerSocket srvSocket;
        port = 10001;
        try {
            ip = InetAddress.getByName("127.0.0.1");
        } catch (UnknownHostException e) {
            System.err.println("Unknown host IP address: ");
            return;
        }
        try {
            srvSocket = new ServerSocket(port, 5, ip);
        } catch (IOException e) {
            System.err.println("FATAL ERROR: server socket could not be created!");
            System.err.println("IOException message: " + e.getMessage());
            return;
        }
        System.out.println("Diagnosis Server socket created!");
        try {
                Socket newSocket;
                newSocket = null;
                
                try {
                    newSocket = srvSocket.accept();
                } catch (IOException e) {
                    System.err.println("Server cannot make new Connection! ");
                    return;
                }
                
               System.out.println("Incoming connection accepted!");
               try{
               InputStream inStream = newSocket.getInputStream();
               BufferedReader reader = new BufferedReader(new InputStreamReader(inStream));
            
               OutputStream outStream = newSocket.getOutputStream();
               outputStream = new DataOutputStream(new BufferedOutputStream(outStream));
               String line = "";
	       boolean first_line = true;
	       boolean end_data = false;
               while (!line.equalsIgnoreCase(COMMAND_EXIT)) {               
                line = reader.readLine();
		if(line.length() > 0){
		   	if(first_line){
				header = line;
				body = "";
				first_line = false;
			}
			else
				body += line;
			end_data = false;
		}else{ if(first_line)
			first_line = false;
		      else if(end_data){
				first_line = true;
				System.out.println("header:"+header);
				System.out.println("body:"+body);
				String[] headerLine = ((String)header).split(" ");
				header = "";
                		String send_msg;
                
                		if(headerLine[0].equalsIgnoreCase(COMMAND_UPLOAD)){
                   			if (headerLine[1].equalsIgnoreCase(COMMAND_DOMAIN)){
                       				boolean domain_loaded = load_domain_file(domain_file);
                       				if(domain_loaded) 
                           				send_response(CODE_OK, MSG_OK);
                       				else
                           				send_response(CODE_ERROR_FILENAME, MSG_ILLEGAL_FILENAME);
                   			}else if (headerLine[1].equalsIgnoreCase(COMMAND_PROBLEM)){
                       				boolean problem_loaded = load_problem_file(problem_file_path+"repair_problem.pddl");
                       				if(problem_loaded)
                           				send_response(CODE_OK, MSG_OK);
                       				else
                           				send_response(CODE_ERROR_FILENAME, MSG_ILLEGAL_FILENAME);
                   			}else
                       				send_response(CODE_ERROR_COMMAND, MSG_ILLEGAL_SECOND_KEYWORD);
                		}else if (headerLine[0].equalsIgnoreCase(COMMAND_RESULT)){
                       				String result = processResult();
                       				if(result.equals("nil"))
                           				send_response(CODE_ERROR_RESULT, MSG_ERROR_RESULT);
                       				else
                           				send_response(CODE_RESULT, result);
                    		}else if (headerLine[0].equalsIgnoreCase(COMMAND_EXIT))
                       				send_response(CODE_EXIT, MSG_EXIT);
               				else    send_response(CODE_ERROR_COMMAND, MSG_ILLEGAL_FIRST_KEYWORD);
							
		     } 
            end_data = true;
            }
				
                

               }
               } catch (IOException e) {
                     System.err.println("IOException in connection: " + e.getMessage());
                     e.printStackTrace(System.err);
               if (!newSocket.isClosed()) {
                 try {
                    newSocket.close();
                 } catch (IOException eIO) {}
             }
             }
                       
             closeSocket(srvSocket);      

        } finally {
            closeSocket(srvSocket);
        }
    }  // run()
    boolean load_domain_file(String domain_file){
       try{
          _domain = _parser.parse(new File(domain_file));
          return true;
       }catch(FileNotFoundException e){
                System.out.println("Domain file does not exist!"+e.toString());
                return false;
      }
    }

    boolean load_problem_file(String problem_file){
       try{
	  BufferedWriter out=new BufferedWriter(new FileWriter(problem_file));
          out.write(body);
          out.close();
          _problem = _parser.parse(new File(problem_file));
          return true;
       }catch(FileNotFoundException e){
                System.out.println("Repair Goal file does not exist!");
                return false;
      } catch(IOException e){System.out.println(e.toString());return false;}
    }
    String processResult(){
      String result = "";
      
      try{
         _pb = _parser.link(_domain, _problem);
         Graphplan gplan = new Graphplan(_pb);
         gplan.preprocessing();
         Plan plan = gplan.solve();
         if (plan != Plan.FAILURE) {
        	String actionServer=null;
		String param = null;
		for (Set<AtomicFormula> layer : plan) {
		     for (AtomicFormula action : layer) {
			actionServer = action.getPredicate();
			ArrayList<String> params = new ArrayList<String>();
			    for (Term parameter : action){ 
                               params.add(parameter.getImage());
                               result += actionServer+"("+params.get(0).toString()+") ";
                            }
                      }
                }
          }
      plan.actions.clear();
      return result;											
      }catch(Throwable t){
	  	return "nil";
      }
    }
    void send_response(int msg_code, String msg){
        String response = String.valueOf(msg_code) + ": " + msg + "\n";
	System.out.println(response);
        try{
           outputStream.writeBytes(response);
           outputStream.flush();
        }
        catch (IOException e){
        System.out.println(e.getMessage());
        }
    }
    void closeSocket(ServerSocket sock){
        try {
                sock.close();
            } catch (IOException eIO) {
                  System.out.println("Error while closing the socket");
              }
    }
    public static void main(String[] args) {
	 if(args.length < 1){
              System.out.println("Error: repair_domain.pddl file required with complete path as argument!");
              System.out.println("e.g. \"/home/../repair_domain.pddl\"");
         }else{
               Diagnosis_planner diag_planner = new Diagnosis_planner(args[0]);
               diag_planner.run();
         }
    }
}
