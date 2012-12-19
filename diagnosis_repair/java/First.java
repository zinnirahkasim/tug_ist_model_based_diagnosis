/**
 * Created by Safdar
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

class First {
private final int CODE_OK = 100;
private final int CODE_ERROR_COMMAND = 200;
private final int CODE_ERROR_FILENAME = 300;
private final int CODE_RESULT = 400;
private final int CODE_EXIT = 500;

private final String MSG_OK = "OK";
private final String MSG_ILLEGAL_FIRST_KEYWORD = "Illigal first command word";
private final String MSG_ILLEGAL_SECOND_KEYWORD = "Illigal second command word";
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
protected DataOutputStream outputStream;
    First(){
      _options = Graphplan.getParserOptions();
      _parser = new Parser(_options);
    }
    public void run() {
        ServerSocket srvSocket;
        port = 10000;
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
               while (!line.equalsIgnoreCase(COMMAND_EXIT)) {               
                line = reader.readLine();
                System.out.println(line);
                line = line.trim();
                String[] firstLine = ((String)line).split(" ");
                String send_msg;
                
                if(firstLine[0].equalsIgnoreCase(COMMAND_UPLOAD)){
                   if (firstLine[1].equals(COMMAND_DOMAIN)){
                       boolean domain_loaded = load_domain_file(firstLine[2]);
                       if(domain_loaded) 
                           send_response(CODE_OK, MSG_OK);
                       else
                           send_response(CODE_ERROR_FILENAME, MSG_ILLEGAL_FILENAME);
                   }
                   else 
                   if (firstLine[1].equalsIgnoreCase(COMMAND_PROBLEM)){
                       boolean problem_loaded = load_problem_file(firstLine[2]);
                       if(problem_loaded)
                           send_response(CODE_OK, MSG_OK);
                       else
                           send_response(CODE_ERROR_FILENAME, MSG_ILLEGAL_FILENAME);
                   }
                   else
                       send_response(CODE_ERROR_COMMAND, MSG_ILLEGAL_SECOND_KEYWORD);
                }
                else if (firstLine[0].equalsIgnoreCase(COMMAND_RESULT)){
                       String results = processResult();
                       send_response(CODE_OK, results);
                    }
               else if (firstLine[0].equalsIgnoreCase(COMMAND_EXIT))
                       send_response(CODE_EXIT, MSG_EXIT);
               else    send_response(CODE_ERROR_COMMAND, MSG_ILLEGAL_FIRST_KEYWORD);

               }
               } catch (IOException e) {
                     System.err.println("IOException in connection: " + e.getMessage());
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
                System.out.println("Domain file does not exist!");
                return false;
      }
    }

    boolean load_problem_file(String problem_file){
       try{
          _problem = _parser.parse(new File(problem_file));
          return true;
       }catch(FileNotFoundException e){
                System.out.println("Repair Goal file does not exist!");
                return false;
      }
    }
    String processResult(){
      String result = "";
      _pb = _parser.link(_domain, _problem);
      
      try{
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
      return result;											
      }catch(Throwable t){
	  	System.err.println(t.getMessage());
        	t.printStackTrace(System.err);
                return result;
      }
    }
    void send_response(int err_code, String err_msg){
        String response = String.valueOf(err_code) + ": " + err_msg + "\n";
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
        First first = new First();
        first.run();
    }
}
