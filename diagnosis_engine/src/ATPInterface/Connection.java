/*
 * (c) copyright 2008, Technische Universitaet Graz and Technische Universitaet Wien
 *
 * This file is part of jdiagengine.
 *
 * jdiagengine is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * jdiagengine is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with jdiagengine. If not, see <http://www.gnu.org/licenses/>.
 *
 * Authors: Joerg Weber, Franz Wotawa
 * Contact: jweber@ist.tugraz.at (preferred), or fwotawa@ist.tugraz.at
 *
 */



package ATPInterface;

import java.net.*;
import java.io.*;
import java.util.*;

import theoremprover.IllegalAssumption;

import com.google.common.base.Preconditions;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;

/**
 * Manages a connection to the ATP server and handles client requests.
 *
 * This class is responsible for maintaining a connection to a client. Client requests are
 * processed, the necessary calls to the theorem prover are made, and when the connection
 * is closed, the socket is released.
 *
 * This class encapsulates the communication protocol.
 *
 * @author Joerg WEBER
 * @version 1.0, DATE: 18.10.2005
 *
 *
 */
public class Connection implements NodeMain {

    /*char SD[]={'P','O','S','T',' ','s','e','s','s','i','o','n','A',' ','A','D','D','_','S','E','N','T','E','N','C','E','S',' ',
    BufferedReader reader = new BufferedReader(new InputStreamReader(inStream));         'S','D',' ','A','T','P','\r','\n',
             'N','u','m','b','e','r','-','R','u','l','e','s',':',' ','2','\r','\n',
             '\r','\n',
             'N','A','B','(','A',')',' ','-','>',' ','a','.','\r','\n',
             'N','A','B','(','B',')',' ','-','>',' ','b','.','\r','\n',
             '\r','\n',
             '\r','\n'};
     */
    String SD = "POST sessionA ADD_SENTENCES SD ATP\r\nNumber-Rules: 3\r\n\r\nNAB(A) -> a.\r\nNAB(B) -> b.\r\nNAB(C) -> c.\r\n\r\n\r\n";
    private Node node;
    private int loc = -1;
    // These are the states of the connection.
    protected final static int STATE_WAIT_FIRST_POST = 0;  // init state: wait for first POST
    protected final static int STATE_DB_CHANGED = 1;  // after at least one POST is received
    protected final static int STATE_CLOSED = 2;  // a CLOSE request has been received

    // The period in which the connection checks if some data has arrived on the socket
    protected final static long SLEEP_MS = 20;

    // Response messages stating which error has occured.
    protected final static String MSG_ILLEGAL_HEADER = "Illegal request header";
    protected final static String MSG_ILLEGAL_RULE = "Syntax of logical sentence incorrect";
    protected final static String MSG_ILLEGAL_ASS = "Illegal assumption";
    protected final static String MSG_OK = "OK";
    protected final static String MSG_INTERNAL_SERVER_ERROR = "Internal server error";

    // The socket for the connection to the client.
    protected Socket socket;

    // Output stream for socket.
    protected DataOutputStream outputStream;

    // A container holding open sessions, see Server.openSessions.
    protected Map openSessions;

    // The session this connection is associated with.
    protected Session session;

    // The state of the connection, see the STATE_xy constants above.
    protected int state;

    // If true, then many data may be printed to the output.
    protected boolean verboseOutput;
    

    /** 
     * Constructor creating a connection for the passed socket. 
     *
     * Parameter openSessions: see Server.openSessions
     */

   @Override
    public void main(Node node) { 
	    Preconditions.checkState(this.node == null);
        this.node = node;
        openSessions = Collections.synchronizedMap(new TreeMap());
             

        try {
      Publisher<org.ros.message.std_msgs.String> publisher =
          node.newPublisher("chatter", "std_msgs/String");
      int seq = 0;
      //while (true) {
        org.ros.message.std_msgs.String str = new org.ros.message.std_msgs.String();
        str.data = "data! " + seq;
        //str.data = "Hello world! " + seq;
        publisher.publish(str);
        node.getLog().info("Hello! " + seq);
        
        seq++;
        Thread.sleep(1000);
        //System.out.println("start");
        start();
        //System.out.println("end");
      //}
    } catch (Exception e) {
      if (node != null) {
        node.getLog().fatal(e);
      } else {
        e.printStackTrace();
      } // else
    } // catch
   } //main

   @Override
   public void shutdown() {
    node.shutdown();
    node = null;
  }

   public Connection(Map openSessions, boolean verboseOutput) {
        
        this.openSessions = openSessions;
        this.verboseOutput = verboseOutput;
        state = STATE_WAIT_FIRST_POST;        
    }

    /*public Connection(Socket socket, Map openSessions, boolean verboseOutput) {
        
        assert(socket.isConnected());

        this.socket = socket;
        this.openSessions = openSessions;
        this.verboseOutput = verboseOutput;
        state = STATE_WAIT_FIRST_POST;        
    }*/


    /**
     * The run-method of the thread which handles incoming requests on this connection.
     *
     * This method has a loop which runs until a CLOSE request is received. The header and
     * the body of each incoming request is read (but not processed!). For different kinds
     * of request, different methods are called.  
     *
     * Finally, the socket is closed, and this connection is removed from the session it
     * is associated with. If there are no other connections related to this session, then
     * the session is closed (forever).
     */
    public void start() {
            
        try {
            // create input and output streams; allocate collections for header/body

            //InputStream inStream = socket.getInputStream();
            //BufferedReader reader = new BufferedReader(new InputStreamReader(inStream));
            BufferedReader reader = new BufferedReader(new StringReader(SD));
            
            //OutputStream outStream = socket.getOutputStream();
            //outputStream = new DataOutputStream(new BufferedOutputStream(outStream));
            outputStream = new DataOutputStream(new FileOutputStream("test1.txt"));
            boolean expectHeader = true;  // true if next input belongs to header
            boolean expectBody = false;   // true if next input belongs to body
            boolean lastLineEmpty = false;   // true if the last entered line was empty
            ArrayList header = new ArrayList();
            ArrayList body = new ArrayList();
            
            // the loop which runs until a CLOSE is received
            while (state != STATE_CLOSED) {               
                //System.out.println("start while");
                //String line = "";
                //char c = '0';
                //line = "POST sessionA ADD_SENTENCES SD ATP";
                //System.out.println(line);
								//if(SD[loc+2]!='\n')
                /*if(loc<(SD.length-1))
                  { 
                   do {
                      loc = loc + 1;
                      line = line + SD[loc];
                      //System.out.println("while");
                      } while(SD[loc]!='\n');
                  }*/
                 //}
                //System.out.println(line);
                //if(line.equals("\r\n")){
                  //line =null;
                  //System.out.println("line empty");}
         
                String line = reader.readLine();
                
                // if nothing received: taka a nap :-)
                
                if (line == null) {
                    try {
                        Thread.sleep(SLEEP_MS);
                    } catch (InterruptedException excInterrupted) {}
                    continue;
                }
                
                // data received on the socket
                
                if (line.length() > 0) {  // non-empty line received
                    if (expectHeader && !expectBody) {
                        header.add(line.trim());
                    } else if (!expectHeader && expectBody) {
                        body.add(line.trim());
                    } else {
                        assert(false);
                    }
                    
                    lastLineEmpty = false;
                    
                } else {  // empty line entered
                                        
                    if (expectHeader) {  // start waiting for body
                        expectHeader = false;
                        expectBody = true;
                        
                    } else if (lastLineEmpty) {  
                        
                        // body received, process the request if 2 empty lines entered

                        Date startProcessTime = new Date();
                        System.out.println("header"+header+",body"+body);
                        boolean ok = processRequest(header, body);
                        Date endProcessTime = new Date();

                        if (ok) {
                            long passedTime = endProcessTime.getTime() - startProcessTime.getTime();
                            System.out.println("Request successfully processed! Time [ms]: " + 
                                passedTime);
                        } else {
                            System.out.println("Error in request!");
                        }
                        
                        expectHeader = true;
                        expectBody = false;
                        
                        header.clear();
                        body.clear();
                    } 
                    
                    lastLineEmpty = true;
                }  // empty line entered
                
                
            }  // while (state != STATE_CLOSED)
            
            // clean-up of socket
            //socket.close();
            
        } catch (IOException e) {
            System.err.println("IOException in connection: " + e.getMessage());
            /*if (!socket.isClosed()) {
                try {
                     socket.close();
                    } catch (IOException eIO) {}
            }*/
        } catch (Throwable eUnknown) {
            System.err.println("Unexpected error during request processing:");
            System.err.println(eUnknown.getMessage());
            eUnknown.printStackTrace();
            
            /*try {
                if (!socket.isClosed()) {
                    sendResponse(ATPConstants.ERR_INTERNAL_SERVER_ERROR,
                                 MSG_INTERNAL_SERVER_ERROR, 
                                 new ArrayList(), new ArrayList());
                    socket.close();
                }
            } catch (IOException e) {}*/
        }
    
        // remove session from openSessions, if there are no other conns relating to session
        if (session != null) {
            removeFromSession();
        }

    }        

    
    /*
     * Removes this connection from the session, if there are no other connections
     * associated with this session: close the connection forever.
     */
    protected void removeFromSession() {
        assert(session != null);

        synchronized(session) {
            Vector sessionConns = session.getConnections();
            boolean exists = sessionConns.remove(this);
            assert(exists);
            
            System.out.println("Connection removed from session: " + session.getName());
            System.out.println("Remaining number of connections for this session: "
                               + String.valueOf(sessionConns.size()));
            
            if (sessionConns.size() == 0) {
                openSessions.remove(session.getName());
                session.close();
            }
            session = null;
        }

    }


    /**
     * Processes an incoming request consisting of a header and a body.
     *
     * The header and the body are passed as string collections.
     * This method considers only the first line of the header, then calls the appropriate
     * methods, for example processPOST() for POST requests.
     *
     * The return value denote if the request was successful (correct). The processXY() 
     * methods are responsible for creating and sending the response to the request.
     */
    protected boolean processRequest(ArrayList header, ArrayList body) throws IOException {
        
        System.out.println("processRequest");

        if (header.size() == 0) {
            sendResponse(ATPConstants.ERR_BAD_REQUEST, MSG_ILLEGAL_HEADER, new ArrayList(),
                         new ArrayList());
            return false;
        }
        else {
            String[] firstLine = ((String)header.get(0)).split(" ");
            
            if (firstLine.length == 0) {
                sendResponse(ATPConstants.ERR_BAD_REQUEST, MSG_ILLEGAL_HEADER, new ArrayList(),
                         new ArrayList());
                return false;
            }
            else {
                if (firstLine[0].equals(ATPConstants.CMD_POST)) 
                    return processPOST(header, body);
                else if (firstLine[0].equals(ATPConstants.CMD_GET)) 
                    return processGET(header, body);
                else if (firstLine[0].equals(ATPConstants.CMD_CLOSE)) 
                    return processCLOSE(header, body);
                else {
                    sendResponse(ATPConstants.ERR_BAD_REQUEST, MSG_ILLEGAL_HEADER, 
                                 new ArrayList(), new ArrayList());
                    return false;
                }
            }            
        }
    }

    protected boolean processPOST(ArrayList header, ArrayList body) throws IOException {
        System.out.println("process POST");

        // check if header and its first line have the required length

        if (header.size() < 2) {
            sendBadRequestResponse();
            return false;
        }
        
        String[] firstLine = ((String)header.get(0)).split(" ");
        if (firstLine.length < 4) {
            sendBadRequestResponse();
            return false;
        }

        // create new session or use existing one
        String sessionName = firstLine[1];        
        if (!associateWithSession(sessionName)) {
            sendBadRequestResponse();
                return false;
        }
        
        String subcmd = firstLine[2];  
        if (subcmd.equalsIgnoreCase(ATPConstants.SUBCMD_ADD_FDG_EDGES)) {
            
            if (firstLine.length != 4) {
                sendBadRequestResponse();
                return false;
            }

            if (!firstLine[3].equals(ATPConstants.STR_ATP)) {
                 sendBadRequestResponse();
                return false;
            }
            return processPOST_FDG(header, body, false, firstLine);

        } else if (subcmd.equalsIgnoreCase(ATPConstants.SUBCMD_REPLACE_FDG_EDGES)) {

            if (firstLine.length != 4) {
                sendBadRequestResponse();
                return false;
            }

            if (!firstLine[3].equals(ATPConstants.STR_ATP)) {
                 sendBadRequestResponse();
                return false;
            }
            return processPOST_FDG(header, body, true, firstLine);

        } else {
            
            if (firstLine.length != 5) {
                sendBadRequestResponse();
                return false;
            }

            if (!firstLine[4].equals(ATPConstants.STR_ATP)) {
                sendBadRequestResponse();
                return false;
            }

            boolean replace;
            if (subcmd.equalsIgnoreCase(ATPConstants.SUBCMD_ADD_SENTENCES)) replace = false;
            else if (subcmd.equalsIgnoreCase(ATPConstants.SUBCMD_REPLACE_SENTENCES)) replace = true;
            else {
                sendBadRequestResponse();
                return false;
            }

            return processPOST_SENTENCES(header, body, replace, firstLine);
        } 
            
    }  //processPOST()
    
    protected boolean processPOST_FDG(ArrayList header, ArrayList body,
                                      boolean replace,
                                      String[] firstLine) throws IOException {

        // read the remaining part of the header
        
        int numEdges = -1;
        for (int i = 1; i < header.size(); ++i) {
            String[] line = ((String)header.get(i)).split(" ");
            
            if (line.length != 2) return false;
            
            if (line[0].equalsIgnoreCase(ATPConstants.PARAM_CONTENT_TYPE)) {
                // do nothing: so far, only the MIME-type "text/plain" is supported
                
            } else if (line[0].equalsIgnoreCase(ATPConstants.PARAM_NUM_EDGES)) {
                try {
                    numEdges = new Integer(line[1]).intValue();
                } catch (NumberFormatException enf) {
                    sendBadRequestResponse();
                    return false;
                }
            }
        }

        // check if all obligatory parameters are defined, and if body has right length
        
        if ((numEdges < 0) || (body.size() != numEdges)) {
            sendBadRequestResponse();
            return false;
        }

        // finally: add edges to FDG
  
        int totalNumEdges;
        try {
            synchronized(session) {
                totalNumEdges = session.getDB().addFDGEdges(body, replace);
            }
        } catch (LogicParseException lpe) {
            sendParseErrorResponse(lpe.getLineNo());
            return false;
        }
        state = STATE_DB_CHANGED;
        
        // send OK response
        
        ArrayList respHeader = new ArrayList();
        String s = ATPConstants.PARAM_NUM_EDGES + " " + totalNumEdges;
        respHeader.add(s);
        sendResponse(ATPConstants.OK, MSG_OK, respHeader,
                     new ArrayList());
        return true;
    }
    

    // Process a POST request with logical sentences, update data base, create and send the response to the client.
    protected boolean processPOST_SENTENCES(ArrayList header, ArrayList body, boolean replace,
                                            String[] firstLine) throws IOException {
        
        String subDBName;
        int numRules = -1;
            
        // read parts of first line of header
        
        subDBName = firstLine[3];
        if (!validSubDBName(subDBName)) {
            sendBadRequestResponse();
            return false;
        }                                   

        // read the remaining part of the header
        
        for (int i = 1; i < header.size(); ++i) {
            String[] line = ((String)header.get(i)).split(" ");
            
            if (line.length != 2) return false;
            
            if (line[0].equalsIgnoreCase(ATPConstants.PARAM_CONTENT_TYPE)) {
                // do nothing: so far, only the MIME-type "text/plain" is supported
                
            } else if (line[0].equalsIgnoreCase(ATPConstants.PARAM_NUM_RULES)) {
                try {
                    numRules = new Integer(line[1]).intValue();
                } catch (NumberFormatException enf) {
                    sendBadRequestResponse();
                    return false;
                }
            }
        }
        
        // check if all obligatory parameters are defined, and if body has right length
        
        if ((numRules < 0) || (body.size() != numRules)) {
            sendBadRequestResponse();
            return false;
        }
        
        // finally: update database!
        
        int numSubDBRules;
        try {
            synchronized(session) {
                numSubDBRules = session.getDB().addRules(subDBName, replace, body);
            }
        } catch (LogicParseException lpe) {
            sendParseErrorResponse(lpe.getLineNo());
            return false;
        }
        state = STATE_DB_CHANGED;
        
        // send OK response
        
        ArrayList respHeader = new ArrayList();
        String s = ATPConstants.PARAM_SUBDB + " " + subDBName;
        respHeader.add(s);
        s = ATPConstants.PARAM_NUM_SUBDB_RULES + " " + numSubDBRules;
        respHeader.add(s);
        sendResponse(ATPConstants.OK, MSG_OK, respHeader,
                     new ArrayList());
        return true;
    
    }  // processPOST_SENTENCES()

    /*
     * When this method is called the first time, then it associates this connection with
     * a session (i.e. the "session" variable is set). If a session of this name does not
     * exist yet: create the session. 
     * In subsequent calls of this method, it is only checked if sessionName is equal to
     * the name of the associated session. If not: return "false" to denote an error.
     */ 
    protected boolean associateWithSession(String sessionName) {
        if (session == null) {   // so this is the first POST on this connection
            if (openSessions.containsKey(sessionName)) {
                session = (Session)openSessions.get(sessionName);
                System.out.println("Connection refers to existing session: "
                                   + sessionName);
            } else {
                session = new Session(sessionName);
                System.out.println("Create new session: " + sessionName);
                openSessions.put(sessionName, session);
            }

            synchronized(session) {
                session.getConnections().add(this);
            }
            return true;
        } else {  // this is not the first POST: check if right session name was sent
            return (sessionName.equals(session.getName()));
        }
    }
    

    // Process a GET request, query the data base, create and send the response to the client.
    protected boolean processGET(ArrayList header, ArrayList body) 
        throws IOException {

        System.out.println("process GET");

        // check if header and its first line have the required length
        
        if (header.size() < 1) {
            sendBadRequestResponse("Header line is missing");
            return false;
        }        
        String[] firstLine = ((String)header.get(0)).split(" ");
        if (firstLine.length != 3) {
            sendBadRequestResponse("First header line: wrong size");
            return false;
        }

        // check if a POST was sent before and if the right session name was passed

        String sessionName = firstLine[1];
        if ((session == null) || (!sessionName.equals(session.getName()))) {
            sendBadRequestResponse();
            return false;
        }
        
        // call appropriate method to process the request
        String actionStr = firstLine[2];
        if (actionStr.equals(ATPConstants.SUBCMD_CONSISTENCY)) {
            return processGET_CONSISTENCY(header); 
        } else if (actionStr.equals(ATPConstants.SUBCMD_CONSISTENCIES)) {
            return processGET_CONSISTENCIES(header, body); 
        } else if (actionStr.equals(ATPConstants.SUBCMD_MINDIAG)) {
            return processGET_MINDIAG(header);
        } else if (actionStr.equals(ATPConstants.SUBCMD_DBSTATS)) {
            return processGET_DBSTATS(header);
        } else if (actionStr.equals(ATPConstants.SUBCMD_DBCONTENT)) {
            return processGET_DBCONTENT(header);
        } else if (actionStr.equals(ATPConstants.SUBCMD_FDGSTATS)) {
            return processGET_FDGSTATS(header);
        } else if (actionStr.equals(ATPConstants.SUBCMD_DIAGENV)) {
            return processGET_DIAGENV(header);
        } else {
            System.out.println("000");
            sendBadRequestResponse();
            return false;
        }

    }  // processGET()


    // util method: returns true iff the subDBName is one of "SD", "OBS", "SDD".
    protected boolean validSubDBName(String subDBName) {
        return (subDBName.equals(ATPConstants.SUBDB_SD) || subDBName.equals(ATPConstants.SUBDB_OBS) 
                || subDBName.equals(ATPConstants.SUBDB_SDD));
    }

    protected boolean processGET_FDGSTATS(ArrayList header) throws IOException {
        ArrayList respHeader = new ArrayList();
        ArrayList respBody = new ArrayList();
        FDGStat stat;

        // create stats

        synchronized(session) {
            LogicalDBInterface db = session.getDB();
            stat = db.createFDGStats();
        }
        
        // generate response header

        String headerLine = ATPConstants.PARAM_NUM_NODES + " " + stat.numNodes;
        respHeader.add(headerLine);
        headerLine = ATPConstants.PARAM_NUM_EDGES + " " + stat.numEdges;
        respHeader.add(headerLine);

        // send response

        sendResponse(ATPConstants.OK, MSG_OK, respHeader, respBody);        
        return true;
    }

    // Process a GET DBSTATS request.
    protected boolean processGET_DBSTATS(ArrayList header) throws IOException {
        
        // read the name of the subDB whose statistical data shall be returned

        String subDBName = null;

        if (header.size() >= 2) {
            String[] line = ((String)header.get(1)).split(" ");
            if ((line.length != 2) 
                || (!line[0].equalsIgnoreCase(ATPConstants.PARAM_SUBDB))) {
                
                sendBadRequestResponse();
                return false;
            }
            subDBName = line[1];
            if (!validSubDBName(subDBName)) {
                sendBadRequestResponse();
                return false;
            }
        }

        // generate statistics

        ArrayList respHeader = new ArrayList();
        ArrayList respBody = new ArrayList();

        if (subDBName == null) {  // return stats related to whole DB
            int numSubDBs;
            int numRules;
            ArrayList subDBStats;

            synchronized(session) {
                LogicalDBInterface db = session.getDB();
                numRules = db.getTotalNumRules();
                subDBStats = db.createSubDBStats();
            }

            String headerLine = ATPConstants.PARAM_NUM_SUBDBS + " " + ATPConstants.NUM_SUBDBS;
            respHeader.add(headerLine);
            headerLine = ATPConstants.PARAM_NUM_RULES + " " + String.valueOf(numRules);
            respHeader.add(headerLine);
        
            Iterator itSubDB = subDBStats.iterator();
            while(itSubDB.hasNext()) {
                SubDBStat stat = (SubDBStat)itSubDB.next();
                String bodyLine = stat.name + ": " + String.valueOf(stat.numRules);
                respBody.add(bodyLine);
            }

        } else {  // requested stats is related to a certain sub-DB.
            
            int numRules;

            synchronized(session) {
                LogicalDBInterface db = session.getDB();
                numRules = db.getSubDBNumRules(subDBName);
            }

            String headerLine = ATPConstants.PARAM_NUM_SUBDBS + " 1";
            respHeader.add(headerLine);
            headerLine = ATPConstants.PARAM_NUM_RULES + " " + String.valueOf(numRules);
            respHeader.add(headerLine);
        }

        // send response
        sendResponse(ATPConstants.OK, MSG_OK, respHeader, respBody);        
        return true;
    }  // processGET_DBSTATS()


    // Process a GET DBCONTENT request.
    protected boolean processGET_DBCONTENT(ArrayList header) throws IOException {
        
        String subDBName = null;

        // read the name of the subDB 

        if (header.size() > 1) {
            String[] line = ((String)header.get(1)).split(" ");
            if ((line.length != 2) 
                || (!line[0].equalsIgnoreCase(ATPConstants.PARAM_SUBDB))) {
                
                sendBadRequestResponse();
                return false;
            }
            subDBName = line[1];
            if (!validSubDBName(subDBName)) {
                sendBadRequestResponse();
                return false;
            }
        }

        // determine rules (body) of response

        ArrayList respHeader = new ArrayList();
        ArrayList respBody = new ArrayList();

        int numRules;
        int numSubDBs;

        if (subDBName == null) {  // request includes the whole DB
           ArrayList subDBStats;

            synchronized(session) {
                LogicalDBInterface db = session.getDB();
                numSubDBs = ATPConstants.NUM_SUBDBS;
                numRules = db.getTotalNumRules();
                subDBStats = db.createSubDBStats();

                Iterator itSubDBStats = subDBStats.iterator();
                while(itSubDBStats.hasNext()) {
                    SubDBStat stat = (SubDBStat)itSubDBStats.next();
                    ArrayList subDBRules = db.getSubDBRules(stat.name);
                    respBody.addAll(createSubDBRuleStrings(stat.name, subDBRules));
                }
            }

        } else {  // request relates to selected subDBs only
            numRules = 0;
            numSubDBs = 1;

            synchronized(session) {
                LogicalDBInterface db = session.getDB();
                ArrayList subDBRules = db.getSubDBRules(subDBName);
                numRules += subDBRules.size();
                    respBody.addAll(createSubDBRuleStrings(subDBName, subDBRules));
            }
        }

        // create header of response

        String headerLine = ATPConstants.PARAM_NUM_SUBDBS + " " 
            + String.valueOf(numSubDBs);
        respHeader.add(headerLine);
        headerLine = ATPConstants.PARAM_NUM_RULES + " " + String.valueOf(numRules);
        respHeader.add(headerLine);

        // send response

        sendResponse(ATPConstants.OK, MSG_OK, respHeader, respBody);        
        return true;

    }  // processGET_CONTENT()
    

    /* 
     * Helper method, creates a sequence of strings of the format "x: rule", where x is an ID
     * of a sub database, and rule is a logical rule. 
     * The size of the returned collections is equal to the size of parameter "rules".
     */
    protected ArrayList createSubDBRuleStrings(String subDBName, ArrayList rules) {
        ArrayList result = new ArrayList(rules.size());

        Iterator it = rules.iterator();
        while(it.hasNext()) {
            String rule = (String)it.next();
            String s = subDBName + ": " + rule;
            result.add(s);
        }

        return result;
    }

    // Process a GET CONSISTENCIES request.
    protected boolean processGET_CONSISTENCIES(ArrayList header, ArrayList body) throws IOException {
        boolean useFaultModes = false;
        int numQueries = -1;

        // process remaining part of header
        
        for (int i = 1; i < header.size(); ++i) {
            String[] line = ((String)header.get(i)).split(" ");
            if (line.length < 2) {
                sendBadRequestResponse();
                return false;
            }
            
            if (line[0].equalsIgnoreCase(ATPConstants.PARAM_USE_FAULT_MODES)) {
                Boolean b = new Boolean(line[1]);
                useFaultModes = b.booleanValue();
            } else if (line[0].equalsIgnoreCase(ATPConstants.PARAM_NUM_QUERIES)) {
                try {
                    numQueries = new Integer(line[1]).intValue();
                } catch (NumberFormatException enf) {
                    sendBadRequestResponse();
                    return false;
                }
            } else {
                sendBadRequestResponse();
                return false;
            }
        }

        // check if body has the right length
        if (body.size() != numQueries) {
            sendBadRequestResponse();
            return false;
        }

        // perform consistency checks

        BitSet result = new BitSet();

        assert(session != null);
        try {
            synchronized(session) {
                LogicalDBInterface db = session.getDB();
                db.performConsistencyChecks(body, useFaultModes, result);
                assert(result.length() <= numQueries);
            }
        } catch (LogicParseException e) {
            sendResponse(ATPConstants.ERR_ILLEGAL_RULE, MSG_ILLEGAL_RULE, new ArrayList(),
                         new ArrayList());
            return false;
        }

        // construct response header (empty)
        ArrayList respHeader = new ArrayList();
        
        // construct response body
        
        ArrayList respBody = new ArrayList();
        for (int i = 0; i < numQueries; ++i) 
        {
            boolean value = result.get(i);
            String line;
            if (value) line = ATPConstants.STR_YES;
            else line = ATPConstants.STR_NO;
            respBody.add(line);
        }

        // send response

        sendResponse(ATPConstants.OK, MSG_OK, respHeader, respBody);
        return true;
    }

    // Process a GET CONSISTENCY request.
    protected boolean processGET_CONSISTENCY(ArrayList header) throws IOException {
        boolean consistent;

        boolean useFaultModes = false;

        // process remaining part of header
        
        for (int i = 1; i < header.size(); ++i) {
            String[] line = ((String)header.get(i)).split(" ");
            if (line.length < 2) {
                sendBadRequestResponse();
                return false;
            }

            try {
                if (line[0].equalsIgnoreCase(ATPConstants.PARAM_USE_FAULT_MODES)) {
                    Boolean b = new Boolean(line[1]);
                    useFaultModes = b.booleanValue();
                } else {
                    sendBadRequestResponse();
                    return false;
                }
            } catch (NumberFormatException e) {
                sendBadRequestResponse();
                return false;
            }
        }

        // call to theorem prover

        System.out.println("use fault modes: " + useFaultModes);

        assert(session != null);
        try {
            synchronized(session) {
                LogicalDBInterface db = session.getDB();
                consistent = db.checkConsistency(useFaultModes);
            }
        } catch (LogicParseException e) {
            sendResponse(ATPConstants.ERR_ILLEGAL_RULE, MSG_ILLEGAL_RULE, new ArrayList(),
                         new ArrayList());
            return false;
        }

        // construct response header

        String headerLine = ATPConstants.PARAM_CONSISTENT + " ";
        if (consistent) headerLine = headerLine + ATPConstants.STR_YES;
        else headerLine = headerLine + ATPConstants.STR_NO;

        ArrayList respHeader = new ArrayList();
        respHeader.add(headerLine);

        // send response

        sendResponse(ATPConstants.OK, MSG_OK, respHeader, new ArrayList());
        return true;

    }  // processGET_CONSISTENCY()

    protected boolean processGET_DIAGENV(ArrayList header) throws IOException {
        int maxExplSize = Integer.MAX_VALUE - 10;  // subtract a value: avoid overflows if there is a bug somewhere ;)
        int maxNumExpl = Integer.MAX_VALUE - 10;
        int maxDFChain = Integer.MAX_VALUE - 10;
        boolean inclBetaDEs = true;
        boolean mergeDEs = true;
        boolean discardOrderPerms = false;

        // process remaining part of header

        for (int i = 1; i < header.size(); ++i) {
            String[] line = ((String)header.get(i)).split(" ");
            if (line.length < 2) {
                sendBadRequestResponse();
                return false;
            }

            try {
                if (line[0].equalsIgnoreCase(ATPConstants.PARAM_MAX_DIAG_SIZE)) {
                    Integer n = new Integer(line[1]);
                    maxExplSize = n.intValue();
                } else if (line[0].equalsIgnoreCase(ATPConstants.PARAM_MAX_NUM_DIAG)) {
                    Integer n = new Integer(line[1]);
                    maxNumExpl = n.intValue();
                } else if (line[0].equalsIgnoreCase(ATPConstants.PARAM_MAX_DF_CHAIN)) {
                    Integer n = new Integer(line[1]);
                    maxDFChain = n.intValue();   
                } else if (line[0].equalsIgnoreCase(ATPConstants.PARAM_INCL_BETA_DE)) {
                    Boolean b = new Boolean(line[1]);
                    inclBetaDEs = b.booleanValue(); 
                } else if (line[0].equalsIgnoreCase(ATPConstants.PARAM_MERGE_DES)) {
                    Boolean b = new Boolean(line[1]);
                    mergeDEs = b.booleanValue();  
                } else if (line[0].equalsIgnoreCase(ATPConstants.PARAM_DISCARD_ORDER_PERMS)) {
                    Boolean b = new Boolean(line[1]);
                    discardOrderPerms = b.booleanValue();   
                } else {
                    sendBadRequestResponse();
                    return false;
                }
            } catch (NumberFormatException e) {
                sendBadRequestResponse();
                return false;
            }
        }

        // call to engine

        ArrayList diagEnvs;
        ArrayList minDiags = new ArrayList();

        boolean consistent;
        
        try {
            synchronized(session) {
                LogicalDBInterface db = session.getDB();
                consistent = db.checkConsistency(true);
                diagEnvs = db.computeDEs(maxExplSize, maxNumExpl, inclBetaDEs, maxDFChain, 
                                         mergeDEs, discardOrderPerms, minDiags);
            }
        } catch (LogicParseException e) {
            sendResponse(ATPConstants.ERR_ILLEGAL_RULE, MSG_ILLEGAL_RULE, new ArrayList(),
                         new ArrayList());
            return false;
        } catch (IllegalAssumption e) {
            sendResponse(ATPConstants.ERR_ILLEGAL_ASS, MSG_ILLEGAL_ASS, new ArrayList(),
                         new ArrayList());
            return false;
        }

        // construct response header

        String headerLine = ATPConstants.PARAM_CONSISTENT + " ";
        if (consistent) headerLine = headerLine + ATPConstants.STR_YES;
        else headerLine = headerLine + ATPConstants.STR_NO;

        ArrayList respHeader = new ArrayList();
        respHeader.add(headerLine);

        headerLine = ATPConstants.PARAM_NUM_DIAG + " " + minDiags.size();
        respHeader.add(headerLine);

        headerLine = ATPConstants.PARAM_NUM_DIAGENV + " " + diagEnvs.size();
        respHeader.add(headerLine);

        // construct response body
        
        ArrayList respBody = new ArrayList();
        
        Iterator itDiags = minDiags.iterator();
        while (itDiags.hasNext()) {
            String diagStr = ATPConstants.LINE_PREFIX_DIAG + " " + (String)itDiags.next();
            respBody.add(diagStr);
        }

        Iterator itDE = diagEnvs.iterator();
        while(itDE.hasNext()) {
            String deStr = ATPConstants.LINE_PREFIX_DE + " " + (String)itDE.next();
            respBody.add(deStr);
        }

        // send response

        sendResponse(ATPConstants.OK, MSG_OK, respHeader, respBody);
        return true;
    }

    // Process a GET EXPLANATIONS request.
    protected boolean processGET_MINDIAG(ArrayList header) throws IOException {
        
        boolean consistent;
        ArrayList explanations;
        
        int maxExplSize = Integer.MAX_VALUE - 10;
        int maxNumExpl = Integer.MAX_VALUE - 10;
        boolean useFaultModes = false;

        // process remaining part of header

        for (int i = 1; i < header.size(); ++i) {
            String[] line = ((String)header.get(i)).split(" ");
            if (line.length < 2) {
                sendBadRequestResponse();
                return false;
            }

            try {
                if (line[0].equalsIgnoreCase(ATPConstants.PARAM_MAX_DIAG_SIZE)) {
                    Integer n = new Integer(line[1]);
                    maxExplSize = n.intValue();
                } else if (line[0].equalsIgnoreCase(ATPConstants.PARAM_MAX_NUM_DIAG)) {
                    Integer n = new Integer(line[1]);
                    maxNumExpl = n.intValue();
                } else if (line[0].equalsIgnoreCase(ATPConstants.PARAM_USE_FAULT_MODES)) {
                    Boolean b = new Boolean(line[1]);
                    useFaultModes = b.booleanValue();
                    System.out.println("use fault modes: " + b);
                } else {
                    sendBadRequestResponse();
                    return false;
                }
            } catch (NumberFormatException e) {
                sendBadRequestResponse();
                return false;
            }
        }

        // call to theorem prover

        assert(session != null);
        try {
            synchronized(session) {
                LogicalDBInterface db = session.getDB();
                consistent = db.checkConsistency(useFaultModes);
                explanations = db.computeMinDiag(maxExplSize, maxNumExpl, useFaultModes, verboseOutput);
            }
        } catch (LogicParseException e) {
            sendResponse(ATPConstants.ERR_ILLEGAL_RULE, MSG_ILLEGAL_RULE, new ArrayList(),
                         new ArrayList());
            return false;
        } catch (IllegalAssumption e) {
            sendResponse(ATPConstants.ERR_ILLEGAL_ASS, MSG_ILLEGAL_ASS, new ArrayList(),
                         new ArrayList());
            return false;
        }

        // construct response header

        String headerLine = ATPConstants.PARAM_CONSISTENT + " ";
        if (consistent) headerLine = headerLine + ATPConstants.STR_YES;
        else headerLine = headerLine + ATPConstants.STR_NO;

        ArrayList respHeader = new ArrayList();
        respHeader.add(headerLine);

        headerLine = ATPConstants.PARAM_NUM_DIAG + " " 
            + String.valueOf(explanations.size());
        respHeader.add(headerLine);

        // construct response body
        
        ArrayList respBody = new ArrayList();
        Iterator it = explanations.iterator();
        while(it.hasNext()) {
            ArrayList explanation = (ArrayList)it.next();
            StringBuffer explStr = new StringBuffer("");
            
            Iterator itExpl = explanation.iterator();
            while(itExpl.hasNext()) {
                explStr.append((String)itExpl.next());
                if (itExpl.hasNext()) explStr.append(" ");
            }

            respBody.add(explStr.toString());
        }

        // send response

        sendResponse(ATPConstants.OK, MSG_OK, respHeader, respBody);
        return true;

    }  // processGET_MINDIAG((


    // Process a CLOSE request.
    protected boolean processCLOSE(ArrayList header, ArrayList body) {
        System.out.println("process CLOSE");

        state = STATE_CLOSED;
        return true;
    }

    
    // Helper method: write a \r\n sequence to stream.
    protected void writeLineBreak(OutputStream stream) throws IOException {
        stream.write('\r');
        stream.write('\n');
    }

    protected void sendBadRequestResponse() {
        try {
            sendResponse(ATPConstants.ERR_BAD_REQUEST, MSG_ILLEGAL_HEADER,
                         new ArrayList(), new ArrayList());
        } catch (IOException e) {} 
    }
    
    // Send a BAD REQUEST response to the client (client error, request is illegal)
    protected void sendBadRequestResponse(String detailedMsg) {
        try {
            sendResponse(ATPConstants.ERR_BAD_REQUEST, MSG_ILLEGAL_HEADER
                         + ": " + detailedMsg, new ArrayList(),
                         new ArrayList());
        } catch (IOException e) {}            
    }


    // Send an response to the client stating that the syntax of a logical rule is incorrect.
    protected void sendParseErrorResponse(int lineNo) {
        ArrayList header = new ArrayList();
        header.add(ATPConstants.PARAM_LINE_NUMBER + " " + String.valueOf(lineNo));
        try {
            sendResponse(ATPConstants.ERR_ILLEGAL_RULE, MSG_ILLEGAL_RULE, header,
                         new ArrayList()); 
        } catch (IOException e) {}
    }


    // Util method, send a response to the client with an error code and a message.
    protected void sendResponse(int errorCode, String msg, ArrayList header,
                                ArrayList body) throws IOException {

        String response = ATPConstants.STR_ATP + " " 
            + String.valueOf(errorCode) + " " + msg;
        outputStream.writeBytes(response);
        writeLineBreak(outputStream);

        for (int i = 0; i < header.size(); ++i) { 
            outputStream.writeBytes((String)header.get(i));
            writeLineBreak(outputStream);
        }
        writeLineBreak(outputStream);

        for (int i = 0; i < body.size(); ++i) {
            outputStream.writeBytes((String)body.get(i));
            writeLineBreak(outputStream);
        }
        writeLineBreak(outputStream);
        if (body.size() > 0) writeLineBreak(outputStream);

        outputStream.flush();
    }

}
