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
import java.io.IOException;
import java.util.*;

import com.google.common.base.Preconditions;


/**
 * A TCP server acting as network front-end to the assumption-based theorem prover.
 *
 * The Server class has a main method requiring two command line arguments: 
 * the IP address and the port the server binds to.
 *
 * The public method run() starts the server. The server accepts all incoming connections.
 * For each incoming connection, an instance of the class Connection is created and the
 * socket is passed to it. This instance is then responsible for handling the entire 
 * communication with the client and for closing the socket. A Connection instance runs
 * in its own thread.
 *
 * The communication protocol is called ATP (Assumption-based Theorem-prover Protocol).
 * It is a text-based protocol which has some similarities to HTTP.
 *
 * @author Joerg WEBER
 * @version 1.0, DATE: 18.10.2005
 *
 *
 */
public class Server{

    /** The IP address this server binds to. */
    protected InetAddress ip;

    /** The port this server binds to. */
    protected int port;
    
    /** 
     * A container of the sessions which are open at the moment.
     *
     * This container maps session names to session instances. A session basically 
     * maintains a logical database.
     * For each open session, there must be one or more connections which are 
     * associated with the session. A session is closed (by the Connection class) 
     * if there are no more connections referring to it.
     *
     * A thread-safe map must be used, since a session may be accessed by several
     * connections!!
     */
    protected Map openSessions;

    /**
     * Command line option may set this to true.
     */
    boolean verboseOutput = false;


    /** Prints some help lines explaining the required command line arguments. */
    protected static void printSyntaxHelp() {
        System.err.println("arguments: IP PORT [options]");
        System.err.println("possible options:");
        System.err.println("    -v   verbose output to stdout");
        System.err.println();
    }

    /**
     * The main method processes the command line arguments and then starts the 
     * server loop by calling run().
     */
    
    public static void main(String[] args) {
        System.err.println();

        System.out.println();
        System.out.println("ATP (Assumption-based Theorem prover Protocol) Server");

        if (args.length < 2) {
            printSyntaxHelp();
            return;
        }
            

        InetAddress ip;
        try {
            //ip = InetAddress.getByName(args[0]);
            ip = InetAddress.getByName("127.0.0.1");
        } catch (UnknownHostException e) {
            System.err.println("Unknown host IP address: + args[0]" );
            printSyntaxHelp();
            return;
        }

        Integer port;
        try {
            //port = new Integer(args[1]);
            port = new Integer("10000");
        } catch (NumberFormatException e) {
            System.err.println("The PORT argument is not a valid integer number!");
            printSyntaxHelp();
            return;
        }
          
        Server server = new Server(ip, port.intValue(), args, 2, args.length - 1);

        server.run();        
    }

    /** Constructor getting the IP address and the port. */
   public Server(InetAddress ip, int port, String[] options, int firstOptionIndex, int lastOptionIndex) {
        this.ip = ip;
        this.port = port;
        
        parseOptions(options, firstOptionIndex, lastOptionIndex);

        openSessions = Collections.synchronizedMap(new TreeMap());

    }

    protected void parseOptions(String[] args, int startIndex, int endIndex) {
        for (int iArgs = startIndex; iArgs <= endIndex; ++iArgs) {
            String opt = args[iArgs];
            if (opt.equals("-v")) {
                verboseOutput = true;
                System.out.println("Set option \"verbose output\" to TRUE");
            }
        }
    }

    /**
     * Creates a server socket and enters an infinite loop waiting for incoming connections.
     *
     * Creates a server socket and binds it to the IP and port passed to the constructor.
     * Then an infinite loop is entered, in which the server waits for incoming connections
     * For each incoming connection, a Connection instance is created and the socket
     * is passed to it. Then the thread of the Connection instance is started, and the
     * server continuous waiting for incoming connections.
     */
    public void run() {
        
        ServerSocket srvSocket;

        try {
            srvSocket = new ServerSocket(port, 5, ip);
        } catch (IOException e) {
            System.err.println("FATAL ERROR: server socket could not be created!");
            System.err.println("IOException message: " + e.getMessage());
            return;
        }

        System.out.println("Server socket created!");

        try {
            while(true) {
                
                Socket newSocket;
                newSocket = null;
                
                try {
                    newSocket = srvSocket.accept();
                } catch (IOException e) {
                    System.err.println("FATAL ERROR: server not able to accept "
                                       + "incoming connections!");
                    System.err.println("IOException message: " + e.getMessage());
                    return;
                }
                
                System.out.println("Incoming connection accepted!");
                
                try {
                    //Connection newConn = new Connection(newSocket, openSessions, verboseOutput);
                    //newConn.start();
                } catch(Exception e) {
                    System.err.println(e.getMessage());
                    e.printStackTrace();
                    if ((newSocket != null) && (!newSocket.isClosed())) {
                        
                        try {
                            newSocket.close();
                        } catch (Exception e1) {
                        }
                    }
                }
                
            }  // while(true)

        } finally {
            try {
                srvSocket.close();
            } catch (IOException eIO) {}
        }
    }  // run()

   
}
