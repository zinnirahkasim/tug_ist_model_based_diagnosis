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

import java.util.*;

/**
 * A session is associated to one or more connections and maintains the access to a logical
 * DB.
 *
 * A session (and its database) may be shared by several connections. Thus, each access to
 * a session or its database MUST be enclosed by a "synchronized(session)" statement.
 *
 * If a session is no longer needed, then the close() method should be called. This method
 * must not be called if the session is still associated to one or more connections.
 *
 * @author Joerg WEBER
 * @version 1.0, DATE: 18.10.2005
 *
 *
 */
public class Session {

    // The name of the session.
    protected String name;

    // The connections this session is associated to.
    protected Vector connections;

    // The logical DB.
    protected LogicalDBInterface db;

    /**
     * Creates a session of a certain name.
     */ 
    public Session(String name) {
        this.name = name;
        connections = new Vector();
        db = new LogicalDB();
    }
    
    /**
     * Returns the name of this session.
     */
    public String getName() {
        return name;
    }

    /**
     * These are the connections this session is associated to.
     *
     * The session must not be closed if this collection still holds any connections.
     */
    public Vector getConnections() {
        assert(Thread.holdsLock(this));
        return connections;
    }

    /**
     * Returns the logical database of this session, DO NOT forget to synchronize the access
     * to it (see class description above)!!
     */
    public LogicalDBInterface getDB() {
        assert(Thread.holdsLock(this));
        return db;
    }

    /**
     * Closes the session and its database forever, can be called only if the collection
     * returned by getConnections() is empty.
     */
    public void close() {
        assert(Thread.holdsLock(this));
        assert(connections.size() == 0);
    }

}
