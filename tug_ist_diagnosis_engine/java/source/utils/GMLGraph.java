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


package utils;

import java.util.*;

/**
 * A graph in the GML portable graph file format.
 *
 * GML is a text-based file format for storing graphs. This class encapsulates this
 * this textual representation.
 *
 * GML: see http://www.infosun.fim.uni-passau.de/Graphlet/GML
 * 
 */
public class GMLGraph {
    
    protected static String GRAPH_HEADER = "graph [";
    protected static String GRAPH_FOOTER = "]";
    protected static String NODE_HEADER = "node [";
    protected static String NODE_FOOTER = "]";
    protected static String EDGE_HEADER = "edge [";
    protected static String EDGE_FOOTER = "]";
    
    protected static String KEY_ID = "id";
    protected static String KEY_LABEL = "label";
    protected static String KEY_SOURCE = "source";
    protected static String KEY_TARGET = "target";

    protected Properties headerProps = new Properties();

    //! List of strings, contains all text lines for the nodes.
    protected ArrayList nodeLines;

    //! List of strings, contains all text lines for the edges.
    protected ArrayList edgeLines;

    //! These lines are added to each node definition
    protected Collection additionalNodeLines;

    protected Collection additionalEdgeLines;


    /*!
     * This property is added to the header of the GML graph.
     *
     * In other words, the property is a key-value pair in the top level of the graph.
     */
    public void addHeaderProperty(String key, String value) {
        headerProps.setProperty(key, value);
    }

    /*!
     * Set lines of GML code which are added to each GML node definition.
     */
    public void defineAdditionalNodeLines(Collection additionalNodeLines) {
        this.additionalNodeLines = additionalNodeLines;
    }

    /*!
     * Set lines of GML code which are added to each GML edge definition.
     */
    public void defineAdditionalEdgeLines(Collection additionalEdgeLines) {
        this.additionalEdgeLines = additionalEdgeLines;
    }
    
    /*!
     * Generates the GML code representing \a source.
     */
    public void generateFrom(DoubleLinkedDAG source) {
        
        nodeLines = new ArrayList();
        edgeLines = new ArrayList();
        Properties nodeProps = new Properties();
        Properties edgeProps = new Properties();

        Collection nodes = source.getNodes();
        Iterator itNodes = nodes.iterator();

        while (itNodes.hasNext()) {
            
            // generate GML lines for node

            DoubleLinkedDAGNode node = (DoubleLinkedDAGNode)itNodes.next();
            assert(node instanceof GMLNode);
            if (((GMLNode)node).includeInGMLCode()) {
            
                generateNodeProps(node, nodeProps);
                
                nodeLines.add(NODE_HEADER);
                addProperties(nodeProps, nodeLines);
                nodeProps.clear();
                if (additionalNodeLines != null) nodeLines.addAll(additionalNodeLines);
                nodeLines.add(NODE_FOOTER);
                
                // generate GML lines for the edges from node to its children
                
                Iterator itChildren = node.getChildrenIterator();
                while (itChildren.hasNext()) {
                    DoubleLinkedDAGNode child = (DoubleLinkedDAGNode)itChildren.next();
                    assert(child instanceof GMLNode);
                    if (((GMLNode)child).includeInGMLCode()) {

                        generateEdgeProps(node, child, edgeProps);
                        
                        edgeLines.add(EDGE_HEADER);
                        addProperties(edgeProps, edgeLines);
                        edgeProps.clear();
                        if (additionalEdgeLines != null) edgeLines.addAll(additionalEdgeLines);
                        edgeLines.add(EDGE_FOOTER);                
                    }
                }
            }
        }        
    }

    /*!
     * Returns the complete GML code.
     *
     * The resulting lines of code can be written to a file. The resulting file is a complete
     * GML file.
     */
    public Collection getGMLCode() {
        assert((nodeLines != null) && (edgeLines != null));

        ArrayList result = new ArrayList();
        result.add(GRAPH_HEADER);
        addProperties(headerProps, result);
        result.addAll(nodeLines);
        result.addAll(edgeLines);
        result.add(GRAPH_FOOTER);

        return result;
    }

    protected void addProperties(Properties props, Collection lines) {
        Iterator itProps = props.entrySet().iterator();
        while (itProps.hasNext()) {
            Map.Entry entry = (Map.Entry)itProps.next();
            assert((entry.getKey() instanceof String) && (entry.getValue() instanceof String));
            lines.add(entry.getKey().toString() + " " + entry.getValue().toString());
        }
    }

    protected void generateNodeProps(DoubleLinkedDAGNode node, Properties props) {
        assert(node instanceof GMLNode);

        props.setProperty(KEY_ID, (new Integer(node.getID())).toString());
        props.setProperty(KEY_LABEL, "\"" + ((GMLNode)node).getLabel() + "\"");

    }

    protected void generateEdgeProps(DoubleLinkedDAGNode from, DoubleLinkedDAGNode to,
                                     Properties props) {

        assert((from instanceof GMLNode) && (to instanceof GMLNode));

        props.setProperty(KEY_SOURCE, (new Integer(from.getID())).toString());
        props.setProperty(KEY_TARGET, (new Integer(to.getID())).toString());
    }

}
