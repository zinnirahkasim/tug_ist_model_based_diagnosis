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

/*
 * Checks if a certain condition holds for a node.
 *
 * The condition is an object which implements the Condition interface.
 *
 * If the type of the visitor, which is passed to the constructor, 
 * is TYPE_CONJUNCTION, then the result of the evaluation is "true" iff
 * the cond. holds for all nodes. Furthermore, the initial value is "true".

 * For TYPE_DISJUNCTION, it is sufficient that the condition holds for
 * at least one node, but the initial value is "false".
 */
public class ConditionVisitor extends DoubleLinkedDAGVisitor {

    public final static int TYPE_CONJUNCTION = 0;
    public final static int TYPE_DISJUNCTION = 0;

    protected Condition cond;

    protected boolean holds = false;

    /*
     * One of TYPE_x.
     */
    protected int type;

    
    public ConditionVisitor(int type, Condition cond) {
        this.type = type;
        this.cond = cond;

        if (type == TYPE_CONJUNCTION) holds = true;
        else if (type == TYPE_DISJUNCTION) holds = false;
        else assert(false);
    }

    public boolean wantMoreNodes() {
        if (type == TYPE_CONJUNCTION) return holds;
        else return !holds;
    }

    public void visit(DoubleLinkedDAGNode node) {
        boolean b = cond.holds(node);

        if (type == TYPE_CONJUNCTION) holds = holds && b;
        else holds = holds || b;
    }

    public boolean conditionHolds() {
        return holds;
    }

}
