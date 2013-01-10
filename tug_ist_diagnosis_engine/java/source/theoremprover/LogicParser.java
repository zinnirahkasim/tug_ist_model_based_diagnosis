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


/**
 * LogicParser: Implementation of a parser for logic programs
 *
 * @version 0.2, DATE: 30.12.1998
 * @author Franz Wotawa
 *
 * This class provides an implementation of a parser
 * for logic programs. It understands rules of the form
 * 
 * a1, .. ,an -> an+1. 
 * -> a.
 * an+1 :- an, .. ,a1.
 * a.
 *
 * V0.1: Implementing the basic functionality (29.12.1998)
 * V0.2: Adding LObject support (30.12.1998)
 */

package theoremprover;

import java.lang.*;    // Java language classes
import java.util.*;
import java.awt.*;     // User Interface components
import java.io.*;      // IO specific classes (File, Stream,..)
import java.awt.event.*;   // Event handling

import theoremprover.*;


public class LogicParser extends GenericParser
{

    // Instance creation and initialization

    public LogicParser()
    {
	scanner = defaultScanner();
	source = "";
	actualToken = null;
	result = null;
    }

    public LogicParser(String str)
    {
	scanner = defaultScanner();
	source = str;
	actualToken = null;
	result = null;
    }

    // Parsing

    public Object defaultResult()
    {
	return new LSentence();
    }

    // Private parsing

    public void parse () throws ParserErrorException
    {
	if (actualToken.isEOI()) {
	    // do nothing
	} else {
	    parseSentence();
	    parse();
	}
    }

    public void parseSentence () throws ParserErrorException
    {
	ArrayList tail = new ArrayList();
	LPredicate head;

	if (actualToken.isDelimiter() && actualToken.equalValue("->")) {
	    nextToken();
	    head = parsePredicate();
	    if (actualToken.isDelimiter() && 
		actualToken.equalValue(".")) {
		(((theoremprover.LSentence)result).rules).
		    add(new LRule(tail,head));
		nextToken();
	    } else {
		errorDetected("'.' expected");
	    }
	} else {
	    LPredicate pred;
	    pred = parsePredicate();
	    if (actualToken.isDelimiter() &&
		actualToken.equalValue(",")) {
		tail.add(pred);
		tail = parseAntecedenceRest(tail);
		if (actualToken.isDelimiter() &&
		    actualToken.equalValue("->")) {
		    nextToken();
		    head = parsePredicate();
		    if (actualToken.isDelimiter() && 
			actualToken.equalValue(".")) {
			(((theoremprover.LSentence)result).rules).
			    add(new LRule(tail,head));
			nextToken();
		    } else {
			errorDetected("'.' expected");
		    }
		}
	    } else if (actualToken.isDelimiter() &&
		       actualToken.equalValue(":-")) {
		nextToken();
		head = pred;
		tail = parseAntecedence(tail);
		if (actualToken.isDelimiter() &&
		    actualToken.equalValue(".")) {
		    (((theoremprover.LSentence)result).rules).
			add(new LRule(tail,head));
		    nextToken();
		} else {
		    errorDetected("'.' expected");
		}
	    } else if (actualToken.isDelimiter() &&
		       actualToken.equalValue(".")) {
		head = pred;
		(((theoremprover.LSentence)result).rules).
		    add(new LRule(tail,head));
		nextToken();
	    } else if (actualToken.isDelimiter() &&
		       actualToken.equalValue("->")) {
		nextToken();
		tail.add(pred);
		head = parsePredicate();
		if (actualToken.isDelimiter() && 
		    actualToken.equalValue(".")) {
		    (((theoremprover.LSentence)result).rules).
			add(new LRule(tail,head));
		    nextToken();
		} else {
		    errorDetected("'.' expected");
		}
	    } else {
		errorDetected("',', ':-', '->', or '.' expected");
	    }
	}
    }

    public LPredicate parsePredicate () throws ParserErrorException
    {
	LPredicate pred = new LPredicate();
	if (actualToken.isIdentifier() || actualToken.isString()) {
	    pred.identifier = actualToken.value();
	    nextToken();
	    if (actualToken.isDelimiter() && 
		actualToken.equalValue("(")) {
		nextToken();
		pred.arguments = parseArguments(new ArrayList());
		if (actualToken.isDelimiter() &&
		    actualToken.equalValue(")")) {
		    nextToken();
		} else {
		    errorDetected("')' expected");
		}
	    }
	} else {
	    errorDetected("Identifier or string expected");
	}
	return pred;
    }

    public ArrayList parseArguments (ArrayList v) throws ParserErrorException
    {
	if (actualToken.isDelimiter() &&
	    actualToken.equalValue(")")) {
	    return v;
	}
	while (true) {
	    v.add(parseFunction());
	    if (actualToken.isDelimiter() &&
		actualToken.equalValue(",")) {
		nextToken();
	    } else {
		return v;
	    }
	}
    }

    public LObject parseFunction () throws ParserErrorException
    {
	LObject obj = null;

	if (actualToken.isIdentifier() ||
	    actualToken.isString() ||
	    actualToken.isCharacter()) {
	    String val;
	    if (actualToken.isIdentifier()) {
		val = actualToken.value();
	    } else if (actualToken.isString()) {
		val = "\"" + actualToken.value() + "\"";
	    } else {
		val = "'" + actualToken.value() + "'";
	    }
	    nextToken();
	    if (actualToken.isDelimiter() &&
		actualToken.equalValue("(")) {
		nextToken();
		obj = new LFunction(val,parseArguments(new ArrayList()));
		if (actualToken.isDelimiter() &&
		    actualToken.equalValue(")")) {
		    nextToken();
		} else {
		    errorDetected("')' expected");
		}
	    } else {
		if (Character.isUpperCase(val.charAt(0))) {
		    obj = new LVariable(val);
		} else {
		    obj = new LConstant(val);
		}
	    }
	} else if (actualToken.isFloat() ||
		   actualToken.isInteger() ||
		   actualToken.isCharacter()) {
	    String val;
	    if (actualToken.isCharacter()) {
		val = "'" + actualToken.value() + "'";
	    } else {
		val = actualToken.value();
	    }
	    obj = new LConstant(val);
	    nextToken();
	} else {
	    errorDetected("Identifier, string, float, integer, or character expected");
	}
	return obj;
    }

    public ArrayList parseAntecedence (ArrayList v) throws ParserErrorException
    {
	LPredicate pred;
	pred = parsePredicate();
	v.add(pred);
	if (actualToken.isDelimiter() && 
	    actualToken.equalValue(",")) {
	    v = parseAntecedenceRest(v);
	} 
	return v;
    }

    public ArrayList parseAntecedenceRest (ArrayList v) throws ParserErrorException
    {
	while (true) {
	    LPredicate pred;
	    if (actualToken.isDelimiter() && 
		actualToken.equalValue(",")) {
		nextToken();
		pred = parsePredicate();
		v.add(pred);
	    } else {
		return v;
	    } 
        } 
    }

}
