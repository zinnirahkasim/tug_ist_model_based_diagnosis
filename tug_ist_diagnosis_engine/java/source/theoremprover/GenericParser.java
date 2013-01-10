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
 * GenericParser: Implementation of a generic parser
 *
 * @version 0.1, DATE: 03.12.1998
 * @author Franz Wotawa
 *
 * The GenericParser class implements a parser used
 * for parsing strings. This class only provides the
 * basic methods. The concrete implementation must
 * be given by my subclasses.
 *
 * V0.1: Creating the basic functionality (03.12.1998,28.12.1998)
 * V0.2: Introducing exception handling (29.12.1998)
 */

package theoremprover;

import java.lang.*;    // Java language classes 
import java.awt.*;     // User Interface components
import java.io.*;      // IO specific classes (File, Stream,..)
import java.awt.event.*;   // Event handling

import theoremprover.*;


public class GenericParser extends Object
{

    // Instance variables ...

    protected GenericScanner scanner;
    protected String source;
    protected GenericToken actualToken;
    protected Object result;
    protected String errorMessage;

    // Instance creation and initialization

    GenericParser()
    {
	errorMessage = null;
	scanner = defaultScanner();
	source = "";
	actualToken = null;
	result = null;
    }

    GenericParser(String str)
    {
	errorMessage = null;
	scanner = defaultScanner();
	source = str;
	actualToken = null;
	result = null;
    }

    public GenericScanner defaultScanner()
    {
	return new GenericScanner();
    }

    // Accessing

    public Object result()
    {
	return result;
    }

    public String errorMessage()
    {
	return errorMessage;
    }

    // Parsing

    public boolean parse(String str)
    // Returns true if the parser accepts the given string and
    // false otherwise
    {
	boolean noerror = true;
	errorMessage = null;
	result = defaultResult();
	source = str;
	actualToken = scanner.scanSource(str);
	try {
	    parse();
	} catch (ParserErrorException e) {
	    errorMessage = e.getMessage();
	    noerror = false;
	}
	return noerror;
    }

    public Object defaultResult()
    {
	return null;
    }

    public boolean parseFile (String file)
    {
	FileInputStream freader;
	String str;
	try{
	    freader = new FileInputStream(file);
	} catch (FileNotFoundException e) {
	    return false;
	}
	try {
	    byte[] chars = new byte[freader.available()];
	    freader.read(chars);
	    freader.close();
	    str = new String(chars);
	} catch (IOException e){
	    return false;
	}
	return parse(str);
    }


    // Private parsing

    public void parse () throws ParserErrorException {}

    public void nextToken () throws ParserErrorException
    {
	if (actualToken.isEOI()) {
	    // Do nothing after dedecting the end of the input
	} else {
	    actualToken = scanner.scanToken();
	}

	if (actualToken.isErrorToken()) {
	    throw new ParserErrorException(
			     "Lexical Error at position " + 
			     Integer.toString(actualToken.position()));
	}
    }

    public void errorDetected () throws ParserErrorException
    {
	throw new ParserErrorException(
			   "Parser Error [ " +
			   actualToken.value() +
			   " ] at position " +
			   Integer.toString(actualToken.position()));
    }

    public void errorDetected (String str) throws ParserErrorException
    {
	throw new ParserErrorException(
			   "Parser Error [ " +
			   actualToken.value() +
			   " ] at position " +
			   Integer.toString(actualToken.position()) +
			   " [ " + str + " ] ");
    }

}
