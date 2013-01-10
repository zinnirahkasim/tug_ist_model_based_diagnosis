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
 * GenericScanner: Implementation of a generic scanner
 *
 * @version 0.1, DATE: 17.08.1998
 * @author Franz Wotawa
 *
 * The GenericScanner class implements a scanner used
 * for converting strings into tokens.
 *
 * V0.1: Creating the basic functionality (17.08.1998)
 */

package theoremprover;

import java.lang.*;    // Java language classes 
import java.awt.*;     // User Interface components
import java.io.*;      // IO specific classes (File, Stream,..)
import java.awt.event.*;   // Event handling

import theoremprover.*;


public class GenericScanner extends Object
{

  // Instance variables ...

  protected String source;
  protected char nextChar;
  protected int position;
  protected StringBuffer charBuffer;

  // Instance creation and initialization

  public GenericScanner()
  {
    super();
    initialize();
  }

  public GenericScanner(String str)
  {
    super();
    initialize();
    source = str;
    getNextChar();
  }

  public static GenericScanner parseFile (String file)
  {
    GenericScanner gs;
 
    FileInputStream freader;
    try{
      freader = new FileInputStream(file);}
    catch (FileNotFoundException e) {
      return null;
    }

    try {
      byte[] chars = new byte[freader.available()];
      freader.read(chars);
      freader.close();
      String str = new String(chars);
      return new GenericScanner(str);}
    catch (IOException e){
      return null;
    }
  }

  public void initialize()
  {
    source = "";
    nextChar = endOfInput();
    position = 0;
    charBuffer = new StringBuffer();
  }

  // Accessing

  public String source()
  {
    return source;
  }

  public int position()
  {
    return position;
  }


  // Private methods

  public char endOfInput ()
  {
    return 0;
  }

  public boolean isEndOfInput ()
  {
    return (nextChar == endOfInput());
  }

  public boolean isDelimiter (char c)
  {
    return false; // This method should be overwritten by my subclasses..
  }

  public String bufferValue ()
  {
    return charBuffer.toString();
  }

  // Public methods

  public void initSource (String str)
  {
    source = str;
    position = 0;
    getNextChar();
  }

  public char getNextChar ()
  {
      if (position < source.length()) {
	  nextChar = source.charAt(position);
	  charBuffer.append(nextChar);
	  position = position + 1; }
      else {
	  charBuffer.append('\u0000');
	  nextChar = endOfInput(); }
      return nextChar;
  }

  public GenericToken scanSource(String str)
  {
      initSource(str);
      return scanToken();
  }

  public void overReadSpaces()
  {
    if (isEndOfInput()) { return; }
    while (Character.isWhitespace(nextChar)) {
      getNextChar();
      if (isEndOfInput()) { return; }}
    charBuffer = new StringBuffer((new Character(nextChar)).toString());
  }

  public GenericToken scanToken ()
  {
    GenericToken actualToken;
    String str;
    
    overReadSpaces();

    // Ignore comments

    if (nextChar == '%') {
	while ((nextChar != '\n') && (nextChar != '\r')) {
	    getNextChar();
	    if (isEndOfInput()) { return new EOIToken("EOI",position); }
	}
	charBuffer = new StringBuffer();
	getNextChar();
	return scanToken();
    }

    // Classify tokens

    int oldPos = position;

    if (isEndOfInput()) {
      return new EOIToken("EOI",oldPos); }

    if (Character.isDigit(nextChar)) {
      return scanNumber(); }

    if (Character.isLetter(nextChar) || (nextChar == '!')) {
      return scanIdOrKeyword(); }

    if ((nextChar == ',') || (nextChar == '.') ||
	(nextChar == ')') || (nextChar == '(')) {
      str = bufferValue();
      actualToken = new DelimiterToken(str,oldPos);
      charBuffer = new StringBuffer();
      getNextChar();
      return actualToken;}

    if (nextChar == ':') {
      getNextChar();
      if (nextChar == '-') {
	actualToken = new DelimiterToken(bufferValue(),oldPos);
	charBuffer = new StringBuffer();
	getNextChar();
      }
      else {
	// Lexical Error detected
	actualToken = new ErrorToken(":- expected", position);
	charBuffer = new StringBuffer();
	getNextChar();
      }
      return actualToken; }

    if (nextChar == '-') {
      getNextChar();
      if (nextChar == '>') {
	actualToken = new DelimiterToken(bufferValue(),oldPos);
	charBuffer = new StringBuffer();
	getNextChar();
      }
      else {
	if (Character.isDigit(nextChar)) {
	  actualToken = scanNumber();
	} else {
	  // Lexical Error detected
	  charBuffer = new StringBuffer();
	  getNextChar();
	  actualToken = new ErrorToken("-> or number expected", position);
	}
      }
      return actualToken; }

    if (nextChar == '"') {
      return scanString(); }
    
    if (nextChar == '\'') {
      return scanCharacter(); }
    
    // Lexical Error detected
    charBuffer = new StringBuffer();
    getNextChar();
    return new ErrorToken("No token recognized", position);
  }

  public GenericToken scanNumber ()
  {
    boolean floatFlag = false;
    String str;

    getNextChar();
    while (Character.isDigit(nextChar)) {
      getNextChar(); }
    if (nextChar == '.') {
      floatFlag = true;
      getNextChar();
      while (Character.isDigit(nextChar)) {
	getNextChar(); } }
    str = bufferValue().substring(0,bufferValue().length() - 1);
    charBuffer = new StringBuffer((new Character(nextChar)).toString());
    if (floatFlag) {
      return new FloatToken(str, position);
    } else {
      return new IntegerToken(str, position);
    }
  }

  public GenericToken scanIdOrKeyword ()
  {
    String str; 

    getNextChar();
    if (nextChar == '!') getNextChar();
    while (Character.isLetterOrDigit(nextChar) || (nextChar == '_')) {
      getNextChar(); }
    str = bufferValue().substring(0,bufferValue().length() - 1);
    charBuffer = new StringBuffer((new Character(nextChar)).toString());
    return new IdentifierToken(str, position);
  }

  public GenericToken scanString ()
  {
    String str;

    boolean flag = true;
    while (flag ) {
	getNextChar();
	if (nextChar == '\"') {
	    getNextChar();
	    if (nextChar != '\"') {
		flag = false; 
	    }
	}
	if (isEndOfInput()) {
	    return new ErrorToken("eoi detected while scanning string", position);
	}
    }
  
    str = bufferValue().substring(1,bufferValue().length() - 2);
    charBuffer = new StringBuffer((new Character(nextChar)).toString());
    return new StringToken(str, position);
  }

  public GenericToken scanCharacter ()
  {
    getNextChar();
    char ch = nextChar;
    getNextChar();
    if (nextChar == '\'') {
      charBuffer = new StringBuffer();
      getNextChar();
      return new CharacterToken(new Character(ch).toString(),position);
    } else {
      // Lexical error detected
      charBuffer = new StringBuffer();
      getNextChar();
      return new ErrorToken("character token expected",position);
    }
  }

}

