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
 * GenericToken: Implements an abstract token class
 *
 * @version 1.0, DATE: 19.08.1998
 * @author Franz Wotawa
 *
 * Tokens are used by parsers to check grammar and
 * are created by scanner objects. The current
 * implementation provides several token classes
 * useable for writing compilers for almost every
 * language.
 *
 * 19.8.1998: First implementation
 */

package theoremprover;

import java.lang.*;

public class GenericToken extends Object
{
  protected String value;
  protected int position;

  // Accessing structure...

  public String value ()
  {
    return value;
  }

  public int position ()
  {
    return position;
  }

  public void setValue(String val)
  {
    setValue(val, 0);
  }

  public void setValue(String val, int i)
  {
    value = val;
    position = i;
  }


  // Testing...

  public boolean isInteger ()
  {
    return false;
  }

  public boolean isFloat ()
  {
    return false;
  }

  public boolean isCharacter ()
  {
    return false;
  }

  public boolean isString ()
  {
    return false;
  }

  public boolean isKeyword ()
  {
    return false;
  }

  public boolean isDelimiter ()
  {
    return false;
  }

  public boolean isIdentifier ()
  {
    return false;
  }

  public boolean isEOI ()
  {
    return false;
  }

  public boolean isErrorToken ()
  {
    return false;
  }

  public boolean equalValue (String val)
  {
    return value.equals(val);
  }

  // Printing...

  public String print ()
  {
    return "token(" + value + ")";
  }


  // Constructors...

  public GenericToken ()
  {
    value = "";
    position = 0;
  }

  public GenericToken (String val)
  {
    value = val;
    position = 0;
  }

  public GenericToken (String val, int i)
  {
    value = val;
    position = i;
  }

}

