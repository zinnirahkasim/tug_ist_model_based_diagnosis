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
 * ErrorToken: Implements an error token class
 *
 * @version 1.0, DATE: 26.11.1998
 * @author Franz Wotawa
 *
 * 26.11.1998: First implementation
 */

package theoremprover;

public class ErrorToken extends GenericToken
{
  public ErrorToken (String str, int i)
  {
    super(str,i);
  }

  // Testing...

  public boolean isErrorToken ()
  {
    return true;
  }

  // Printing...

  public String print ()
  {
    return "error(" + value + ")";
  }


}

