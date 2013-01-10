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
 * NumberDialog: Provides a dialog for numbers
 *
 * @version 0.1, DATE: 21.05.1999
 * @author Franz Wotawa
 *
 */

package gui;

import java.lang.*;    // Java language classes 
import java.awt.*;     // User Interface components
import java.io.*;      // IO specific classes (File, Stream,..)
import java.awt.event.*;   // Event handling
import java.util.*;

public class NumberDialog extends InputDialog
{
  // Instance variables ...

  protected boolean integer = false;

  // Instance creation

  public NumberDialog(Frame dw, String label)
  {
    super(dw,label);
  }

  // Action handling


  public void okPressed()
  {
    result = textHolder.getText();

    if (integer) {
      Integer i;
      try {
	i = new Integer(result);
      } catch (Exception e) {
	return;
      }
      result = i.toString();
    } else {
      Double d;
      try {
	d = Double.valueOf(result);
      } catch (Exception e) {
	return;
      }
      result = d.toString();
    }
    closeDialog();
  }

  public void cancelPressed()
  {
    result = null;
    closeDialog();
  }

  public void closeDialog()
  {
    setVisible(false);
    dispose();
  }

  // Accessing

  public void integer(boolean b)
  {
    integer = b;
  }

  public Number resultAsNumber() {
    if (result != null) {
      if (integer) {
	Integer i;
	try {
	  i = new Integer(result);
	} catch (Exception e) {
	  return null;
	}
	return i;
      } else {
	Double d;
	try {
	  d = Double.valueOf(result);
	} catch (Exception e) {
	  return null;
	}
	return d;
      }    
    }
    return null;
  }
}

