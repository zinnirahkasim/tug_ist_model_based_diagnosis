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


/*
 * SelectionDialog: Provides a dialog with one selection list
 *
 * @version 0.1, DATE: 20.05.1999
 * @author Franz Wotawa
 *
 */

package gui;

import java.lang.*;    // Java language classes 
import java.awt.*;     // User Interface components
import java.io.*;      // IO specific classes (File, Stream,..)
import java.awt.event.*;   // Event handling
import java.util.*;

public class SelectionDialog extends Dialog
{
  // Instance variables ...

  protected java.awt.List selectionList;
  protected String result = null;

  // Instance creation


  public SelectionDialog(Frame dw, String label, Iterator e)
  {
    this(dw,label);
    setInitialValue(e);
  }
   
  public SelectionDialog(Frame dw, String label)
  {
    super(dw, "Dialog", true);

    Panel p1 = new Panel();
    p1.add(new Label(label));

    Panel p2 = new Panel();
    selectionList = new java.awt.List(5,false);
    selectionList.addItemListener(new ItemListener()
		    { public void itemStateChanged(ItemEvent e) {
		      objectSelected(); }});
    p2.add(selectionList);

    // Create buttons

    Panel p3 = new Panel();
    Button okButton = new Button("Ok");
    okButton.addActionListener(new ActionListener()
			       { public void actionPerformed(ActionEvent e)
				   { okPressed();}});
    Button cancelButton = new Button("Cancel");
    cancelButton.addActionListener(new ActionListener()
				   { public void 
				       actionPerformed(ActionEvent e)
				       { cancelPressed();}});
    p3.add(okButton);
    p3.add(cancelButton);

    add("North",p1);
    add("Center",p2);
    add("South",p3);

    // Initialize this dialog to its preferred size
    pack();
    setLocation(defaultLocation());
  }

  public Point defaultLocation()
  {
    Point newLoc;
    Toolkit tk = Toolkit.getDefaultToolkit();
    Dimension screenSize = tk.getScreenSize();
    int w = this.getBounds().width;
    int h = this.getBounds().height;
    if ((w < screenSize.width) && (h < screenSize.height)) {
      newLoc = new Point((screenSize.width-w)/2,
		  (screenSize.height-h)/2);
    } else {
      newLoc = new Point(0,0);
    }
    return newLoc;
  }

  // Action handling

  public void objectSelected() {
    result = selectionList.getSelectedItem();
  }

  public void okPressed()
  {
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

  public String result()
  {
    return result;
  }

  public void select(String str) {
    String [] items = selectionList.getItems();
    for (int i=0; i<items.length; i++) {
      if (items[i].equals(str)) {
	selectionList.select(i);
	return;
      }
    }
  }

  public void setInitialValue(Iterator e)
  {
    if (selectionList.getItemCount() > 0) {
      selectionList.removeAll();
    }
    while (e.hasNext()) {
      selectionList.add((e.next()).toString());
    }    
  }

}

