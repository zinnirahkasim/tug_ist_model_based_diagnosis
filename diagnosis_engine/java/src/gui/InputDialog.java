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
 * InputDialog: Provides a dialog with one textual input field
 *
 * @version 0.1, DATE: 23.11.1998
 * @author Franz Wotawa
 *
 * This class...
 *
 * V0.1: Creating the dialog (23.11.1998)
 *
 */

package gui;

import java.lang.*;    // Java language classes 
import java.awt.*;     // User Interface components
import java.io.*;      // IO specific classes (File, Stream,..)
import java.awt.event.*;   // Event handling

public class InputDialog extends Dialog
{
  // Instance variables ...

  protected TextField textHolder;
  protected String result = null;

  // Instance creation

  public InputDialog(Frame dw, String label)
  {
    super(dw, "Dialog", true);

    Panel p1 = new Panel();
    p1.add(new Label(label));

    Panel p2 = new Panel();
    textHolder = new TextField(label.length());
    p2.add(textHolder);

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

    /*    textHolder.requestFocus();
    textHolder.getCursor();
    textHolder.selectAll(); */

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

  public void okPressed()
  {
    result = textHolder.getText();
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

  public void setInitialValue(String txt)
  {
    textHolder.setText(txt);
  }

}

