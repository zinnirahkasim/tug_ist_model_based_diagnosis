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
 * WarnDialog: Provides a dialog with one textual field
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

public class WarnDialog extends Dialog
{

  // Instance creation

  public WarnDialog(Frame dw, String text)
  {
    super(dw, "Info", true);

    Panel p1 = new Panel();
    p1.add(new Label(text));

    // Create buttons

    Panel p2 = new Panel();
    Button okButton = new Button("Ok");
    okButton.addActionListener(new ActionListener()
			       { public void actionPerformed(ActionEvent e)
				   { okPressed();}});
    p2.add(okButton);

    add("Center",p1);
    add("South",p2);

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

  public void okPressed()
  {
    closeDialog();
  }

  public void closeDialog()
  {
    setVisible(false);
    dispose();
  }

}

