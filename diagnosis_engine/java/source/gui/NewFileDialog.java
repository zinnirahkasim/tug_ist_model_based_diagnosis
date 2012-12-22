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
 * NewFileDialog: Provides a dialog for loading and saving files
 *
 * @version 0.1, DATE: 22.04.1999
 * @author Franz Wotawa
 *
 * This class...
 *
 * V0.1: Creating the dialog (22.04.1999)
 *
 */

package gui;

import java.lang.*;    // Java language classes 
import java.awt.*;     // User Interface components
import java.io.*;      // IO specific classes (File, Stream,..)
import java.awt.event.*;   // Event handling

public class NewFileDialog extends Dialog
{
  // Class variables ..

  public static final int LOAD = 0;
  public static final int SAVE = 1;

  // Instance variables ..

  protected TextField directoryField;
  protected TextField fileField;
  protected TextField filterField;
  protected List fileList;

  protected String directory = null;
  protected String file = null;
  protected int mode = -1;

  // Instance creation

  public NewFileDialog(Frame dw, String title)
  {
    super(dw, title, true);

    GridBagLayout gridbag = new GridBagLayout();
    GridBagConstraints constr = new GridBagConstraints();
    setLayout(gridbag);

    directoryField = new TextField(40);
    directoryField.addTextListener(new TextListener()
			   { public void textValueChanged(TextEvent e)
			     {directoryChanged();}});
 
    Panel p1 = new Panel();
    p1.add(new Label("Directory"));
    p1.add(directoryField);
    constr.fill = GridBagConstraints.NONE;
    constr.gridwidth = GridBagConstraints.REMAINDER;
    gridbag.setConstraints(p1,constr);
    add(p1);

    fileList = new List(4);
    fileList.addItemListener(new ItemListener()
		      { public void itemStateChanged(ItemEvent e) {
			itemSelected();}});
    constr.fill = GridBagConstraints.BOTH;
    constr.gridwidth = GridBagConstraints.REMAINDER;
    constr.weighty = 1.0;
    gridbag.setConstraints(fileList,constr);
    add(fileList);

    Panel psum = new Panel();

    filterField = new TextField(8);
    Panel p4 = new Panel();
    p4.add(new Label("Filter"));
    p4.add(filterField);
    psum.add(p4);

    fileField = new TextField(20);
    Panel p3 = new Panel();
    p3.add(new Label("File"));
    p3.add(fileField);
    psum.add(p3);
    constr.fill = GridBagConstraints.NONE;
    constr.gridwidth = GridBagConstraints.REMAINDER;
    constr.weighty = 0.0;
    gridbag.setConstraints(psum,constr);
    add(psum);

    // Create buttons

    Panel p5 = new Panel();
    Button okButton = new Button("Ok");
    okButton.addActionListener(new ActionListener()
			       { public void actionPerformed(ActionEvent e)
				   { okPressed();}});
    Button cancelButton = new Button("Cancel");
    cancelButton.addActionListener(new ActionListener()
				   { public void 
				       actionPerformed(ActionEvent e)
				       { cancelPressed();}});
    p5.add(okButton);
    p5.add(cancelButton);
    constr.fill = GridBagConstraints.NONE;
    constr.gridwidth = GridBagConstraints.REMAINDER;
    gridbag.setConstraints(p5,constr);
    add(p5);

    // Initialize this dialog to its preferred size

    setSize(580,200);
    pack();
  }

  // Action handling

  public void okPressed()
  {
    directory = directoryField.getText();
    file = fileField.getText();
    closeDialog();
  }

  public void cancelPressed()
  {
    directory = null;
    file = null;
    closeDialog();
  }

  public void closeDialog()
  {
    setVisible(false);
    dispose();
  }

  public void directoryChanged() {
    File dir = new File(directoryField.getText());
    if (dir.isDirectory()) {
      loadDirectory(dir);
    }
  }

  public void loadDirectory(File dir) {
    String[] dirlist = dir.list();
    int i;
    if (fileList.getItemCount()>0) {
      fileList.removeAll();
    }
    fileList.add("..");
    fileList.addItemListener(new ItemListener()
		    { public void itemStateChanged(ItemEvent e) {
			itemSelected();}});
    for(i=0;i<dirlist.length;i++) {
      fileList.add(dirlist[i]);
    }
  }

  public void itemSelected() {
    if (fileList.getSelectedIndex() >= 0) {
      String item = fileList.getSelectedItem();
      fileList.deselect(fileList.getSelectedIndex());
      File fileOrDir;
      File newDir;
      if (item.equals("..")) {
	fileOrDir = new File(directoryField.getText());
	String dirStr = fileOrDir.getParent();
	if (dirStr != null) {
	  newDir = new File(dirStr);
	  directoryField.setText(newDir.getAbsolutePath());
	  directoryChanged();
	}
      } else {
	fileOrDir = new File(directoryField.getText(),item);
	if (fileOrDir.isDirectory()) {
	  directoryField.setText(fileOrDir.getAbsolutePath());
	  directoryChanged();
	} else {
	  fileField.setText(item);
	}
      }	 
    }
  }

  // Accessing

  public synchronized void setDirectory(String dir) {
    File fdir = new File(dir);
    if (!fdir.isDirectory()) {
      fdir = new File("");
    }
    directoryField.setText(fdir.getAbsolutePath());
    loadDirectory(fdir);
  }

  public String getDirectory() {
    return directory;
  }

  public synchronized void setFile(String file) {
    fileField.setText(file);
  }

  public String getFile() {
    return file;
  }

  public synchronized void setFilenameFilter(FilenameFilter filter) {
  }

  public FilenameFilter getFilenameFilter() {
    return null;
  }

  public synchronized void setMode(int mode) {
    this.mode = mode;
  }

  public int getMode(int mode) {
    return mode;
  }

}

