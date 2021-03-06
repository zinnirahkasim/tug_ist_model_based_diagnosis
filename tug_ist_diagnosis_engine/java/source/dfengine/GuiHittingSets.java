

package dfengine;

import java.util.*;
import java.text.*;
import java.io.*;
import javax.swing.*;
import javax.swing.text.*;
import javax.swing.border.*;
import java.awt.*;
import java.awt.event.*;


import hittingsetalg.*;
import theoremprover.*;
import utils.*;

public class GuiHittingSets  
    implements ActionListener, ItemListener, 
    WindowListener, FocusListener {

    private JFrame frame;

    private JTabbedPane tabbedPane;

    private JLabel abLabel;
    private JLabel nabLabel;
    private JLabel dfLabel;
    private JTextArea propText;
    private JTextArea sdText;
    private JTextArea obsText;
    private JSpinner explSizeSpinner;
    private JTextField numExplText;
    private JCheckBox useFaultModelsCB;
    private JCheckBox depFaultsCB;
    private JCheckBox useProbabilitiesCB;
    private JTextField assABText;
    private JTextField assNABText;
    private JTextField assDFText;
    private JTextField negPrefixText;
    private JButton checkConsBtn;
    private JButton computeMinHSBtn;
    private JButton computeMoreBtn;
    private JButton computeMinDiagEnvBtn;
    //private JButton computeMergedDEBtn;
    private TitledBorder resultsBorder;
    private JPanel resultsPanel;
    private JTextArea results1Text;
    private JTextArea results2Text;
    private JButton clearPropBtn;
    private JButton clearSdBtn;
    private JButton clearObsBtn;
    private JButton commentBtn;
    private JButton uncommentBtn;
    private JTextField searchText;
    private JButton searchFirstBtn;
    private JButton searchNextBtn;
    private JLabel searchResultLabel;
    private JTextArea fdgText;
    private JTextArea sddText;
    private JPanel dfSettingsPanel;
    private JCheckBox betaEnvCB;
    private JCheckBox mergeDEsCB;
    private JCheckBox discardOrderPermsCB;
    private JSpinner maxDFChainSpinner;

    private JMenuItem savePropAsMenu;
    private JMenuItem saveSdAsMenu;
    private JMenuItem saveObsAsMenu;
    private JMenuItem saveFdgAsMenu;
    private JMenuItem saveSddAsMenu;

    private JMenuItem savePropMenu;
    private JMenuItem saveSdMenu;
    private JMenuItem saveObsMenu;
    private JMenuItem saveFdgMenu;
    private JMenuItem saveSddMenu;
    private JMenuItem saveAllMenu;

    private JMenuItem openPropMenu;
    private JMenuItem openSdMenu;
    private JMenuItem openObsMenu;
    private JMenuItem openFdgMenu;
    private JMenuItem openSddMenu;
    private JMenuItem reloadPropMenu;
    private JMenuItem reloadSdMenu;
    private JMenuItem reloadObsMenu;
    private JMenuItem reloadFdgMenu;
    private JMenuItem reloadSddMenu;
    private JMenuItem reloadAllMenu;
    private JMenuItem commentMenu;
    private JMenuItem uncommentMenu;

    private File lastDir;
    private File propFile;
    private File sdFile;
    private File obsFile;
    private File fdgFile;
    private File sddFile;

    private boolean supportDepFaults = false;

    private Object focusedComponent;

    private boolean mergeDEs = true;

    private MinHittingSetsFM hsFM = null;
    private MinHittingSets hs = null;

    private final static String[] COMMENT_PREFIXES = {"#", "*"};
    private final static char AUTO_COMMENT_PREFIX = '#';

    private final static String FDG_EDGE_STR = "=>";

    private final static String ASS_IF = "IF";

    private final static int TAB_LOG_MODEL = 0;
    private final static int TAB_FDG = 1;

    private final static String SAVE_PROP_MENU_TEXT = "Save Propositions";
    private final static String SAVE_SD_MENU_TEXT = "Save System Description";
    private final static String SAVE_OBS_MENU_TEXT = "Save Observations";
    private final static String SAVE_FDG_MENU_TEXT = "Save FDG";
    private final static String SAVE_SDD_MENU_TEXT = "Save System Dep. Description";

    private final static String RELOAD_PROP_MENU_TEXT = "Reload Propositions";
    private final static String RELOAD_SD_MENU_TEXT = "Reload System Description";
    private final static String RELOAD_OBS_MENU_TEXT = "Reload Observations";
    private final static String RELOAD_FDG_MENU_TEXT = "Reload FDG";
    private final static String RELOAD_SDD_MENU_TEXT = "Reload System Dep. Description";

    private final static String TITLE_RESULTS = "Results: ";

    private final static String PR_PROP_FILE = "proposition_file";
    private final static String PR_SD_FILE = "system_description_file";
    private final static String PR_OBS_FILE = "observations_file";
    private final static String PR_FDG_FILE = "fdg_file";
    private final static String PR_SDD_FILE = "system_dep_description_file";

    private final static String FILE_SETTINGS = ".GuiHittingSets";

    // command line settings
    private final static String CMD_SUPPORT_DEP_FAULTS  = "-df";
    private final static String CMD_MERGE_DES = "merge_des";
    private final static String OPT_YES = "yes";
    private final static String OPT_NO = "no";


    public static void main(final String[] args) {
 
        SwingUtilities.invokeLater(new Runnable() {
                public void run() {
                    createAndShowGUI(args);
                }
            });
        
    }

    private final static void createAndShowGUI(final String[] args) {
        
        // set look-and-feel
        try {
            UIManager.setLookAndFeel(UIManager.getSystemLookAndFeelClassName());
        } catch (ClassNotFoundException e) {
            System.out.println(e.toString());
        } catch (InstantiationException e) {
            System.out.println(e.toString());
        } catch (IllegalAccessException e) {
            System.out.println(e.toString());
        } catch (UnsupportedLookAndFeelException e) {
            System.out.println(e.toString());
        }

        //Make sure we have nice window decorations.
        JFrame.setDefaultLookAndFeelDecorated(true);
        
        //Create and set up the window.
        JFrame frame = new JFrame("GuiHittingSets - a front-end to Reiter's HS algorithm");
        frame.setDefaultCloseOperation(JFrame.DO_NOTHING_ON_CLOSE);

        // create instance of this class
        GuiHittingSets app = new GuiHittingSets(frame, args);
        app.fillFrame();
        try {
            app.readSettings();
        } catch (IOException e) {
            System.out.println("On reading settings from file: " + e);
        }
        app.updateMenuStatus();

        //Display the window.
        frame.pack();
        frame.setVisible(true);
    }

    private GuiHittingSets(JFrame frame, String[] args) {
        this.frame = frame;
        frame.addWindowListener(this);

        parseCmdLine(args);
    }

    private void parseCmdLine(String[] args) {
        for (int i = 0; i < args.length; ++i) {
            
            String cmd = args[i];
            if (cmd.startsWith(CMD_MERGE_DES) && cmd.endsWith(OPT_NO)) mergeDEs = false;
            else if (cmd.equals(CMD_SUPPORT_DEP_FAULTS)) {
                supportDepFaults = true;
            }

        }
    }

    private void setTextAreaAttributes(JTextArea textArea) {
        textArea.setFont(new Font("Monospaced", Font.PLAIN, 14));
        textArea.setTabSize(4);
    }

    private void updateMenuStatus() {
        if (propFile != null) {
            reloadPropMenu.setEnabled(true);
            reloadPropMenu.setText(RELOAD_PROP_MENU_TEXT + " (" + propFile.getName() + ")");

            savePropMenu.setEnabled(true);
            savePropMenu.setText(SAVE_PROP_MENU_TEXT + " (" + propFile.getName() + ")");
        }
        if (sdFile != null) {
            reloadSdMenu.setEnabled(true);
            reloadSdMenu.setText(RELOAD_SD_MENU_TEXT + " (" + sdFile.getName() + ")");

            saveSdMenu.setEnabled(true);
            saveSdMenu.setText(SAVE_SD_MENU_TEXT + " (" + sdFile.getName() + ")");
        }
        if (obsFile != null) {
            reloadObsMenu.setEnabled(true);
            reloadObsMenu.setText(RELOAD_OBS_MENU_TEXT + " (" + obsFile.getName() + ")");

            saveObsMenu.setEnabled(true);
            saveObsMenu.setText(SAVE_OBS_MENU_TEXT + " (" + obsFile.getName() + ")");
        }
        if (fdgFile != null) {
            reloadFdgMenu.setEnabled(true);
            reloadFdgMenu.setText(RELOAD_FDG_MENU_TEXT + " (" + fdgFile.getName() + ")");

            saveFdgMenu.setEnabled(true);
            saveFdgMenu.setText(SAVE_FDG_MENU_TEXT + " (" + fdgFile.getName() + ")");
        }
        if (sddFile != null) {
            reloadSddMenu.setEnabled(true);
            reloadSddMenu.setText(RELOAD_SDD_MENU_TEXT + " (" + sddFile.getName() + ")");

            saveSddMenu.setEnabled(true);
            saveSddMenu.setText(SAVE_SDD_MENU_TEXT + " (" + sddFile.getName() + ")");
        }

        if ((propFile != null) && (sdFile != null) && (obsFile != null)) {

            if (!supportDepFaults
                ||
                ((fdgFile != null) || (fdgText.getText().length() == 0))
                && ((sddFile != null) || (sddText.getText().length() == 0))
                ) {
                
                reloadAllMenu.setEnabled(true);
                saveAllMenu.setEnabled(true);
            }
        }
    }

    private void createMenu() {
        JMenuBar menuBar = new JMenuBar();

        JMenu fileMenu = new JMenu("File");
        fileMenu.setMnemonic(KeyEvent.VK_F);

        openPropMenu = new JMenuItem("Open Proposition File...");
        openPropMenu.addActionListener(this);

        openSdMenu = new JMenuItem("Open System Description File...");
        openSdMenu.addActionListener(this);
        
        openObsMenu = new JMenuItem("Open Observations File...");
        openObsMenu.addActionListener(this);

        openFdgMenu = new JMenuItem("Open FDG...");
        openFdgMenu.addActionListener(this);

        openSddMenu = new JMenuItem("Open System Dep. Description File...");
        openSddMenu.addActionListener(this);

        reloadPropMenu = new JMenuItem(RELOAD_PROP_MENU_TEXT);
        reloadPropMenu.setEnabled(false);
        reloadPropMenu.addActionListener(this);
        reloadPropMenu.setAccelerator(KeyStroke
                                     .getKeyStroke('P', InputEvent.CTRL_MASK | InputEvent.ALT_MASK));

        reloadSdMenu = new JMenuItem(RELOAD_SD_MENU_TEXT);
        reloadSdMenu.setEnabled(false);
        reloadSdMenu.addActionListener(this);
        reloadSdMenu.setAccelerator(KeyStroke
                                     .getKeyStroke('Y', InputEvent.CTRL_MASK | InputEvent.ALT_MASK));
        
        reloadObsMenu = new JMenuItem(RELOAD_OBS_MENU_TEXT);
        reloadObsMenu.setEnabled(false);
        reloadObsMenu.addActionListener(this);
        reloadObsMenu.setAccelerator(KeyStroke
                                     .getKeyStroke('O', InputEvent.CTRL_MASK | InputEvent.ALT_MASK));

        reloadFdgMenu = new JMenuItem(RELOAD_FDG_MENU_TEXT);
        reloadFdgMenu.setEnabled(false);
        reloadFdgMenu.addActionListener(this);
        reloadFdgMenu.setAccelerator(KeyStroke
                                     .getKeyStroke('F', InputEvent.CTRL_MASK | InputEvent.ALT_MASK));

        reloadSddMenu = new JMenuItem(RELOAD_SDD_MENU_TEXT);
        reloadSddMenu.setEnabled(false);
        reloadSddMenu.addActionListener(this);
        reloadSddMenu.setAccelerator(KeyStroke
                                     .getKeyStroke('D', InputEvent.CTRL_MASK | InputEvent.ALT_MASK));

        reloadAllMenu = new JMenuItem("Reload All");
        reloadAllMenu.setEnabled(false);
        reloadAllMenu.addActionListener(this);
        reloadAllMenu.setMnemonic(KeyEvent.VK_R);
        reloadAllMenu.setAccelerator(KeyStroke
                                     .getKeyStroke('R', InputEvent.CTRL_MASK));

        savePropAsMenu = new JMenuItem("Save Propositions as...");
        savePropAsMenu.addActionListener(this);

        saveSdAsMenu = new JMenuItem("Save System Description as...");
        saveSdAsMenu.addActionListener(this);

        saveObsAsMenu = new JMenuItem("Save Observations as...");
        saveObsAsMenu.addActionListener(this);

        saveFdgAsMenu = new JMenuItem("Safe FDG as...");
        saveFdgAsMenu.addActionListener(this);

        saveSddAsMenu = new JMenuItem("Save System Dep. Description as...");
        saveSddAsMenu.addActionListener(this);

        savePropMenu = new JMenuItem(SAVE_PROP_MENU_TEXT);
        savePropMenu.setEnabled(false);
        savePropMenu.setMnemonic(KeyEvent.VK_P);
        savePropMenu.setAccelerator(KeyStroke
                                    .getKeyStroke('P', InputEvent.CTRL_MASK));
        savePropMenu.addActionListener(this);

        saveFdgMenu = new JMenuItem(SAVE_FDG_MENU_TEXT);
        saveFdgMenu.setEnabled(false);
        saveFdgMenu.setMnemonic(KeyEvent.VK_F);
        saveFdgMenu.setAccelerator(KeyStroke
                                   .getKeyStroke('F', InputEvent.CTRL_MASK));
        saveFdgMenu.addActionListener(this);

        saveSdMenu = new JMenuItem(SAVE_SD_MENU_TEXT);
        saveSdMenu.setEnabled(false);
        saveSdMenu.setMnemonic(KeyEvent.VK_Y);
        saveSdMenu.setAccelerator(KeyStroke
                                    .getKeyStroke('Y', InputEvent.CTRL_MASK));
        saveSdMenu.addActionListener(this);

        saveSddMenu = new JMenuItem(SAVE_SDD_MENU_TEXT);
        saveSddMenu.setEnabled(false);
        saveSddMenu.setMnemonic(KeyEvent.VK_D);
        saveSddMenu.setAccelerator(KeyStroke
                                    .getKeyStroke('D', InputEvent.CTRL_MASK));
        saveSddMenu.addActionListener(this);

        saveObsMenu = new JMenuItem(SAVE_OBS_MENU_TEXT);
        saveObsMenu.setEnabled(false);
        saveObsMenu.setMnemonic(KeyEvent.VK_O);
        saveObsMenu.setAccelerator(KeyStroke
                                    .getKeyStroke('O', InputEvent.CTRL_MASK));
        saveObsMenu.addActionListener(this);

        saveAllMenu = new JMenuItem("Save All");
        saveAllMenu.setEnabled(false);
        saveAllMenu.setMnemonic(KeyEvent.VK_S);
        saveAllMenu.setAccelerator(KeyStroke
                                    .getKeyStroke('S', InputEvent.CTRL_MASK));
        saveAllMenu.addActionListener(this);


        fileMenu.add(openPropMenu);
        fileMenu.add(openSdMenu);
        fileMenu.add(openObsMenu);
        if (supportDepFaults) {
            fileMenu.add(openFdgMenu);
            fileMenu.add(openSddMenu);
        }

        fileMenu.addSeparator();

        fileMenu.add(reloadPropMenu);
        fileMenu.add(reloadSdMenu);
        fileMenu.add(reloadObsMenu);
        if (supportDepFaults) {
            fileMenu.add(reloadFdgMenu);
            fileMenu.add(reloadSddMenu);
        }
        fileMenu.add(reloadAllMenu);

        fileMenu.addSeparator();

        fileMenu.add(savePropAsMenu);
        fileMenu.add(saveSdAsMenu);
        fileMenu.add(saveObsAsMenu);
        if (supportDepFaults) {
            fileMenu.add(saveFdgAsMenu);
            fileMenu.add(saveSddAsMenu);
        }

        fileMenu.addSeparator();

        fileMenu.add(savePropMenu);
        fileMenu.add(saveSdMenu);
        fileMenu.add(saveObsMenu);
        if (supportDepFaults) {
            fileMenu.add(saveFdgMenu);
            fileMenu.add(saveSddMenu);
        }
        fileMenu.add(saveAllMenu);
        
        JMenu editMenu = new JMenu("Edit");

        /*
        commentMenu = new JMenuItem("Comment");
        commentMenu.addActionListener(this);
        commentMenu.setAccelerator(KeyStroke
                                    .getKeyStroke('D', InputEvent.CTRL_MASK));
        commentMenu.addActionListener(this);

        uncommentMenu = new JMenuItem("Uncomment");
        uncommentMenu.setAccelerator(KeyStroke
                                    .getKeyStroke('D', InputEvent.CTRL_MASK | InputEvent.SHIFT_MASK));
        uncommentMenu.addActionListener(this);

        editMenu.add(commentMenu);
        editMenu.add(uncommentMenu);
        */

        menuBar.add(fileMenu);
        menuBar.add(editMenu);
        editMenu.setVisible(false);
        frame.setJMenuBar(menuBar);
    
    }  // createMenu

    private void readSettings() throws IOException {
        Properties prs = new Properties();
        File f = new File(FILE_SETTINGS);
        
        if (f.exists()) {
            FileInputStream stream = new FileInputStream(f);
            try {
                prs.load(stream);

                String s = prs.getProperty(PR_PROP_FILE);
                if (s != null) {
                    propFile = new File(s);
                    try {
                        readFile(propText, propFile);
                    } catch(IOException e) {
                        propFile = null;
                    }
                }
                
                s = prs.getProperty(PR_SD_FILE);
                if (s != null) {
                    sdFile = new File(s);
                    try {
                        readFile(sdText, sdFile);
                    } catch(IOException e) {
                        sdFile = null;
                    }
                }

                s = prs.getProperty(PR_OBS_FILE);
                if (s != null) {
                    obsFile = new File(s);
                    try {
                        readFile(obsText, obsFile);
                    } catch(IOException e) {
                        obsFile = null;
                    }
                }
                
                s = prs.getProperty(PR_FDG_FILE);
                if (s != null) {
                    fdgFile = new File(s);
                    try {
                        readFile(fdgText, fdgFile);
                    } catch(IOException e) {
                        fdgFile = null;
                    }
                }

                s = prs.getProperty(PR_SDD_FILE);
                if (s != null) {
                    sddFile = new File(s);
                    try {
                        readFile(sddText, sddFile);
                    } catch(IOException e) {
                        sddFile = null;
                    }
                }

            } finally {
                stream.close();
            }
        }

    }  // readSettings

    private void writeSettings() throws IOException {
        
        Properties prs = new Properties();
        
        if (propFile != null) prs.put(PR_PROP_FILE, propFile.getCanonicalPath());
        if (sdFile != null) prs.put(PR_SD_FILE, sdFile.getCanonicalPath());
        if (obsFile != null) prs.put(PR_OBS_FILE, obsFile.getCanonicalPath());
        if (fdgFile != null) prs.put(PR_FDG_FILE, fdgFile.getCanonicalPath());
        if (sddFile != null) prs.put(PR_SDD_FILE, sddFile.getCanonicalPath());

        FileOutputStream stream = new FileOutputStream(FILE_SETTINGS);
        try {
            prs.store(stream, null);
        } catch (IOException err) {
            System.out.println("Error on writing settings");
            throw err;
        } finally {
            stream.close();
        }
    }

    private void saveText(JTextArea textArea, File file) throws IOException {
        FileWriter fw = null;
        try {
            fw = new FileWriter(file);
            fw.write(textArea.getText());
        } finally {
            fw.close();
        }
    }

    private void readFile(JTextArea textArea, File file) throws IOException {
        FileReader fr = new FileReader(file);
        try {

            StringBuffer s = new StringBuffer();
            boolean eof = false;
            
            while (!eof) {
                char[] buf = new char[1024];
                int res = fr.read(buf, 0, buf.length);
                if (res == -1) {
                    eof = true;
                } else {
                    eof = false;
                    s.append(buf, 0, res);
                }
            }
            
            textArea.setText(s.toString());

        } finally {
            fr.close();
        }
    }

    private File showFileSaveAsDialog(String title) {
        JFileChooser fc;
        if (lastDir == null) fc = new JFileChooser();
        else fc = new JFileChooser(lastDir);
        fc.setDialogTitle("Save " + title + " to file..");

        while (true) {

            int returnVal = fc.showSaveDialog(frame);
            
            if (returnVal == JFileChooser.APPROVE_OPTION) {
                File f = fc.getSelectedFile();
                if (f.exists()) {
                    int opt = JOptionPane
                        .showOptionDialog(frame,
                                          "File already exists. Overwrite?", "?", 
                                          JOptionPane.YES_NO_OPTION,
                                          JOptionPane.QUESTION_MESSAGE, null, null, null);
                    if (opt == JOptionPane.YES_OPTION) {
                        return f;
                    }
                } else {
                    return f;
                }
                
            } else return null;
        }
    }

    private File showFileOpenDialog(String title) {

        JFileChooser fc;
        if (lastDir == null) fc = new JFileChooser();
        else fc = new JFileChooser(lastDir);
        
        fc.setDialogTitle("Read " + title + " from file..");

        int returnVal = fc.showOpenDialog(frame);
        if (returnVal == JFileChooser.APPROVE_OPTION) {
            File f = fc.getSelectedFile();
            return f;
        } else return null;
    }

    private void savePropAs() throws IOException {
        File file = showFileSaveAsDialog("Propositions");
        if (file != null) {
            saveText(propText, file);
            propFile = file;
            extractLastDir(file);
        }
    }

    private void saveSdAs() throws IOException {
        File file = showFileSaveAsDialog("System Description");
        if (file != null) {
            saveText(sdText, file);
            sdFile = file;
            extractLastDir(file);
        }
    }

    private void saveObsAs() throws IOException {
        File file = showFileSaveAsDialog("Observations");
        if (file != null) {
            saveText(obsText, file);
            obsFile = file;
            extractLastDir(file);
        }
    }

    private void saveFdgAs() throws IOException {
        File file = showFileSaveAsDialog("Failure Dependency Graph");
        if (file != null) {
            saveText(fdgText, file);
            fdgFile = file;
            extractLastDir(file);
        }
    }

    private void saveSddAs() throws IOException {
        File file = showFileSaveAsDialog("System Dependency Description");
        if (file != null) {
            saveText(sddText, file);
            sddFile = file;
            extractLastDir(file);
        }
    }

    private void saveProp() throws IOException {
        saveText(propText, propFile);
    }

    private void saveSd() throws IOException {
        saveText(sdText, sdFile);
    }

    private void saveObs() throws IOException {
        saveText(obsText, obsFile);
    }

    private void saveFdg() throws IOException {
        saveText(fdgText, fdgFile);
    }

    private void saveSdd() throws IOException {
        saveText(sddText, sddFile);
    }

    private void saveAll() throws IOException {
        if (propFile != null) saveText(propText, propFile);
        if (sdFile != null) saveText(sdText, sdFile);
        if (obsFile != null) saveText(obsText, obsFile);
        if (fdgFile != null) saveText(fdgText, fdgFile);
        if (sddFile != null) saveText(sddText, sddFile);
    }

    private void reloadAll() throws IOException {
        if (propFile != null) readFile(propText, propFile); 
        if (sdFile != null) readFile(sdText, sdFile); 
        if (obsFile != null) readFile(obsText, obsFile); 
        if (fdgFile != null) readFile(fdgText, fdgFile);
        if (sddFile != null) readFile(sddText, sddFile);
    }

    private void readPropFile() throws IOException {
        File file = showFileOpenDialog("Propositions");
        if (file != null) {
            readFile(propText, file);
            propFile = file;
            extractLastDir(file);
        }
    }

    private void readSdFile() throws IOException {
        File file = showFileOpenDialog("System Description");
        if (file != null) {
            readFile(sdText, file);
            sdFile = file;
            extractLastDir(file);
        }
    }

    private void readObsFile() throws IOException {
        File file = showFileOpenDialog("Observations");
        if (file != null) {
            readFile(obsText, file);
            obsFile = file;
            extractLastDir(file);
        }
    }

    private void readFdgFile() throws IOException {
        File file = showFileOpenDialog("Failure Dependency Graph");
        if (file != null) {
            readFile(fdgText, file);
            fdgFile = file;
            extractLastDir(file);
        }
    }

    private void readSddFile() throws IOException {
        File file = showFileOpenDialog("System Dependency Description");
        if (file != null) {
            readFile(sddText, file);
            sddFile = file;
            extractLastDir(file);
        }
    }

    private void extractLastDir(File file) {
        lastDir = file.getParentFile();
    }

    private void fillSettingsPanel(JPanel settingsPanel) {
   
        JPanel explSettingsPanel = new JPanel();
        explSettingsPanel.setBorder(BorderFactory.createTitledBorder("Explanations"));
        explSettingsPanel.setLayout(new SpringLayout());
        
        JLabel explSizeLabel = new JLabel("max. size:");
        explSettingsPanel.add(explSizeLabel);
        SpinnerModel explSizeModel = new SpinnerNumberModel(10, 1, 100, 1);
        explSizeSpinner = new JSpinner(explSizeModel);
        explSizeLabel.setLabelFor(explSizeSpinner);
        explSettingsPanel.add(explSizeSpinner);

        JLabel numExplLabel = new JLabel("max. number:");
        explSettingsPanel.add(numExplLabel);
        numExplText = new JTextField("100");
        numExplLabel.setLabelFor(numExplText);
        explSettingsPanel.add(numExplText);

        SpringUtilities.makeCompactGrid(explSettingsPanel, 2, 2, 3, 3, 3, 3);
        settingsPanel.add(explSettingsPanel);

        JPanel fmSettingsPanel = new JPanel();
        fmSettingsPanel.setBorder(BorderFactory.createTitledBorder("Fault modes"));
        fmSettingsPanel.setLayout(new BoxLayout(fmSettingsPanel, BoxLayout.Y_AXIS));
        
        useFaultModelsCB = new JCheckBox("use fault models", true);
        useFaultModelsCB.addItemListener(this);
        Box useFaultModelsBox = new Box(BoxLayout.X_AXIS);
        useFaultModelsBox.add(useFaultModelsCB);
        useFaultModelsBox.add(Box.createGlue());
        fmSettingsPanel.add(useFaultModelsBox);
        
        if (supportDepFaults) {
            depFaultsCB = new JCheckBox("dependent faults", true);
            depFaultsCB.addItemListener(this);
        
            Box depFaultsBox = new Box(BoxLayout.X_AXIS);
            depFaultsBox.add(depFaultsCB);
            depFaultsBox.add(Box.createGlue());
            fmSettingsPanel.add(depFaultsBox);
        }

        /*
          useProbabilitiesCB = new JCheckBox("use prob.", false);
        useProbabilitiesCB.addItemListener(this);
        Box useProbBox = new Box(BoxLayout.X_AXIS);
        useProbBox.add(useProbabilitiesCB);
        useProbBox.add(Box.createGlue());
        fmSettingsPanel.add(useProbBox);
        */

        JPanel assSettingsPanel = new JPanel();
        assSettingsPanel.setLayout(new SpringLayout());
        abLabel = new JLabel("AB assumpt.:");
        assSettingsPanel.add(abLabel);
        assABText = new JTextField("AB");
        abLabel.setLabelFor(assABText);
        assSettingsPanel.add(assABText);
        nabLabel = new JLabel("-AB assumpt.:");
        assSettingsPanel.add(nabLabel);
        assNABText = new JTextField("NAB");
        nabLabel.setLabelFor(assNABText);
        assSettingsPanel.add(assNABText);
        
        if (supportDepFaults) {
            dfLabel = new JLabel("DF assumpt.:");
            dfLabel.setEnabled(true);
            assSettingsPanel.add(dfLabel);
            assDFText = new JTextField("DF");
            assDFText.setEditable(true);
            assSettingsPanel.add(assDFText);
        }

        if (supportDepFaults) SpringUtilities.makeCompactGrid(assSettingsPanel, 3, 2, 3, 3, 3, 3);
        else SpringUtilities.makeCompactGrid(assSettingsPanel, 2, 2, 3, 3, 3, 3);
        fmSettingsPanel.add(assSettingsPanel);
        settingsPanel.add(fmSettingsPanel);

        JPanel propSettingsPanel = new JPanel();
        propSettingsPanel.setBorder(BorderFactory.createTitledBorder("Propositions"));
        propSettingsPanel.setLayout(new SpringLayout());
        JLabel negPrefixLabel = new JLabel("negation prefix:");
        propSettingsPanel.add(negPrefixLabel);
        negPrefixText = new JTextField("n_");
        negPrefixLabel.setLabelFor(negPrefixText);
        propSettingsPanel.add(negPrefixText);        
        SpringUtilities.makeCompactGrid(propSettingsPanel, 1, 2, 3, 3, 3, 3);

        settingsPanel.add(propSettingsPanel);

        if (supportDepFaults) {
            dfSettingsPanel = new JPanel();
            dfSettingsPanel.setBorder(BorderFactory.createTitledBorder("Dependent faults"));
            dfSettingsPanel.setLayout(new SpringLayout());
            
            betaEnvCB = new JCheckBox("compute BETA env.", true);
            betaEnvCB.addItemListener(this);
            Box betaEnvBox = new Box(BoxLayout.X_AXIS);
            betaEnvBox.add(betaEnvCB);
            betaEnvBox.add(Box.createGlue());
            dfSettingsPanel.add(betaEnvBox);
            
            mergeDEsCB = new JCheckBox("merge DEs", true);
            mergeDEsCB.addItemListener(this);
            Box mergeDEsBox = new Box(BoxLayout.X_AXIS);
            mergeDEsBox.add(mergeDEsCB);
            mergeDEsBox.add(Box.createGlue());
            dfSettingsPanel.add(mergeDEsBox);
            
            discardOrderPermsCB = new JCheckBox("no order perms.", false);
            discardOrderPermsCB.addItemListener(this);
            Box discardOrderPermsBox = new Box(BoxLayout.X_AXIS);
            discardOrderPermsBox.add(discardOrderPermsCB);
            discardOrderPermsBox.add(Box.createGlue());
            dfSettingsPanel.add(discardOrderPermsBox);

            Box maxDFChainBox = new Box(BoxLayout.X_AXIS);
            
            JLabel maxDFChainLabel = new JLabel("max. DF:");
            maxDFChainBox.add(maxDFChainLabel);
            maxDFChainBox.add(Box.createGlue());
            SpinnerModel maxDFChainModel = new SpinnerNumberModel(10, 1, 1000, 1);
            maxDFChainSpinner = new JSpinner(maxDFChainModel);
            maxDFChainBox.add(maxDFChainSpinner);
            maxDFChainLabel.setLabelFor(maxDFChainSpinner);
            dfSettingsPanel.add(maxDFChainBox);
            
            SpringUtilities.makeCompactGrid(dfSettingsPanel, 4, 1, 3, 3, 3, 3);
            
            settingsPanel.add(dfSettingsPanel);
        }

    }  // fillSettingsPanel()

    private void fillActionsPanel(JPanel actionsPanel) {

        actionsPanel.setLayout(new SpringLayout());

        /*
        JPanel clearPanel = new JPanel();
        clearPanel.setLayout(new GridLayout(3, 1, 3, 3));
        
        clearPropBtn = new JButton("Clear propositions");
        clearPropBtn.addActionListener(this);
        clearPanel.add(clearPropBtn);

        clearSdBtn = new JButton("Clear System Descr.");
        clearSdBtn.addActionListener(this);
        clearPanel.add(clearSdBtn);

        clearObsBtn = new JButton("Clear Observations");
        clearObsBtn.addActionListener(this);
        clearPanel.add(clearObsBtn);

        actionsPanel.add(clearPanel);
        */

        JPanel commentPanel = new JPanel();
        commentPanel.setLayout(new GridLayout(2, 1, 3, 3));

        commentBtn = new JButton("Comment");
        commentBtn.addActionListener(this);
        commentPanel.add(commentBtn);

        uncommentBtn = new JButton("Uncomment");
        uncommentBtn.addActionListener(this);
        commentPanel.add(uncommentBtn);
        actionsPanel.add(commentPanel);

        JPanel searchPanel = new JPanel();
        
        searchPanel.setLayout(new GridLayout(3, 1, 5, 5));
        searchPanel.setBorder(BorderFactory.createTitledBorder("Search"));
        
        searchText = new JTextField();
        searchPanel.add(searchText);

        searchFirstBtn = new JButton("First");
        searchFirstBtn.addActionListener(this);
        searchNextBtn = new JButton("Next");
        searchNextBtn.addActionListener(this);

        Box searchBtnBox = new Box(BoxLayout.X_AXIS);
        searchBtnBox.add(searchFirstBtn);
        searchBtnBox.add(Box.createGlue());
        searchBtnBox.add(searchNextBtn);
        searchPanel.add(searchBtnBox);

        searchResultLabel = new JLabel("");
        searchPanel.add(searchResultLabel);

        actionsPanel.add(searchPanel);

        SpringUtilities.makeCompactGrid(actionsPanel, 2, 1, 10, 10, 10, 10);

    }  // fillActionsPanel()

    private void fillDepFaultPanel(JPanel depFaultPanel) {        

        depFaultPanel.setLayout(new BorderLayout());

        JPanel fdgPanel = new JPanel();
        JPanel fdgBgPanel = new JPanel();
        fdgBgPanel.setBorder(BorderFactory.createEmptyBorder(5, 5, 0, 0));
        fdgBgPanel.setLayout(new BorderLayout());
        fdgBgPanel.add(fdgPanel, BorderLayout.CENTER);
        fdgPanel.setBorder(BorderFactory.createTitledBorder("Failure Dependency Graph"));
        fdgPanel.setLayout(new BorderLayout());
        fdgText = new JTextArea(15, 40);
        setTextAreaAttributes(fdgText);
        fdgText.addFocusListener(this);
        JScrollPane fdgTextScrollPane = new JScrollPane(fdgText,
                                                        JScrollPane.VERTICAL_SCROLLBAR_ALWAYS,
                                                        JScrollPane.HORIZONTAL_SCROLLBAR_ALWAYS);
        fdgPanel.add(fdgTextScrollPane, BorderLayout.CENTER);

        JPanel sddPanel = new JPanel();
        JPanel sddBgPanel = new JPanel();
        sddBgPanel.setLayout(new BorderLayout());
        sddBgPanel.setBorder(BorderFactory.createEmptyBorder(5, 5, 0, 0));
        sddBgPanel.add(sddPanel, BorderLayout.CENTER);
        sddPanel.setBorder(BorderFactory.createTitledBorder("System Dependency Description"));
        sddPanel.setLayout(new BorderLayout());
        sddText = new JTextArea(15, 40);
        setTextAreaAttributes(sddText);
        sddText.addFocusListener(this);
        JScrollPane sddTextScrollPane  = new JScrollPane(sddText,
                                                         JScrollPane.VERTICAL_SCROLLBAR_ALWAYS,
                                                         JScrollPane.HORIZONTAL_SCROLLBAR_ALWAYS);
        sddPanel.add(sddTextScrollPane, BorderLayout.CENTER);

        JSplitPane depFaultSplitPane = new JSplitPane(JSplitPane.HORIZONTAL_SPLIT, fdgBgPanel, 
                                                      sddBgPanel);
        depFaultPanel.add(depFaultSplitPane, BorderLayout.CENTER);
    }

    private void fillFrame() {
         
        JPanel settingsPanel = new JPanel();        
        settingsPanel.setBorder(BorderFactory.createTitledBorder("Settings"));
        settingsPanel.setLayout(new SpringLayout());
        fillSettingsPanel(settingsPanel);

        if (supportDepFaults) SpringUtilities.makeCompactGrid(settingsPanel, 4, 1, 5, 5, 5, 5);
        else SpringUtilities.makeCompactGrid(settingsPanel, 3, 1, 5, 5, 5, 5);
        
        JPanel settingsBgPanel = new JPanel();
        settingsBgPanel.setBorder(BorderFactory.createEmptyBorder(5, 5, 5, 5));
        settingsBgPanel.setLayout(new BorderLayout());
        settingsBgPanel.add(settingsPanel, BorderLayout.NORTH);
  
        JPanel actionsPanel = new JPanel();
        actionsPanel.setBorder(BorderFactory.createTitledBorder("Edit Actions"));
        fillActionsPanel(actionsPanel);

        settingsBgPanel.add(actionsPanel, BorderLayout.SOUTH);
        
        JPanel propBgPanel = new JPanel();
        propBgPanel.setBorder(BorderFactory.createEmptyBorder(5, 5, 0, 0));
        propBgPanel.setLayout(new BorderLayout());
        JPanel propPanel = new JPanel();
        propBgPanel.add(propPanel, BorderLayout.CENTER);
        propPanel.setBorder(BorderFactory.createTitledBorder("Propositions")); 
        propPanel.setLayout(new BorderLayout());
        propText = new JTextArea(30, 30);
        setTextAreaAttributes(propText);      
        propText.addFocusListener(this);
        JScrollPane propScrollPane = new JScrollPane(propText,
                                                     JScrollPane.VERTICAL_SCROLLBAR_ALWAYS,
                                                     JScrollPane.HORIZONTAL_SCROLLBAR_ALWAYS);
        propPanel.add(propScrollPane, BorderLayout.CENTER);

        JPanel sdBgPanel = new JPanel();
        sdBgPanel.setBorder(BorderFactory.createEmptyBorder(5, 5, 0, 0));
        sdBgPanel.setLayout(new BorderLayout());
        JPanel sdPanel = new JPanel();
        sdBgPanel.add(sdPanel, BorderLayout.CENTER);
        sdPanel.setBorder(BorderFactory.createTitledBorder("System Description"));
        sdPanel.setLayout(new BorderLayout());
        sdText = new JTextArea(22, 70);        
        setTextAreaAttributes(sdText);
        sdText.addFocusListener(this);
        JScrollPane sdScrollPane = new JScrollPane(sdText,
                                                   JScrollPane.VERTICAL_SCROLLBAR_ALWAYS,
                                                   JScrollPane.HORIZONTAL_SCROLLBAR_ALWAYS);
        sdPanel.add(sdScrollPane, BorderLayout.CENTER);

        JPanel obsBgPanel = new JPanel();
        obsBgPanel.setBorder(BorderFactory.createEmptyBorder(5, 5, 0, 0));
        obsBgPanel.setLayout(new BorderLayout());
        JPanel obsPanel = new JPanel();
        obsBgPanel.add(obsPanel, BorderLayout.CENTER);
        obsPanel.setBorder(BorderFactory.createTitledBorder("Observations"));
        obsPanel.setLayout(new BorderLayout());
        obsText = new JTextArea(8, 70);        
        setTextAreaAttributes(obsText);
        obsText.addFocusListener(this);
        JScrollPane obsScrollPane = new JScrollPane(obsText,
                                                    JScrollPane.VERTICAL_SCROLLBAR_ALWAYS,
                                                    JScrollPane.HORIZONTAL_SCROLLBAR_ALWAYS);
        obsPanel.add(obsScrollPane, BorderLayout.CENTER);

        JSplitPane sdObsSplitPane = new JSplitPane(JSplitPane.VERTICAL_SPLIT, sdBgPanel, obsBgPanel);
        //sdObsSplitPane.setDividerLocation(0.8);
        JSplitPane propSplitPane = new JSplitPane(JSplitPane.HORIZONTAL_SPLIT, propBgPanel, sdObsSplitPane);
        propSplitPane.setDividerLocation(0.8);
        JPanel propSplitPaneBgPanel = new JPanel();
        propSplitPaneBgPanel.setLayout(new BorderLayout());
        propSplitPaneBgPanel.setBorder(BorderFactory.createEmptyBorder(5, 5, 0, 0));
        propSplitPaneBgPanel.add(propSplitPane, BorderLayout.CENTER);

        JPanel depFaultBgPanel = new JPanel();
        depFaultBgPanel.setLayout(new BorderLayout());
        depFaultBgPanel.setBorder(BorderFactory.createEmptyBorder(5, 5, 0, 0));
        JPanel depFaultPanel = new JPanel();
        fillDepFaultPanel(depFaultPanel);
        depFaultBgPanel.add(depFaultPanel, BorderLayout.CENTER);

        tabbedPane = new JTabbedPane();
        tabbedPane.addTab("Logical Model", null, propSplitPaneBgPanel);
        if (supportDepFaults) {
            tabbedPane.addTab("Dependent Fault Models", null, depFaultBgPanel);
        }

        JPanel topPanel = new JPanel();
        topPanel.setLayout(new BorderLayout());
        topPanel.add(settingsBgPanel, BorderLayout.WEST);
        topPanel.add(tabbedPane, BorderLayout.CENTER);

        JPanel resultButtonBgPanel = new JPanel();
        resultButtonBgPanel.setBorder(BorderFactory.createEmptyBorder(5, 5, 5, 5));
        resultButtonBgPanel.setLayout(new BorderLayout());
        JPanel resultButtonPanel = new JPanel();
        resultButtonBgPanel.add(resultButtonPanel, BorderLayout.CENTER);
        resultButtonPanel.setBorder(BorderFactory.createEmptyBorder());
        resultButtonPanel.setLayout(new GridLayout(4, 1, 10, 10));
        checkConsBtn = new JButton("Check Consistency");
        checkConsBtn.addActionListener(this);
        resultButtonPanel.add(checkConsBtn);
        computeMinHSBtn = new JButton("Compute Min. HS");
        computeMinHSBtn.addActionListener(this);
        resultButtonPanel.add(computeMinHSBtn);
        computeMoreBtn = new JButton("Compute more HS");
        computeMoreBtn.addActionListener(this);
        computeMoreBtn.setEnabled(false);
        resultButtonPanel.add(computeMoreBtn);
        
        if (supportDepFaults) {
            computeMinDiagEnvBtn = new JButton("Compute Min. DEs");
            computeMinDiagEnvBtn.addActionListener(this);
            resultButtonPanel.add(computeMinDiagEnvBtn);
        }

        JPanel resultBgPanel = new JPanel();
        resultBgPanel.setBorder(BorderFactory.createEmptyBorder(5, 5, 0, 0));
        resultBgPanel.setLayout(new BorderLayout());
        resultsPanel = new JPanel();
        resultBgPanel.add(resultsPanel, BorderLayout.CENTER);
        resultsBorder = BorderFactory.createTitledBorder(TITLE_RESULTS);
        resultsPanel.setBorder(resultsBorder);
        resultsPanel.setLayout(new GridLayout(1, 2, 10, 10));

        fillResultPanel(resultsPanel);

        JPanel bottomPanel = new JPanel();
        bottomPanel.setLayout(new BorderLayout());
        bottomPanel.add(resultButtonBgPanel, BorderLayout.WEST);
        bottomPanel.add(resultBgPanel, BorderLayout.CENTER);

        JSplitPane mainSplitPane = new JSplitPane(JSplitPane.VERTICAL_SPLIT, topPanel, bottomPanel);
        //mainSplitPane.setDividerLocation(0.8);
        frame.getContentPane().add(mainSplitPane);

        createMenu();

    }  // fillFrame()

    void fillResultPanel(JPanel resultsPanel) {

        results1Text = new JTextArea(8, 10);
        setTextAreaAttributes(results1Text);
        results1Text.setEditable(false);
        JScrollPane res1ScrollPane = new JScrollPane(results1Text,
                                                     JScrollPane.VERTICAL_SCROLLBAR_ALWAYS,
                                                     JScrollPane.HORIZONTAL_SCROLLBAR_ALWAYS);
        resultsPanel.add(res1ScrollPane, BorderLayout.WEST);
        
        results2Text = new JTextArea(8, 10);
        setTextAreaAttributes(results2Text);
        results2Text.setEditable(false);
        JScrollPane res2ScrollPane = new JScrollPane(results2Text,
                                                     JScrollPane.VERTICAL_SCROLLBAR_ALWAYS,
                                                     JScrollPane.HORIZONTAL_SCROLLBAR_ALWAYS);
        resultsPanel.add(res2ScrollPane, BorderLayout.CENTER);

    }

    /* 
     * ignoredAss: list of Strings; contains assumption names. Sentences containing
     * one of these assumptions are NOT parsed!
     */
    protected LSentence parseSD() throws ParseError, IllegalUserInput {

        LogicParser parser = new LogicParser();
        LSentence allRules = new LSentence();

        generatePropNegationAxioms(propText, parser, allRules);
        parseLogSentences(sdText, parser, allRules);
  
        return allRules;
    }

    protected LSentence parseOBS() throws ParseError, IllegalUserInput {
        LogicParser parser = new LogicParser();
        LSentence allRules = new LSentence();

        parseLogSentences(obsText, parser, allRules);

        return allRules;
    }

    protected LSentence parseSDD() throws ParseError, IllegalUserInput {
        LogicParser parser = new LogicParser();
        LSentence allRules = new LSentence();

        parseLogSentences(sddText, parser, allRules);

        return allRules;
    }

    protected String readAssAB() {
        String res = assABText.getText().trim();
        if ((res.length() == 0) || (Character.isLowerCase(res.charAt(0)))) {
            return null;
        } else return res;
    }

    protected String readAssNAB() {
        String res = assNABText.getText().trim();
        if ((res.length() == 0) || (Character.isLowerCase(res.charAt(0)))) {
            return null;
        } else return res;
    }

    protected String readAssDF() {
        String res = assDFText.getText().trim();
        if ((res.length() == 0) || (Character.isLowerCase(res.charAt(0)))) {
            return null;
        } else return res;
    }

    protected boolean checkConsistency(LSentence allRules) throws ParseError,
        IllegalUserInput {
        
        // prepare theorem prover, then check consistency

        ABTheoremProver theoremProver = new ABTheoremProver();
        theoremProver = allRules.asABPropositionalSentence(theoremProver);
        if (theoremProver == null) {
            throw new ParseError("[unknown]");
        }

        boolean consistent;

        if (useFaultModelsCB.isSelected()) {
            String assAB = readAssAB();
            String assNAB = readAssNAB();

            if (assAB == null) {
                throw new IllegalUserInput("Invalid AB assumption defined!");
            } else if (assNAB == null) {
                throw new IllegalUserInput("Invalid NAB assumption defined!");
            }

            ArrayList posAssPrefixes = new ArrayList();
            posAssPrefixes.add(assNAB);
            consistent = theoremProver.checkConsistency(posAssPrefixes);
        } else {
            consistent = theoremProver.checkConsistency();
        }

        // print conflict set to console

        ArrayList conflict = theoremProver.contradiction().collectAssumptions();
        printConflictSet(conflict);

        return consistent;
    }

    protected void printConflictSet(ArrayList conflict) {        

        String cs = "";
        Iterator itC = conflict.iterator();
        while (itC.hasNext()) {
            Assumption a = (Assumption)itC.next();
            if (cs.length() == 0) cs += "-";
            else cs += " \\/ -";
            cs += (String)a.identifier;
        }

        System.out.println("\n\nConflict set returned by theorem prover: " + cs + "\n\n");
    }

    protected int readExplSize() throws IllegalUserInput {
        Number n = (Number)explSizeSpinner.getValue();
        return n.intValue();
    }

    protected int readMaxDFChain()  throws IllegalUserInput {
        Number n = (Number)maxDFChainSpinner.getValue();
        return n.intValue();
    }

    protected int readNumExpl() throws IllegalUserInput {
        String text = numExplText.getText().trim();

        if (text.length() == 0) throw new IllegalUserInput("The max. number of explanations must be defined!");

        int numExpl;
        try {
            numExpl = Integer.parseInt(text);
        } catch (NumberFormatException e) {
            throw new IllegalUserInput("The max. number of explanations must be defined as integer!");
        }

        if (numExpl < 1) throw new IllegalUserInput("The max. number of explanations must be larger than 0");

        return numExpl;
    }

    /*
     * The results of the computation are returned in diagnoses and conflictSets.
     * These lists contain String's.
     *
     * Returns true iff all min. diagnoses have been computed. Returns false if there may be more min. diagnoses.
     */
    protected boolean computeMinHS(LSentence allRules, ArrayList diagnoses, ArrayList conflictSets) 
        throws ParseError, IllegalUserInput {
        
        Date startComputationTime = new Date();

        hs = null;
        hsFM = null;
        boolean hasMoreDiags;
        ABTheoremProver theoremProver = new ABTheoremProver();
        theoremProver = allRules.asABPropositionalSentence(theoremProver);
        if (theoremProver == null) {
            throw new ParseError("[unknown]");
        }

        // results from the algorithm
        ArrayList minHittingSetsAsAss;
        ArrayList conflictsAsAss;

        int explSize = readExplSize();
        int numExpl = readNumExpl();
        if (useFaultModelsCB.isSelected()) {
            
           String assAB = readAssAB();
           String assNAB = readAssNAB();
           if (assAB == null) {
               throw new IllegalUserInput("Invalid AB assumption defined!");
           } else if (assNAB == null) {
               throw new IllegalUserInput("Invalid NAB assumption defined!");
           } 

           try {
               hsFM = new MinHittingSetsFM(false, theoremProver, assAB, assNAB);
           } catch (IllegalAssumption e) {
               throw new IllegalUserInput("Illegal assumption in sentence: " + e.getAssumption());
           }
           
           int computationResult = hsFM.compute(explSize, numExpl);
           hasMoreDiags = (computationResult != MinHittingSetsFM.CS_ALL_MIN_DIAGS_COMPUTED);
           minHittingSetsAsAss = hsFM.getMinHS();
           conflictsAsAss = hsFM.getConflictsAsAss();
        
        } else {
            hs = new MinHittingSets(false, theoremProver);
            int computationResult = hs.compute(explSize, numExpl);
            hasMoreDiags = (computationResult != MinHittingSets.CS_ALL_MIN_DIAGS_COMPUTED);
            minHittingSetsAsAss = hs.getMinHS();
            conflictsAsAss = hs.getConflictsAsAss();
        }
        
        Date endComputationTime = new Date();
        long passedTime = endComputationTime.getTime() - startComputationTime.getTime();        
        System.out.println("Computation time [ms]: " + passedTime);
        
        // compose the "diagnoses" list which is returned
        composeDiagnosesAsStrings(minHittingSetsAsAss, diagnoses);        
        // compose the "conflictSets" list which is returned
        composeConflictSetsAsStrings(conflictsAsAss, conflictSets);        
       
        return !hasMoreDiags;

    }  // computeMinHS()

    protected boolean computeMoreHS(ArrayList diagnoses, ArrayList conflictSets) 
        throws ParseError, IllegalUserInput {

        assert((hs != null) || (hsFM != null));
        
        Date startComputationTime = new Date();
        
        // results from the algorithm
        boolean hasMoreDiags;
        ArrayList minHittingSetsAsAss;
        ArrayList conflictsAsAss;

        int explSize = readExplSize();
        int numExpl = readNumExpl();
        if (hsFM != null) {
            int computationResult = hsFM.computeMore(explSize, numExpl);
            hasMoreDiags = (computationResult != MinHittingSetsFM.CS_ALL_MIN_DIAGS_COMPUTED);
            minHittingSetsAsAss = hsFM.getMinHS();
            conflictsAsAss = hsFM.getConflictsAsAss();
        } else {
            int computationResult = hs.computeMore(explSize, numExpl);
            hasMoreDiags = (computationResult != MinHittingSets.CS_ALL_MIN_DIAGS_COMPUTED);
            minHittingSetsAsAss = hs.getMinHS();
            conflictsAsAss = hs.getConflictsAsAss();
        }

        Date endComputationTime = new Date();
        long passedTime = endComputationTime.getTime() - startComputationTime.getTime();        
        System.out.println("Computation time [ms]: " + passedTime);

        // compose the "diagnoses" list which is returned
        composeDiagnosesAsStrings(minHittingSetsAsAss, diagnoses);        
        // compose the "conflictSets" list which is returned
        composeConflictSetsAsStrings(conflictsAsAss, conflictSets); 

        return !hasMoreDiags;

    }  // computeMoreHS()

    protected void composeDiagnosesAsStrings(ArrayList minHittingSetsAsAss, ArrayList result) {
        int index = 1;

        Iterator itMinHS = minHittingSetsAsAss.iterator();
        while(itMinHS.hasNext()) {
            String diagStr = "" + index + ": ";
            ++index;
            
            ArrayList expl = (ArrayList)itMinHS.next();
            boolean firstExpl = true;
            Iterator itExpl = expl.iterator();
            while(itExpl.hasNext()) {
                Assumption a = ((Assumption)itExpl.next());
                if (firstExpl) firstExpl = false;
                else diagStr += ", ";
                diagStr += a.toString();
            }         
            
            result.add(diagStr);
        }
    }
    
    protected void composeConflictSetsAsStrings(ArrayList conflictsAsAss, ArrayList result) {
        Iterator itCS = conflictsAsAss.iterator();
        while (itCS.hasNext()) {
            String conflictStr = "";
            ArrayList cs = (ArrayList)itCS.next();
            Iterator itAss = cs.iterator();
            while (itAss.hasNext()) {
                Object o = itAss.next();
                assert (o != null);
                Assumption a = (Assumption)o;
                if (conflictStr.length() > 0) conflictStr += " \\/ -";
                else conflictStr += "-";
                conflictStr += a.toString();
            }
            result.add(conflictStr);
        }
    }

    /*
     * Parses the FDG from the GUI and adds the nodes/edges to the FDG in the diagnosis problem.
     */
    protected void parseFDG(DiagnosisProblem diagProblem, boolean useProb) throws ParseError {
        String fdgStr = fdgText.getText();
        StringTokenizer tokenizer = new StringTokenizer(fdgStr, "\n");
        
        while(tokenizer.hasMoreTokens()) {  // iterate through FDG tokens
            
            String line = tokenizer.nextToken().trim();
            if (isCommentLine(line)) continue;
            Double prob = null;
            
            int pos = line.indexOf("[");
            if (pos == 0) throw new ParseError(line);
            else if ((pos < 0) && useProb) throw new ParseError(line);
            else if (pos > 0) {
                int pos1 = line.indexOf("]");
                if (pos1 < pos) throw new ParseError(line);
                String probStr = line.substring(pos + 1, pos1).trim();
                try {
                    prob = new Double(probStr);
                    line = line.substring(0, pos).trim();
                } catch (NumberFormatException err) {
                    throw new ParseError(line);
                    }
            }
            assert(!useProb || (prob != null));
            
            pos = line.indexOf(FDG_EDGE_STR);
            
            if (pos < 0) {
                if (line.length() > 0) {
                    String comp1Str = line;
                    if (diagProblem.hasComponent(comp1Str)) {
                        throw new ParseError(line);
                    }
                    if (useProb) diagProblem.addComponent(comp1Str, prob.doubleValue());
                        else diagProblem.addComponent(comp1Str);
                    
                }
            } else {
                String comp1Str = line.substring(0, pos).trim();
                String comp2Str = line.substring(pos + FDG_EDGE_STR.length()).trim();
                
                if ((comp1Str.length() == 0) || (comp2Str.length() == 0)) {
                    throw new ParseError(line);
                }
                if (!diagProblem.hasComponent(comp1Str) || !diagProblem.hasComponent(comp2Str)) {
                    throw new ParseError(line);
                } 
                
                if (useProb) diagProblem.addFailureDep(comp1Str, comp2Str, prob.doubleValue());
                else diagProblem.addFailureDep(comp1Str, comp2Str);
            }
            
        }  // iterate through lines
    
    }  // parseFDG()
    
    protected ArrayList computeDiagEnv(boolean mergeDEs, boolean discardOrderPerms) throws ParseError, 
        IllegalUserInput {
        
        System.out.println("computeDiagEnv");

        LSentence sd = parseSD();
        LSentence obs = parseOBS();
        LSentence indepModel = new LSentence();
        indepModel.addRules(sd);
        indepModel.addRules(obs);

        ArrayList result;

        boolean consistent = checkConsistency(indepModel);
        if (consistent) {
            result = new ArrayList();
            result.add("Consistent!");
            System.out.println("consistent");

        } else {  // inconsistent

            System.out.println("inconsistent");

            // read user settings

            int explSize = readExplSize();
            int numExpl = readNumExpl();

            boolean computeBetaDE = betaEnvCB.isSelected();
            //boolean useProb = useProbabilitiesCB.isSelected();
            boolean useProb = false;
            int maxDFChain = readMaxDFChain();

            String assAB = readAssAB();
            String assNAB = readAssNAB();
            String assDF = readAssDF();
            if (assAB == null) {
                throw new IllegalUserInput("Invalid AB assumption defined!");
            } else if (assNAB == null) {
                throw new IllegalUserInput("Invalid NAB assumption defined!");
            } else if (assDF == null) {
                throw new IllegalUserInput("Invalid DF assumption defined!");
            }

            // create DiagnosisProblem, assign logical model

            LSentence sdd = parseSDD();

            DiagnosisProblem diagProblem = new DiagnosisProblem(useProb, assAB, assNAB, ASS_IF, assDF);
            diagProblem.setSD(sd);
            diagProblem.setOBS(obs);
            diagProblem.setSDD(sdd);
            
            // parse FDG

            parseFDG(diagProblem, useProb);

            // compute diagnoses and DEs
            
            ArrayList minHS;
            ArrayList conflictSets = new ArrayList();
            try {
                minHS = diagProblem.computeMinHittingSets(explSize, numExpl, conflictSets);
            } catch (IllegalAssumption e) {
                throw new IllegalUserInput("Illegal assumption in sentence: " + e.getAssumption());
            }
     
            if (mergeDEs || discardOrderPerms) {
                RepairCandidates rcs 
                    = diagProblem.computeRepairCandidates(minHS, computeBetaDE,
                                                          maxDFChain, discardOrderPerms, conflictSets);
                result = composeRepairCandidateResult(rcs);
            } else {
                ArrayList minDEs = diagProblem.computeDEs(minHS, computeBetaDE, 
                                                          maxDFChain, conflictSets);
                result = composeMinDEsResult(diagProblem, minDEs, useProb);
            }
            
        } // inconsistent
        
        return result;
        
    }  // computeDiagEnv
    
    protected ArrayList composeRepairCandidateResult(RepairCandidates rcs) {
        ArrayList result = new ArrayList(rcs.size());
        
        Iterator itCands = rcs.iterator();
        while (itCands.hasNext()) {
            RepairCandidate rc = (RepairCandidate)itCands.next();
            result.add(rc.toString());
        }

        return result;
    }
    
    protected ArrayList composeMinDEsResult(DiagnosisProblem diagProblem, ArrayList minDEs, boolean useProb) {
        ArrayList result = new ArrayList(minDEs.size());

        if (useProb) {

            ArrayList rankedDEs = diagProblem.computeDERanking(minDEs, true);
            result = new ArrayList(rankedDEs.size());
            
            DecimalFormat formatter = new DecimalFormat("0.###E0");
            int rank = 1;
            Iterator itDE = rankedDEs.iterator();
            while (itDE.hasNext()) {
                ObjectPair op = (ObjectPair)itDE.next();
                ModeAssignment de = (ModeAssignment)op.first;
                double prob = ((Double)op.last).doubleValue();
                String probStr = formatter.format(prob);                    
                String resultStr = "" + rank + ". [" + probStr + "]: " + de.toStringShort(); 
                result.add(resultStr);
                ++rank;
            }
            
        } else {
            result = new ArrayList(minDEs.size());
            Iterator itDE = minDEs.iterator();
            while (itDE.hasNext()) {
                ModeAssignment de = (ModeAssignment)itDE.next();
                result.add(de.toStringShort());
            }
        }
        
        return result;
    }

    protected void onComputeMinHSClick() throws IllegalUserInput, ParseError {
        LSentence allRules = parseSD();
        LSentence obs = parseOBS();
        allRules.addRules(obs);
        boolean consistent = checkConsistency(allRules);
        
        if (consistent) {
            displayResult("Consistent!", "");
            computeMoreBtn.setEnabled(false);
        } else {
            
            ArrayList diagnoses = new ArrayList();  // list of String
            ArrayList conflictSets = new ArrayList();  // list of String
            boolean allDiagsComputed = computeMinHS(allRules, diagnoses, conflictSets);
            displayResults(diagnoses, (new Integer(diagnoses.size())).toString());
            //printConflictSets(conflictSets);
            computeMoreBtn.setEnabled(!allDiagsComputed);
        }
    }

    protected void onComputeMoreHSClick() throws IllegalUserInput, ParseError {
        assert((hs != null) || (hsFM != null));

        ArrayList diagnoses = new ArrayList();  // list of String
        ArrayList conflictSets = new ArrayList();  // list of String
        boolean allDiagsComputed = computeMoreHS(diagnoses, conflictSets);
        displayResults(diagnoses, (new Integer(diagnoses.size())).toString());
        //printConflictSets(conflictSets);
        computeMoreBtn.setEnabled(!allDiagsComputed);
    }

    public void actionPerformed(ActionEvent e) {
        
        try {

            if (e.getSource() == checkConsBtn) {
                
                LSentence allRules = parseSD();
                LSentence obs = parseOBS();
                allRules.addRules(obs);
                boolean consistent = checkConsistency(allRules);
                
                if (consistent) displayResult("Consistent!", "");
                else displayResult("Not consistent!", "");

            } else if (e.getSource() == computeMinHSBtn) {
                onComputeMinHSClick();
            } else if (e.getSource() == computeMoreBtn) {
                onComputeMoreHSClick();
            } else if (e.getSource() == computeMinDiagEnvBtn) {
                ArrayList results = computeDiagEnv(mergeDEsCB.isSelected(), discardOrderPermsCB.isSelected());
                displayResults(results, (new Integer(results.size())).toString());
                /*} else if (e.getSource() == computeMergedDEBtn) {
                ArrayList results = computeDiagEnv(true);
                displayResults(results, (new Integer(results.size())).toString());  */
            } else if (e.getSource() == savePropAsMenu) {
                savePropAs();
            } else if (e.getSource() == saveSdAsMenu) {
                saveSdAs();
            } else if (e.getSource() == saveObsAsMenu) {
                saveObsAs();
            } else if (e.getSource() == saveFdgAsMenu) {
                saveFdgAs();
            } else if (e.getSource() == saveSddAsMenu) {
                saveSddAs();
            } else if (e.getSource() == openPropMenu) {
                readPropFile();  
            } else if (e.getSource() == openSdMenu) {
                readSdFile();  
            } else if (e.getSource() == openObsMenu) {
                readObsFile();  
            } else if (e.getSource() == openFdgMenu) {
                readFdgFile();  
            } else if (e.getSource() == openSddMenu) {
                readSddFile();  
            } else if (e.getSource() == reloadPropMenu) {
                readFile(propText, propFile);  
            } else if (e.getSource() == reloadSdMenu) {
                readFile(sdText, sdFile);  
            } else if (e.getSource() == reloadObsMenu) {
                readFile(obsText, obsFile); 
            } else if (e.getSource() == reloadFdgMenu) {
                readFile(fdgText, fdgFile);    
             } else if (e.getSource() == reloadSddMenu) {
                readFile(sddText, sddFile);    
            } else if (e.getSource() == reloadAllMenu) {
                reloadAll();
            } else if (e.getSource() == savePropMenu) {
                saveProp();
            } else if (e.getSource() == saveSdMenu) {
                saveSd();
            } else if (e.getSource() == saveObsMenu) {
                saveObs();
            } else if (e.getSource() == saveFdgMenu) {
                saveFdg();
            } else if (e.getSource() == saveSddMenu) {
                saveSdd();  
            } else if (e.getSource() == saveAllMenu) {
                saveAll();
            } else if (e.getSource() == clearPropBtn) {
                propText.setText("");
            } else if (e.getSource() == clearSdBtn) {
                sdText.setText("");
            } else if (e.getSource() == clearObsBtn) {
                obsText.setText("");  
            } else if ((e.getSource() == commentBtn) || (e.getSource() == commentMenu)) {
                addCommentChars();
            } else if ((e.getSource() == uncommentBtn) || (e.getSource() == uncommentMenu)) {
                removeCommentChars();
            } else if ((e.getSource() == searchFirstBtn)) {
                searchFirst();
            } else if ((e.getSource() == searchNextBtn)) {
                searchNext();
            } else assert(false);
            
        } catch (ParseError err) {

            displayResult("Parse error in line: " + err.getLine(), "");

        } catch (IllegalUserInput err) {
            displayResult("Illegal user input: " + err.getMessage(), "");
        } catch (IOException err) {
            JOptionPane.showMessageDialog(frame, err.getMessage(),
                                          "IO error", JOptionPane.ERROR_MESSAGE);
        }

        updateMenuStatus();
    }

    // print conflict sets passed as strings
    protected void printConflictSets(ArrayList conflictSets) {
        System.out.println("\n\n");
        System.out.println("***********  Conflict sets:  **********");
        Iterator itCS = conflictSets.iterator();
        while (itCS.hasNext()) {
            String cs = (String)itCS.next();
            System.out.println(cs);
        }

        System.out.println("\n\n ***********************************\n\n");
    }

    protected void doSearch(boolean fromCursor) {
        String stext = searchText.getText().trim();
        
        if (stext.length() > 0) {
            
            JTextArea searchDest = null;
            if (focusedComponent == propText) searchDest = propText;
            else if (focusedComponent == sdText) searchDest = sdText;
            else if (focusedComponent == obsText) searchDest = obsText;
            else if (focusedComponent == fdgText) searchDest = fdgText;
            else if (focusedComponent == sddText) searchDest = sddText;

            if (searchDest != null) {
                String text = searchDest.getText();
                int searchStartPos = 0;
                if (fromCursor) searchStartPos = searchDest.getSelectionEnd();

                int foundPos = text.indexOf(stext, searchStartPos);
                if (foundPos != -1) {
                    searchDest.setSelectionStart(foundPos);
                    searchDest.setSelectionEnd(foundPos + stext.length());
                    searchDest.grabFocus();
                    
                    searchResultLabel.setText("Found!");
                } else {
                    searchResultLabel.setText("Not found!");
                }

            } 

        }
    }

    protected void searchFirst() {
        doSearch(false);
    }

    protected void searchNext() {
        doSearch(true);
    }

    protected void addCommentChars(JTextArea textArea) {

        try {
            int startLine = textArea.getLineOfOffset(textArea.getSelectionStart());
            int endLine = textArea.getLineOfOffset(textArea.getSelectionEnd());
            
            System.out.println("startLine: " + startLine);
            System.out.println("endLine: " + endLine);

            for (int iLine = startLine; iLine <= endLine; ++iLine) {
                System.out.println("addCommentChars: line: " + iLine);
                int startOff = textArea.getLineStartOffset(iLine);
                textArea.insert((new Character(AUTO_COMMENT_PREFIX)).toString(), startOff);
            }
        } catch (BadLocationException e) {
            assert(false);
        }
    }

    public void focusGained(FocusEvent e) {
        focusedComponent = e.getSource();
    }

    public void focusLost(FocusEvent e) {
    }

    protected void removeCommentChars(JTextArea textArea) {
        try {
            int startLine = textArea.getLineOfOffset(textArea.getSelectionStart());
            int endLine = textArea.getLineOfOffset(textArea.getSelectionEnd());
            
            String text = textArea.getText();
            int startOffIncr = 0;

            for (int iLine = startLine; iLine <= endLine; ++iLine) {
                int startOff = textArea.getLineStartOffset(iLine);
                
                System.out.println("line " + iLine + ": " + text.charAt(startOff));
                if (text.charAt(startOff + startOffIncr) == AUTO_COMMENT_PREFIX) {
                    textArea.replaceRange("", startOff, startOff + 1);
                    ++startOffIncr;
                }
            }
        } catch (BadLocationException e) {
            assert(false);
        }
    }

    protected void addCommentChars() {
        if (focusedComponent == propText) addCommentChars(propText);
        else if (focusedComponent == sdText) addCommentChars(sdText);
        else if (focusedComponent == obsText) addCommentChars(obsText);
        else if (focusedComponent == fdgText) addCommentChars(fdgText);
        else if (focusedComponent == sddText) addCommentChars(sddText);
    }

    protected void removeCommentChars() {
        if (focusedComponent == propText) removeCommentChars(propText);
        else if (focusedComponent == sdText) removeCommentChars(sdText);
        else if (focusedComponent == obsText) removeCommentChars(obsText);
        else if (focusedComponent == fdgText) removeCommentChars(fdgText);
        else if (focusedComponent == sddText) removeCommentChars(sddText);
    }

    protected void displayResult(String result, String title) {
        
        ArrayList resultLines = new ArrayList();
        resultLines.add(result);
        displayResults(resultLines, title);
    }

    protected void displayResults(ArrayList resultLines, String title) {

        resultsBorder.setTitle(TITLE_RESULTS + title);
        resultsPanel.repaint();

        StringBuffer text1 = new StringBuffer();
        StringBuffer text2 = new StringBuffer();

        for (int i = 0; i < resultLines.size(); ++i) {
            if (i % 2 == 0) {
                text1.append(resultLines.get(i));
                text1.append('\n');
            } else {
                text2.append(resultLines.get(i));
                text2.append('\n');
            }
        }

        results1Text.setText(text1.toString());
        results2Text.setText(text2.toString());
    }

    public void itemStateChanged(ItemEvent e) {
        Object source = e.getItemSelectable();

        if (supportDepFaults && (source == useFaultModelsCB)) {            
            if (useFaultModelsCB.isSelected()) {
                depFaultsCB.setEnabled(true);
                abLabel.setEnabled(true);
                nabLabel.setEnabled(true);
                assABText.setEditable(true);
                assNABText.setEditable(true);
            } else {
                depFaultsCB.setEnabled(false);
                depFaultsCB.setSelected(false);
                abLabel.setEnabled(false);
                nabLabel.setEnabled(false);
                dfLabel.setEnabled(false);
                assABText.setEditable(false);
                assNABText.setEditable(false);
                assDFText.setEditable(false);
                betaEnvCB.setEnabled(false);
            }

        } else if (supportDepFaults && (source == depFaultsCB)) {

            if (depFaultsCB.isSelected()) {
                //useProbabilitiesCB.setEnabled(true);
                dfLabel.setEnabled(true);
                assDFText.setEditable(true);
                computeMinDiagEnvBtn.setEnabled(true);
                betaEnvCB.setEnabled(true);
                mergeDEsCB.setEnabled(true);
                discardOrderPermsCB.setEnabled(true);
            } else {
                /*useProbabilitiesCB.setSelected(false);
                  useProbabilitiesCB.setEnabled(false);*/
                dfLabel.setEnabled(false);
                assDFText.setEditable(false);
                computeMinDiagEnvBtn.setEnabled(false);
                betaEnvCB.setEnabled(false);
                mergeDEsCB.setEnabled(false);
                discardOrderPermsCB.setEnabled(false);
            }
        } else if (source == betaEnvCB) {
            // do nothing
        } else if (source == useProbabilitiesCB) {
            assert(false);
        } else if (supportDepFaults && (source == mergeDEsCB)) {
            if (mergeDEsCB.isSelected()) {
                discardOrderPermsCB.setSelected(false);
            }
        } else if (supportDepFaults && (source == discardOrderPermsCB)) {
            if (discardOrderPermsCB.isSelected()) {
                mergeDEsCB.setSelected(false);
            }
        }
 
    }  // itemStateChanged()

    // it is assumed that the line is already trimmed
    protected boolean isCommentLine(String line) {
        for (int i = 0; i < COMMENT_PREFIXES.length; ++i) {
            if (line.startsWith(COMMENT_PREFIXES[i])) return true;
        }
        return false;
    }

    protected void generatePropNegationAxioms(JTextArea propTextArea, LogicParser parser, 
                                              LSentence allRules) 
        throws ParseError, IllegalUserInput {

        String text = propTextArea.getText();
        StringTokenizer tokenizer = new StringTokenizer(text, "\n");

        String negationPrefix = getNegPrefix();

        while(tokenizer.hasMoreTokens()) {
            String prop = tokenizer.nextToken().trim();
            
            if ((prop.length() > 0) && !isCommentLine(prop)) { 
                String line = prop + ", " + negationPrefix + prop + " -> false.";
                
                if (parser.parse(line)) {
                    allRules.addRules((LSentence)parser.result());
                } else throw new ParseError(prop);
            }
        }
    }

    protected String getNegPrefix() throws IllegalUserInput {
        String prefix = negPrefixText.getText().trim();
        if (prefix.length() == 0) throw new IllegalUserInput("Invalid negation prefix defined!");

        return prefix;
    }

    /* 
     * ignoredAss: list of Strings; contains assumption names. Sentences containing
     * one of these assumptions are NOT parsed!
     */
    protected void parseLogSentences(JTextArea textArea, LogicParser parser, LSentence allRules) 
        throws ParseError {

        String text = textArea.getText();
        StringTokenizer tokenizer = new StringTokenizer(text, "\n");
        
        while(tokenizer.hasMoreTokens()) {
            String line = tokenizer.nextToken().trim();
            
            if ((line.length() > 0) && !isCommentLine(line)) {                
                if (parser.parse(line)) {   
                    allRules.addRules((LSentence)parser.result());
                } else throw new ParseError(line);
            }
            
        } 
    }

    public void windowClosing(WindowEvent e) {
        
        int opt = JOptionPane
            .showOptionDialog(frame,
                              "Are you sure you want to exit? Everything saved?", "GuiHittingSets", 
                              JOptionPane.YES_NO_OPTION,
                              JOptionPane.QUESTION_MESSAGE, null, null, null);
        if (opt == JOptionPane.YES_OPTION) {
            try {
                writeSettings();
            } catch (IOException exc) {}

            System.exit(0);
        }
        
    }

    public void windowClosed(WindowEvent e) {
    }

    public void windowOpened(WindowEvent e) {
    }

    public void windowIconified(WindowEvent e) {
    }

    public void windowDeiconified(WindowEvent e) {
    }

    public void windowActivated(WindowEvent e) {
    }

    public void windowDeactivated(WindowEvent e) {
    }

}


class IllegalUserInput extends Exception {

    public IllegalUserInput(String msg) {
        super(msg);
    }

}

// The following code is from the Java documentation

/**
 * A 1.4 file that provides utility methods for
 * creating form- or grid-style layouts with SpringLayout.
 * These utilities are used by several programs, such as
 * SpringBox and SpringCompactGrid.
 */
class SpringUtilities {
    /**
     * A debugging utility that prints to stdout the component's
     * minimum, preferred, and maximum sizes.
     */
    public static void printSizes(java.awt.Component c) {
        System.out.println("minimumSize = " + c.getMinimumSize());
        System.out.println("preferredSize = " + c.getPreferredSize());
        System.out.println("maximumSize = " + c.getMaximumSize());
    }

    /**
     * Aligns the first <code>rows</code> * <code>cols</code>
     * components of <code>parent</code> in
     * a grid. Each component is as big as the maximum
     * preferred width and height of the components.
     * The parent is made just big enough to fit them all.
     *
     * @param rows number of rows
     * @param cols number of columns
     * @param initialX x location to start the grid at
     * @param initialY y location to start the grid at
     * @param xPad x padding between cells
     * @param yPad y padding between cells
     */
    public static void makeGrid(Container parent,
                                int rows, int cols,
                                int initialX, int initialY,
                                int xPad, int yPad) {
        SpringLayout layout;
        try {
            layout = (SpringLayout)parent.getLayout();
        } catch (ClassCastException exc) {
            System.err.println("The first argument to makeGrid must use SpringLayout.");
            return;
        }

        Spring xPadSpring = Spring.constant(xPad);
        Spring yPadSpring = Spring.constant(yPad);
        Spring initialXSpring = Spring.constant(initialX);
        Spring initialYSpring = Spring.constant(initialY);
        int max = rows * cols;

        //Calculate Springs that are the max of the width/height so that all
        //cells have the same size.
        Spring maxWidthSpring = layout.getConstraints(parent.getComponent(0)).
                                    getWidth();
        Spring maxHeightSpring = layout.getConstraints(parent.getComponent(0)).
                                    getWidth();
        for (int i = 1; i < max; i++) {
            SpringLayout.Constraints cons = layout.getConstraints(
                                            parent.getComponent(i));

            maxWidthSpring = Spring.max(maxWidthSpring, cons.getWidth());
            maxHeightSpring = Spring.max(maxHeightSpring, cons.getHeight());
        }

        //Apply the new width/height Spring. This forces all the
        //components to have the same size.
        for (int i = 0; i < max; i++) {
            SpringLayout.Constraints cons = layout.getConstraints(
                                            parent.getComponent(i));

            cons.setWidth(maxWidthSpring);
            cons.setHeight(maxHeightSpring);
        }

        //Then adjust the x/y constraints of all the cells so that they
        //are aligned in a grid.
        SpringLayout.Constraints lastCons = null;
        SpringLayout.Constraints lastRowCons = null;
        for (int i = 0; i < max; i++) {
            SpringLayout.Constraints cons = layout.getConstraints(
                                                 parent.getComponent(i));
            if (i % cols == 0) { //start of new row
                lastRowCons = lastCons;
                cons.setX(initialXSpring);
            } else { //x position depends on previous component
                cons.setX(Spring.sum(lastCons.getConstraint(SpringLayout.EAST),
                                     xPadSpring));
            }

            if (i / cols == 0) { //first row
                cons.setY(initialYSpring);
            } else { //y position depends on previous row
                cons.setY(Spring.sum(lastRowCons.getConstraint(SpringLayout.SOUTH),
                                     yPadSpring));
            }
            lastCons = cons;
        }

        //Set the parent's size.
        SpringLayout.Constraints pCons = layout.getConstraints(parent);
        pCons.setConstraint(SpringLayout.SOUTH,
                            Spring.sum(
                                Spring.constant(yPad),
                                lastCons.getConstraint(SpringLayout.SOUTH)));
        pCons.setConstraint(SpringLayout.EAST,
                            Spring.sum(
                                Spring.constant(xPad),
                                lastCons.getConstraint(SpringLayout.EAST)));
    }

    /* Used by makeCompactGrid. */
    private static SpringLayout.Constraints getConstraintsForCell(
        int row, int col,
        Container parent,
        int cols) {
        SpringLayout layout = (SpringLayout) parent.getLayout();
        java.awt.Component c = parent.getComponent(row * cols + col);
        return layout.getConstraints(c);
    }

    /**
     * Aligns the first <code>rows</code> * <code>cols</code>
     * components of <code>parent</code> in
     * a grid. Each component in a column is as wide as the maximum
     * preferred width of the components in that column;
     * height is similarly determined for each row.
     * The parent is made just big enough to fit them all.
     *
     * @param rows number of rows
     * @param cols number of columns
     * @param initialX x location to start the grid at
     * @param initialY y location to start the grid at
     * @param xPad x padding between cells
     * @param yPad y padding between cells
     */
    public static void makeCompactGrid(Container parent,
                                       int rows, int cols,
                                       int initialX, int initialY,
                                       int xPad, int yPad) {
        SpringLayout layout;
        try {
            layout = (SpringLayout)parent.getLayout();
        } catch (ClassCastException exc) {
            System.err.println("The first argument to makeCompactGrid must use SpringLayout.");
            return;
        }

        //Align all cells in each column and make them the same width.
        Spring x = Spring.constant(initialX);
        for (int c = 0; c < cols; c++) {
            Spring width = Spring.constant(0);
            for (int r = 0; r < rows; r++) {
                width = Spring.max(width,
                                   getConstraintsForCell(r, c, parent, cols).
                                       getWidth());
            }
            for (int r = 0; r < rows; r++) {
                SpringLayout.Constraints constraints =
                        getConstraintsForCell(r, c, parent, cols);
                constraints.setX(x);
                constraints.setWidth(width);
            }
            x = Spring.sum(x, Spring.sum(width, Spring.constant(xPad)));
        }

        //Align all cells in each row and make them the same height.
        Spring y = Spring.constant(initialY);
        for (int r = 0; r < rows; r++) {
            Spring height = Spring.constant(0);
            for (int c = 0; c < cols; c++) {
                height = Spring.max(height,
                                    getConstraintsForCell(r, c, parent, cols).
                                        getHeight());
            }
            for (int c = 0; c < cols; c++) {
                SpringLayout.Constraints constraints =
                        getConstraintsForCell(r, c, parent, cols);
                constraints.setY(y);
                constraints.setHeight(height);
            }
            y = Spring.sum(y, Spring.sum(height, Spring.constant(yPad)));
        }

        //Set the parent's size.
        SpringLayout.Constraints pCons = layout.getConstraints(parent);
        pCons.setConstraint(SpringLayout.SOUTH, y);
        pCons.setConstraint(SpringLayout.EAST, x);
    }
}
