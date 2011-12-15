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



package ATPInterface;

/**
 * This interface provides constants for the ATP (Assumption-based Theorem-prover Protocol).
 *
 * These constants are related to the text-based communication protocol ATP. The constants
 * are not documented here, see the documentation of ATP.
 *
 * @author Joerg WEBER
 * @version 1.0, DATE: 18.10.2005
 *
 *
 */
public interface ATPConstants {
    
    public final static String CMD_POST = "POST";
    public final static String CMD_GET = "GET";
    public final static String CMD_CLOSE = "CLOSE";
    
    public final static String SUBCMD_ADD_SENTENCES = "ADD_SENTENCES";
    public final static String SUBCMD_REPLACE_SENTENCES = "REPLACE_SENTENCES";
    public final static String SUBCMD_ADD_FDG_EDGES = "ADD_FDG_EDGES";
    public final static String SUBCMD_REPLACE_FDG_EDGES = "REPLACE_FDG_EDGES";
    public final static String SUBCMD_CONSISTENCY = "CONSISTENCY";
    public final static String SUBCMD_CONSISTENCIES = "CONSISTENCIES";
    public final static String SUBCMD_MINDIAG = "MIN_DIAG";
    public final static String SUBCMD_DIAGENV = "DIAG_ENV";
    public final static String SUBCMD_DBSTATS = "DBSTATS";
    public final static String SUBCMD_FDGSTATS = "FDGSTATS";
    public final static String SUBCMD_DBCONTENT = "DBCONTENT";

    public final static String PARAM_CONTENT_TYPE = "Content-Type:";
    public final static String PARAM_NUM_RULES = "Number-Rules:";
    public final static String PARAM_NUM_QUERIES = "Number-Queries:";
    public final static String PARAM_NUM_NODES = "Number-FDG-Nodes:";
    public final static String PARAM_NUM_EDGES = "Number-FDG-Edges:";
    public final static String PARAM_LINE_NUMBER = "Line-Number:";
    public final static String PARAM_NUM_SUBDB_RULES = "Number-SubDB-Rules:";
    public final static String PARAM_NUM_SUBDBS = "Number-SubDBs:";
    public final static String PARAM_CONSISTENT = "Consistent:";
    public final static String PARAM_NUM_DIAG = "Number-Diagnoses:";
    public final static String PARAM_SUBDB = "SubDB:";
    public final static String PARAM_MAX_DIAG_SIZE = "Max-Diag-Size:";
    public final static String PARAM_MAX_NUM_DIAG = "Max-Number-Diagnoses:";
    public final static String PARAM_USE_FAULT_MODES = "Use-Fault-Modes:";
    public final static String PARAM_MAX_DF_CHAIN = "Max-DF-Chain:";
    public final static String PARAM_INCL_BETA_DE = "Include-Beta-DEs:";
    public final static String PARAM_NUM_DIAGENV = "Number-DEs:";
    public final static String PARAM_MERGE_DES = "Merge-DEs:";
    public final static String PARAM_DISCARD_ORDER_PERMS = "Discard-Order-Perms:";
    
    public final static String STR_ATP = "ATP";
    public final static String STR_FDG_EDGE = "=>";
    public final static String STR_YES = "yes";
    public final static String STR_NO = "no";

    public final static String LINE_PREFIX_DIAG = "DIAG:";
    public final static String LINE_PREFIX_DE = "DE:";

    // the assumption names to be used, unless otherwise defined
    public final static String DEF_AB_ASSUMPTION = "AB";
    public final static String DEF_NAB_ASSUMPTION = "NAB";
    public final static String DEF_IF_ASSUMPTION = "IF";
    public final static String DEF_DF_ASSUMPTION = "DF";

    // Separator between assumptions. E.g.: "IF(A); DF(A, B)"
    public final static String SEP_ASSUMPTION = ";";

    public final static int NUM_SUBDBS = 3;
    public final static String SUBDB_SD = "SD";
    public final static String SUBDB_OBS = "OBS";
    public final static String SUBDB_SDD = "SDD";

    public final static int OK = 200;
    public final static int ERR_BAD_REQUEST = 400;
    public final static int ERR_ILLEGAL_RULE = 410;
    public final static int ERR_ILLEGAL_ASS = 420;
    public final static int ERR_INTERNAL_SERVER_ERROR = 500;

}

