/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: jSettingsFile.java                                        */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Class that manages a settings file for runtime     */
/*              re-configuration without rebuilding.               */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Copyright 1994-2008. Acroname Inc.                              */
/*                                                                 */
/* This software is the property of Acroname Inc.  Any             */
/* distribution, sale, transmission, or re-use of this code is     */
/* strictly forbidden except with permission from Acroname Inc.    */
/*                                                                 */
/* To the full extent allowed by law, Acroname Inc. also excludes  */
/* for itself and its suppliers any liability, wheither based in   */
/* contract or tort (including negligence), for direct,            */
/* incidental, consequential, indirect, special, or punitive       */
/* damages of any kind, or for loss of revenue or profits, loss of */
/* business, loss of information or data, or other financial loss  */
/* arising out of or in connection with this software, even if     */
/* Acroname Inc. has been advised of the possibility of such       */
/* damages.                                                        */
/*                                                                 */
/* Acroname Inc.                                                   */
/* www.acroname.com                                                */
/* 720-564-0373                                                    */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

package acroname;

import java.io.*;
import java.util.*;
import java.awt.*;

public class jSettingsFile {

  private Hashtable settings;
  private File file;


  ///////////////////////////////////////////////////////////////////

  private void ReapSettings(BufferedReader in) throws Exception
  { 
    // read in the settings
    String line;
    for(;;) {
      line = in.readLine();
      if (line == null) {
        break;
      } else if (!line.startsWith("#")) {
        int l = line.indexOf("=");
        if (l != -1) {
          String key = line.substring(0, l).trim();
          String value = line.substring(l + 1).trim();
          settings.put(key, value);
        }
      }
    }

  } // ReapSettings constructor



  ///////////////////////////////////////////////////////////////////

  public jSettingsFile(File file) 
  {
    settings = new Hashtable();
    this.file = file;
    
    // read in the settings file
    try {
      BufferedReader in;
      if (file.exists()) {
        in = new BufferedReader(
	       new InputStreamReader(
		 new FileInputStream(file)));
	ReapSettings(in);
        in.close();
      } else {
        // remain quiet if no file is found and resort 
        // to defaults
      }
    } catch (Exception e) {
      System.out.println("error jSettingsFile " 
      			 + e.toString());
    }

  } // jSettingsFile constructor



  ///////////////////////////////////////////////////////////////////

  public jSettingsFile(String content) 
  {    
    settings = new Hashtable();

    // read in the settings file
    try {
      BufferedReader in;
      in = new BufferedReader(
      	     new StringReader(content));
      ReapSettings(in);
    } catch (Exception e) {
      System.out.println("error creating ship server settings " 
      			 + e.toString());
    }

  } // jSettingsFile constructor



  ///////////////////////////////////////////////////////////////////

  public jSettingsFile(BufferedReader in) 
  {
    settings = new Hashtable();
    
    // read in the settings file
    try {
      String line;
      for(;;) {
        line = in.readLine();
        if (line == null) {
          break;
        } else if (!line.startsWith("#")) {
          int l = line.indexOf("=");
          if (l != -1) {
            String key = line.substring(0, l).trim();
            String value = line.substring(l + 1).trim();
            settings.put(key, value);
          }
        }
      }
    } catch (Exception e) {
      System.out.println("error settings file " 
      			 + e.toString());
    }

  } // jSettingsFile constructor



  ///////////////////////////////////////////////////////////////////
  
  public int getInt(String key, int defaultSetting)
  {
    int retVal = defaultSetting;

    try {
      if (key != null) {
        String settingString = (String)settings.get(key);
        if (settingString != null) {
          retVal = Integer.parseInt(settingString);
        }
      }
    } catch (Exception e) {
    }

    return retVal;

  } // getInt method



  ///////////////////////////////////////////////////////////////////
  
  public long getLong(String key, long defaultSetting)
  {
    long retVal = defaultSetting;

    if (key != null) {
      String settingString = (String)settings.get(key);
      if (settingString != null) {
        retVal = Long.parseLong(settingString);
      }
    }
    
    return retVal;

  } // getLong method



  ///////////////////////////////////////////////////////////////////
  
  public String getString(String key, String defaultSetting)
  {
    String retVal = defaultSetting;

    if (key != null) {
      String settingString = (String)settings.get(key);
      if (settingString != null) {
        retVal = settingString;
      }
    }
    
    return retVal;

  } // getString method



  ///////////////////////////////////////////////////////////////////
  
  public File getDir(String key, File defaultSetting)
  {
    File retVal = defaultSetting;

    if (key != null) {
      String settingString = (String)settings.get(key);
      if (settingString != null) {
        File settingDir = new File(settingString);
        if (settingDir.exists() && settingDir.isDirectory()) {
          retVal = settingDir;
        }
      }
    }
    
    return retVal;

  } // getDir method



  ///////////////////////////////////////////////////////////////////
  
  public File getFile(String key, File defaultSetting)
  {
    File retVal = defaultSetting;

    if (key != null) {
      String settingString = (String)settings.get(key);
      if (settingString != null) {
        File settingFile = new File(settingString);
        if (settingFile.exists() && settingFile.isFile()) {
          retVal = settingFile;
        }
      }
    }
    
    return retVal;

  } // getFile method



  ///////////////////////////////////////////////////////////////////
  
  public Color getColor(String key, Color defaultSetting)
  {
    Color retVal = defaultSetting;

    if (key != null) {
      String settingString = (String)settings.get(key);
      if (settingString != null) {
        int colorInt = Integer.parseInt(settingString, 16);
        retVal = new Color(colorInt);
      }
    }
    
    return retVal;

  } // getColor method



  ///////////////////////////////////////////////////////////////////
  
  public boolean getBoolean(String key, boolean defaultSetting)
  {
    boolean retVal = defaultSetting;

    if (key != null) {
      String settingString = (String)settings.get(key);
      if (settingString != null) {
        retVal = settingString.trim().equals("true");
      }
    }
    
    return retVal;

  } // getBoolean method



  ///////////////////////////////////////////////////////////////////
  
  public void addString(String key, String value)
  {
    settings.put(key, value);

  } // addString method

} // jSettingsFile class
