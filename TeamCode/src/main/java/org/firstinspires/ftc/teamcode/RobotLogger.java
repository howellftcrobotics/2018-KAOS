package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.HashMap;
import java.util.Map;

/**
 *  This class only has static methods that allows easy calling without having to create a RobotLogger object.
 *  The first method that needs to be called is the {@link #initRobotLog(String,Telemetry,SEVERITY)}  to allow the setup of telemetry and logcat tag.
 */
public class RobotLogger
{
    static private Telemetry telemetry=null;
    static private String Tag;
    static private Map<String,Telemetry.Item> watchItems; //a hashmap of telemetry items that can be indexed by watch name

    public enum SEVERITY
    {
        ERROR,
        WARN,
        INFO
    }
    static private SEVERITY severityLevel=SEVERITY.INFO;  // default to all severity levels

    /**
     * This is the first method to call when setting up both the telemetry and the logcat to help
     * in debugging the robot program.
     * @param inTag the tag that can be filtered for in the logcat tab of Android Studio
     * @param inTelemetry the telemetry to log messages and watch items to
     * @param inseverity the severity level determines which messages to send to logcat and telemetry or to just skip.
     *                   INFO - shows INFO, WARNING, and ERROR on telemetry and logcat
     *                   WARN - shows WARNING and ERROR on telemetry and logcat
     *                   ERROR - only shows ERROR on telemetry and logcat
     */
    static public void InitRobotLog(String inTag, Telemetry inTelemetry,SEVERITY inseverity)
    {
        telemetry = inTelemetry;
        severityLevel=inseverity;
        Tag = inTag;
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
    }

    /**
     * Outputs an Error Message to the telemetry on the Driver Station and to the Logcat.
     *
     * @param msg the error message
     */
    static public void error(String msg)
    {
        //check that setup has happened and severity level allows logging
        if((telemetry==null) || (severityLevel.ordinal()<SEVERITY.ERROR.ordinal()))
        {
            return;
        }

        //build string
        String tmpstr = "ERROR: "+msg;
        //add to telemetry log
        telemetry.log().add(tmpstr);
        //write to log file
        RobotLog.ee(Tag,tmpstr);
    }

    /**
     *  Outputs an Error Message to the telemetry on the Driver Station and to the Logcat.  If Telemetry is not set before calling this method then the Error Message will only be saved to the Log File.
     *  The msg string will be the result of calling String.format() with the indicated format and arguments.
     *
     * @param msg the error message with formatting characters
     * @param args the arguments for the message's formatting characters
     */
    static public void error(String msg, Object args)
    {
        String tmpstr = String.format(msg,args); //format string then pass the string to error method
        error(tmpstr);
    }
    /**
     * Outputs an Warning Message to the telemetry on the Driver Station and to the Logcat.
     *
     * @param msg the warning message
     */
    static public void warning(String msg)
    {
        //check that setup has happened and severity level allows logging
        if((telemetry==null) || (severityLevel.ordinal()<SEVERITY.WARN.ordinal()))
        {
            return;
        }

        //build string
        String tmpstr = "WARNING: "+msg;
        //add to telemetry log
        telemetry.log().add(tmpstr);
        //write to log file
        RobotLog.ww(Tag,tmpstr);
    }

    /**
     *  Outputs an Warning Message to the telemetry on the Driver Station and to the Logcat.  If Telemetry is not set before calling this method then the Error Message will only be saved to the Log File.
     *  The msg string will be the result of calling String.format() with the indicated format and arguments.
     *
     * @param msg the warning message with formatting characters
     * @param args the arguments for the message's formatting characters
     */
    static public void warning(String msg, Object args)
    {
        String tmpstr = String.format(msg,args); //format string then pass the string to warning method
        warning(tmpstr);
    }
    /**
     * Outputs an Information Message to the telemetry on the Driver Station and to the Logcat.
     *
     * @param msg the information message
     */
    static public void info(String msg)
    {
        //check that setup has happened and severity level allows logging
        if((telemetry==null) || (severityLevel.ordinal()<SEVERITY.INFO.ordinal()))
        {
            return;
        }

        //build string
        String tmpstr = "INFO: "+msg;
        //add to telemetry log
        telemetry.log().add(tmpstr);
        //write to log file
        RobotLog.ii(Tag,tmpstr);
    }

    /**
     *  Outputs an Information Message to the telemetry on the Driver Station and to the Logcat.  If Telemetry is not set before calling this method then the Error Message will only be saved to the Log File.
     *  The msg string will be the result of calling String.format() with the indicated format and arguments.
     *
     * @param msg the information message with formatting characters
     * @param args the arguments for the message's formatting characters
     */
    static public void info(String msg, Object args)
    {
        String tmpstr = String.format(msg,args); //format string then pass the string to info method
        info(tmpstr);
    }

    /**
     *  This method setups the list of watch values to view on telemetry and to the logcat.
     *  This only shows if SEVERITY level is INFO.
     *
     * @param names array of strings that contain the caption names for the items to watch.
     */
    static public void initWatchValues(String... names)
    {
        //check that setup has happened and severity level allows logging
        if((telemetry == null) || (severityLevel.ordinal() < SEVERITY.INFO.ordinal()))
        {
            return;
        }

        watchItems = new HashMap<>();  //clear any previous watch items
        telemetry.setAutoClear(false);

        // create the telemetry items to later add values to
        for(int i=0;i<names.length;i++)
        {
            Telemetry.Item telemetryItem = telemetry.addData(names[i],0);
            watchItems.put(names[i], telemetryItem);
       }
    }

    /**
     * This method sets the formatted value for the named watch item
     * This only shows if SEVERITY level is INFO.
     * @param name  the caption of the watch item to update the value of
     * @param format the format string of the value
     * @param value the value to be formatted into the string above
     */
    static public void setWatchValue(String name, String format, Object... value)
    {
        //check that setup has happened and severity level allows logging
        if((telemetry==null) || (severityLevel.ordinal()<SEVERITY.INFO.ordinal()))
        {
            return;
        }

        //format the value
        String tmpstr = String.format(format,value);
        //get the named telemetry item to update the value of
        Telemetry.Item telemetryItem = watchItems.get(name);
        //we correctly found a telemetry item so set the value
        if(telemetryItem!=null)
        {
            telemetryItem.setValue(tmpstr); //update telemetry with watch value
            telemetry.update();
        }
        else //could not find the telemetry item to send error to logcat
        {
            RobotLog.ee(Tag, "Did not get the TelemetryItem for " + name);
        }
        RobotLog.ii(Tag,"WATCH("+name+"): "+tmpstr); //log watch value
    }
}