package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OpModeConfig
{
    //TODO: Make sure this is in the proper config
    public final boolean CompetitionMode = true;
    public final boolean LoggingDisabled = true;
    public final boolean UseCoreTelemetryLogging = false;
    public final RobotLogger.SEVERITY LoggingLevel = RobotLogger.SEVERITY.INFO;

    public static final String WATCHES_MOTOR_POWER = "Motors Power";
    public static final String WATCHES_COUNTS_LEFT_FRONT = "LeftFront Counts";
    public static final String WATCHES_COUNTS_RIGHT_FRONT = "RightFront Counts";
    public static final String WATCHES_COUNTS_LEFT_REAR = "LeftRear Counts";
    public static final String WATCHES_COUNTS_RIGHT_REAR = "RightRear Counts";

    // defines the potential starting positions on the field
    public enum ROBOT_STARTING_POSITION
    {
        UNKNOWN,
        CRATER,
        CRATER_DOUBLE_SAMPLE,
        CRATER_LAND_SAMPLE_EXTEND,
        DEPOT,
        DEPOT_TEST
    }

    // defines the team
    public enum ROBOT_TEAM
    {
        UNKNOWN,
        KILTS,
        KAOS,
        KUDOS,
        KRASH
    }

    // defines the operating/driving mode of the robot if there are more than one
    public enum ROBOT_OPERATING_MODE
    {
        UNKNOWN,
        TELEOP,
        AUTONOMOUS
    }

    public enum ROBOT_DRIVE_TRAIN_SET
    {
        UNKNOWN,
        A_LF_RR,
        B_RF_LR,
        C_ALL_FORWARD_BACKWARD,
        D_ALL_SIDE_2_SIDE
    }

    // public configurations properties of this OpMode
    public ROBOT_STARTING_POSITION StartingPosition = ROBOT_STARTING_POSITION.UNKNOWN;
    public ROBOT_TEAM Team = ROBOT_TEAM.UNKNOWN;
    public ROBOT_OPERATING_MODE OperatingMode = ROBOT_OPERATING_MODE.UNKNOWN;
    public ROBOT_DRIVE_TRAIN_SET DriveTrainSet = ROBOT_DRIVE_TRAIN_SET.UNKNOWN;
    public Telemetry Telemetry = null;
    public HardwareMap HardwareMap = null;
    public LinearOpMode OpMode = null;

    // public constructor - pass in all the values of the OpMode when you are creating the class
    public OpModeConfig(ROBOT_TEAM robotTeam, ROBOT_OPERATING_MODE robotOperatingMode,
                        ROBOT_STARTING_POSITION robotStartingPosition,
                        Telemetry telemetry,
                        HardwareMap hardwareMap, LinearOpMode baseOpMode)
    {
        StartingPosition = robotStartingPosition;
        Team = robotTeam;
        OperatingMode = robotOperatingMode;
        Telemetry = telemetry;
        HardwareMap = hardwareMap;
        OpMode = baseOpMode;

        // initialize the the logger with the current team name
        RobotLogger.InitRobotLog("Team:" + robotTeam.toString(), telemetry, LoggingLevel);
    }

    // entry point of this class
    public void runOpMode()
    {
        IAutoCore bot = null;

        // the autonomous class should inherit from LinearOpMode
        switch (Team)
        {
            case KAOS:
                bot = new KAOS_AutoCore(this);
                break;
            case KILTS:
                bot = new KILTS_AutoCore(this);
                break;
            case KUDOS:
                bot = new KUDOS_AutoCore(this);
                break;
            case KRASH:
                bot = new KRASH_AutoCore(this);
                break;
        }
        bot.runOpMode();

    }

    public void SetWatchValue(String name, String format, Object... values)
    {
        // pass it along for now
        if (!LoggingDisabled)
        {
            try
            {
                if (!UseCoreTelemetryLogging) {
                    RobotLogger.setWatchValue(name, format, values);
                } else {
                    Telemetry.addData(name, String.format(format, values));
                }
            }
            catch (Exception ex)
            {

            }
        }
    }

    public void InitWatchValues(String... names)
    {
        // pass it along for now
        if (!LoggingDisabled)
        {
            try
            {
                if (!UseCoreTelemetryLogging) {
                    RobotLogger.initWatchValues(names);
                }
            }
            catch (Exception ex)
            {

            }
        }
    }

    public void LogInfo(String message)
    {
        if (!LoggingDisabled)
        {
            try
            {
                // getting the class name and the method name from the stack frame in which the logging call was originally made
                String[] className = Thread.currentThread().getStackTrace()[3].getClassName().split("\\.");
                String classMethodName = className[className.length - 1] + ":" +
                        Thread.currentThread().getStackTrace()[3].getMethodName() + "()";

                if (!UseCoreTelemetryLogging) {
                    RobotLogger.info(classMethodName + "[INFO]:" + message);
                } else {
                    Telemetry.addData(classMethodName + "[INFO]", message);
                }
            }
            catch (Exception ex)
            {

            }
        }
    }

    public void LogInfo(String message, boolean updateTelemetry)
    {
        if (!LoggingDisabled)
        {
            try
            {
                String[] className = Thread.currentThread().getStackTrace()[3].getClassName().split("\\.");
                String classMethodName = className[className.length - 1] + ":" +
                        Thread.currentThread().getStackTrace()[3].getMethodName() + "()";

                if (!UseCoreTelemetryLogging) {
                    RobotLogger.info(classMethodName + "[INFO]:" + message);
                } else {
                    Telemetry.addData(classMethodName + "[INFO]", message);
                    if (updateTelemetry) {
                        Telemetry.update();
                    }
                }
            }
            catch (Exception ex)
            {

            }
        }
    }

    public void LogWarning(String message)
    {
        if (!LoggingDisabled)
        {
            try
            {
                String[] className = Thread.currentThread().getStackTrace()[3].getClassName().split("\\.");
                String classMethodName = className[className.length - 1] + ":" +
                        Thread.currentThread().getStackTrace()[3].getMethodName() + "()";

                if (!UseCoreTelemetryLogging) {
                    RobotLogger.warning(classMethodName + "[WARN]:" + message);
                } else {
                    Telemetry.addData(classMethodName + "[WARN]", message);
                }
            }
            catch (Exception ex)
            {

            }
        }
    }

    public void LogWarning(String message, boolean updateTelemetry)
    {
        if (!LoggingDisabled) {
            try {
                String[] className = Thread.currentThread().getStackTrace()[3].getClassName().split("\\.");
                String classMethodName = className[className.length - 1] + ":" +
                        Thread.currentThread().getStackTrace()[3].getMethodName() + "()";

                if (!UseCoreTelemetryLogging) {
                    RobotLogger.warning(classMethodName + "[WARN]:" + message);
                } else {
                    Telemetry.addData(classMethodName + "[WARN]", message);
                    if (updateTelemetry) {
                        Telemetry.update();
                    }
                }
            } catch (Exception ex) {

            }
        }
    }

    public void LogError(String message)
    {
        if (!LoggingDisabled)
        {
            try
            {
                String[] className = Thread.currentThread().getStackTrace()[3].getClassName().split("\\.");
                String classMethodName = className[className.length - 1] + ":" +
                        Thread.currentThread().getStackTrace()[3].getMethodName() + "()";

                if (!UseCoreTelemetryLogging) {
                    RobotLogger.error(classMethodName + "[ERR]:" + message);
                } else {
                    Telemetry.addData(classMethodName + "[ERR]", message);
                }
            }
            catch (Exception ex)
            {

            }
        }
    }

    public void LogError(String message, boolean updateTelemetry)
    {
        if (!LoggingDisabled)
        {
            try
            {
                String[] className = Thread.currentThread().getStackTrace()[3].getClassName().split("\\.");
                String classMethodName = className[className.length - 1] + ":" +
                        Thread.currentThread().getStackTrace()[3].getMethodName() + "()";

                if (!UseCoreTelemetryLogging) {
                    RobotLogger.error(classMethodName + "[ERR]:" + message);
                } else {
                    Telemetry.addData(classMethodName + "[ERR]", message);
                    if (updateTelemetry) {
                        Telemetry.update();
                    }
                }
            }
            catch (Exception ex)
            {

            }
        }
    }
}
