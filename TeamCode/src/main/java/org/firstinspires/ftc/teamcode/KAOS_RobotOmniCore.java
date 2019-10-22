package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

// this class represents an instance of the robot and all of the hardware
// and sensors that make up the robot
public class KAOS_RobotOmniCore
{
    // public variables that represent the main drive motors
    public DcMotor leftRearMotor = null;
    public DcMotor rightRearMotor = null;
    public DcMotor leftFrontMotor = null;
    public DcMotor rightFrontMotor = null;

    public DcMotor elevatorMotor = null;
    public DcMotor mineralMotor = null;
    public DcMotor extenderMotor = null;
    public DcMotor liftMotor = null;

    public DigitalChannel upperLimitTouchSensor;
    public DigitalChannel lowerLimitTouchSensor;

    public Servo markerServo;
    public Servo brakeServo;

    // private constants that represent the names of the devices as
    // programmed in to the configuration on the robot controller
    private final String LEFT_REAR_MOTOR = "left_rear";
    private final String LEFT_FRONT_MOTOR = "left_front";
    private final String RIGHT_REAR_MOTOR = "right_rear";
    private final String RIGHT_FRONT_MOTOR = "right_front";

    private final String IMU_GYRO = "imu";
    private final String UPPER_LIMIT_TOUCH_SENSOR = "upper_limit_touch_sensor";
    private final String LOWER_LIMIT_TOUCH_SENSOR = "lower_limit_touch_sensor";
    private final String BRAKE_SERVO = "brake_servo";
    private final String MARKER_SERVO = "marker_servo";
    private final String ELEVATOR_MOTOR = "elevator_motor";
    private final String MINERAL_MOTOR = "mineral_motor";
    private final String EXTENDER_MOTOR = "extender_motor";
    private final String LIFT_MOTOR = "lift_motor";

    // private references that need to be kept locally
    private OpModeConfig m_opModeConfig;
    private ElapsedTime m_elapsedTime = new ElapsedTime();
    public boolean m_useGryo = false;
    public boolean m_gyroInitialized = false;

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double HEADING_THRESHOLD = 1 ; // As tight as we can make it with an integer gyro
    public double P_TURN_COEFF = 0.08;    // Larger is more responsive, but also less stable (default was .09)
    //public double P_DRIVE_COEFF = 0.15;   // Larger is more responsive, but also less stable (default was .15)

    BNO055IMU Gyro = null;
    Orientation GyroAngles = null;
    Acceleration GyroGravity = null;
    BNO055IMU.Parameters GyroParameters = null;

    static final double MEC_DRIVE_CARDINAL_COUNTS_PER_INCH = 44;

    static final double BRAKE_SERVO_UNLOCK = .20;
    static final double BRAKE_SERVO_LOCK = .75;

    // defines the various ways that the robot can easily move in
//    public enum AUTO_DRIVE_DIRECTION
//    {
//        FORWARD,
//        REVERSE,
//        LEFT,
//        RIGHT,
//        DIAG_FORWARD_RIGHT,
//        DIAG_FORWARD_LEFT,
//        DIAG_REVERSE_RIGHT,
//        DIAG_REVERSE_LEFT
//    }
//
//    // define the directions that mineral vision can go
//    public enum MEC_DRIVE_AUTO_DIRECTION
//    {
//        UNKNOWN,
//        CENTER,
//        LEFT,
//        RIGHT
//    }

    public enum MEC_DRIVE_AUTO_DIRECTION
    {
        FORWARD,
        REVERSE,
        LEFT,
        RIGHT,
        DIAG_FORWARD_RIGHT,
        DIAG_FORWARD_LEFT,
        DIAG_REVERSE_RIGHT,
        DIAG_REVERSE_LEFT
    }

    public enum MINERAL_VISION_DIRECTIONS
    {
        UNKNOWN,
        CENTER,
        LEFT,
        RIGHT
    }

    public GoldAlignDetector detector;
    public boolean detectorIsEnabled = false;
    public double virtualCenterPos = 50;  //was 85

    public enum LAST_SEARCH
    {
        FIRST,
        SECOND,
        NONE
    }


    public KAOS_RobotOmniCore(OpModeConfig opModeConfig)
    {
        // we must know which m_robot we are working with
        m_opModeConfig = opModeConfig;
    }

    public boolean init()
    {
        boolean result = true;
        m_opModeConfig.Telemetry.addData(">", "Init:OnEntry");
        m_opModeConfig.Telemetry.update();


        // do we know which m_robot we are?
        if (m_opModeConfig.Team == OpModeConfig.ROBOT_TEAM.UNKNOWN)
        {
            m_opModeConfig.Telemetry.addData(">", "Team is not set, are we KAOS or KILTS?");
            return false;
        }

        // do we have a reference to the opMode?
        if (m_opModeConfig.OpMode == null)
        {
            m_opModeConfig.Telemetry.addData(">", "Reference to OpMode is null.");
            return false;
        }

        // do we have a valid reference to the Hardware map?
        if (m_opModeConfig.OpMode.hardwareMap == null)
        {
            m_opModeConfig.Telemetry.addData(">", "Reference to OpMode.HardwareMap is null.");
            //return false;
        }
        // initialize the drive motors
        try
        {
            leftRearMotor = m_opModeConfig.OpMode.hardwareMap.get(DcMotor.class, LEFT_REAR_MOTOR);
            rightRearMotor = m_opModeConfig.OpMode.hardwareMap.get(DcMotor.class, RIGHT_REAR_MOTOR);
            leftFrontMotor = m_opModeConfig.OpMode.hardwareMap.get(DcMotor.class, LEFT_FRONT_MOTOR);
            rightFrontMotor = m_opModeConfig.OpMode.hardwareMap.get(DcMotor.class, RIGHT_FRONT_MOTOR);

            TrySetDriveMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            TrySetDriveMotorsRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
            rightRearMotor.setDirection(DcMotor.Direction.FORWARD);
            leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
            rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);

            // set all the drive motors to zero power
            TrySetDriveMotorsPower(0);

            // set all motors to run without encoders for now - this is faster
            TrySetDriveMotorsRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        catch (Exception ex)
        {
            // something went wrong
            m_opModeConfig.Telemetry.addData(">", "Initializing Drive Motors Exception: " + ex.getMessage() );
            result = false;
        }

        // initialize the elevator motors
        m_opModeConfig.LogInfo("Init elevator motors...");
        try
        {
            elevatorMotor = m_opModeConfig.OpMode.hardwareMap.get(DcMotor.class, ELEVATOR_MOTOR);

            // set the lift motor to zero power and brake mode
            elevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            elevatorMotor.setPower(0);

            // set default direction of motors
            elevatorMotor.setDirection(DcMotor.Direction.FORWARD);
        }
        catch (Exception ex)
        {
            // something went wrong
            m_opModeConfig.LogError("Initializing elevator motor Exception: " + ex.getMessage());
            //result = false;
        }

        m_opModeConfig.LogInfo("Init lift motors...");
        try
        {
            liftMotor = m_opModeConfig.OpMode.hardwareMap.get(DcMotor.class, LIFT_MOTOR);

            // set the lift motor to zero power and brake mode
            liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftMotor.setPower(0);

            // set default direction of motors
            liftMotor.setDirection(DcMotor.Direction.FORWARD);
        }
        catch (Exception ex)
        {
            // something went wrong
            m_opModeConfig.LogError("Initializing lift motor Exception: " + ex.getMessage());
            //result = false;
        }

        m_opModeConfig.LogInfo("Init mineral motors...");
        try
        {
            mineralMotor = m_opModeConfig.OpMode.hardwareMap.get(DcMotor.class, MINERAL_MOTOR);

            // set the lift motor to zero power and brake mode
            mineralMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mineralMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            mineralMotor.setPower(0);

            // set default direction of motors
            mineralMotor.setDirection(DcMotor.Direction.FORWARD);
        }
        catch (Exception ex)
        {
            // something went wrong
            m_opModeConfig.LogError("Initializing mineral motor Exception: " + ex.getMessage());
            //result = false;
        }

        m_opModeConfig.LogInfo("Init extender motors...");
        try
        {
            extenderMotor = m_opModeConfig.OpMode.hardwareMap.get(DcMotor.class, EXTENDER_MOTOR);

            // set the lift motor to zero power and brake mode
            extenderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            extenderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            extenderMotor.setPower(0);

            // set default direction of motors
            extenderMotor.setDirection(DcMotor.Direction.FORWARD);
        }
        catch (Exception ex)
        {
            // something went wrong
            m_opModeConfig.LogError("Initializing extender motor Exception: " + ex.getMessage());
            //result = false;
        }

        try
        {
            upperLimitTouchSensor = m_opModeConfig.OpMode.hardwareMap.get(DigitalChannel.class, UPPER_LIMIT_TOUCH_SENSOR);
            upperLimitTouchSensor.setMode(DigitalChannel.Mode.INPUT);
        }
        catch (Exception ex)
        {
            // something went wrong
            m_opModeConfig.LogError("Initializing upper lift touch sensors Exception: " + ex.getMessage());
            upperLimitTouchSensor = null;
        }

        try
        {
            lowerLimitTouchSensor = m_opModeConfig.OpMode.hardwareMap.get(DigitalChannel.class, LOWER_LIMIT_TOUCH_SENSOR);
            lowerLimitTouchSensor.setMode(DigitalChannel.Mode.INPUT);
        }
        catch (Exception ex)
        {
            // something went wrong
            m_opModeConfig.LogError("Initializing lower lift touch sensors Exception: " + ex.getMessage());
            lowerLimitTouchSensor = null;
        }

        try
        {
            markerServo = m_opModeConfig.OpMode.hardwareMap.get(Servo.class, MARKER_SERVO);
        }
        catch (Exception ex)
        {
            // something went wrong
            m_opModeConfig.LogError("Initializing marker servo Exception: " + ex.getMessage());
            markerServo = null;
        }

        try
        {
            brakeServo = m_opModeConfig.OpMode.hardwareMap.get(Servo.class, BRAKE_SERVO);
            brakeServo.setDirection(Servo.Direction.REVERSE);
        }
        catch (Exception ex)
        {
            // something went wrong
            m_opModeConfig.LogError("Initializing brake servo Exception: " + ex.getMessage());
            brakeServo = null;
        }

        //initialize vision system
        m_opModeConfig.LogInfo("Init mineral vision...");
        try
        {
            //initialize DogeCV
            detector = new GoldAlignDetector();
            detector.init(m_opModeConfig.OpMode.hardwareMap.appContext, CameraViewDisplay.getInstance());
            detector.useDefaults();

            // Optional Tuning
            detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
            detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
            detector.downscale = 0.4; // How much to downscale the input frames

            detector.areaScoringMethod = DogeCV.AreaScoringMethod.PERFECT_AREA; // Can also be PERFECT_AREA
            detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
            detector.maxAreaScorer.weight = 0.005;

            detector.ratioScorer.weight = 5;
            detector.ratioScorer.perfectRatio = 1.0;

            //Phone in portrait = false, landscape = true.  TRUE IS NOT WORKING.  Needs to move to team config
            detector.rotateMat = false;
        }
        catch (Exception ex)
        {
            // something went wrong
            m_opModeConfig.LogError("Initializing mineral vision Exception: " + ex.getMessage());
            result = false;
        }

        // initialize the imu gyro
        try
        {
            Gyro = m_opModeConfig.OpMode.hardwareMap.get(BNO055IMU.class, IMU_GYRO);
        }
        catch (Exception ex)
        {
            // something went wrong
            m_opModeConfig.Telemetry.addData(">", "Initializing IMU Gyro Exception: " + ex.getMessage() );
            result = false;
        }
        return  result;
    }

    public void InitializeGryo()
    {
        // initialize the gyro
        m_opModeConfig.LogInfo("Entry");
        if (Gyro == null)
        {
            m_opModeConfig.LogError("Gyro is null in InitializeGyro()");
        }
        else
        {
            // finish intializing the gyro
            m_opModeConfig.Telemetry.addData(">", "Preparing gyro parameters...");
            GyroParameters = new BNO055IMU.Parameters();
            GyroParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            GyroParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            GyroParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            GyroParameters.loggingEnabled = true;
            GyroParameters.loggingTag = "IMU";
            GyroParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            m_opModeConfig.LogInfo("Initializing gyro...");
            Gyro.initialize(GyroParameters);

            m_opModeConfig.LogInfo("Starting acceleration integration on gyro...");
            Gyro.startAccelerationIntegration(new Position(), new Velocity(), 1000);

            m_opModeConfig.LogInfo("Composing telemetry...");
            ComposeGyroTelemetry();
            m_gyroInitialized = true;

        }
    }

    public void EnableMineralVision()
    {
        if (detector != null && !detectorIsEnabled)
        {
            detector.enable();
            detectorIsEnabled = true;
        }
    }

    public void DisableMineralVision()
    {
        if (detector != null && detectorIsEnabled)
        {
            detector.disable();
            detectorIsEnabled = false;
        }
    }

//    public MEC_DRIVE_AUTO_DIRECTION KAOS_FindMineralWithStrafe(int strafeSearchTime, double minStrafePower, double maxStrafePower,
//                                                               double timeout, KAOS_TeleOp_OmniDrive.ROBOT_FRONT_MODE frontMode) {
//        m_opModeConfig.LogInfo("Entry");
//
//        double xpos;
//        double strafePower = 0;
//        double MIN_STRAFE_PWR = minStrafePower;
//        double MAX_STRAFE_PWR = maxStrafePower;
//        final double MAX_ALIGN_LIMIT = 200;
//        final double MIN_ALIGNED = 30;
//        final int STRAFE_TIME = 100;  //milliseconds
//        int STRAFE_SEARCH = strafeSearchTime; //milliseconds
//        final double CENTER_RANGE = 5; //+- inches for gold mineral at center
//
//
//        LAST_SEARCH lastSearch = LAST_SEARCH.NONE;
//
//
//        // check to make sure enabled, if not error
//        if (!detectorIsEnabled)
//        {
//            m_opModeConfig.LogError("No active GoldAlignDetector for KAOS_FindMineralWithStrafe()");
//            return MEC_DRIVE_AUTO_DIRECTION.UNKNOWN;
//        } else
//        {
//            TrySetDriveMotorsRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            TrySetDriveMotorsRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//            m_elapsedTime.reset();
//            while (m_opModeConfig.OpMode.opModeIsActive() && (m_elapsedTime.seconds() < timeout))
//            {
//                if (detector.isFound())
//                {
//                    // Mineral is detected.  Determine offset from center and set power to strafe to align
//                    TrySetDriveMotorsRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    m_opModeConfig.LogInfo("Found Mineral, Calculating Strafe");
//                    xpos = detector.getXPosition() - detector.getAdjustedSize().width / 2;
//                    m_opModeConfig.LogInfo("xpos = " + xpos);
//                    strafePower = Math.signum(xpos) * Range.clip(MAX_STRAFE_PWR * Math.abs(xpos) / MAX_ALIGN_LIMIT, MIN_STRAFE_PWR, MAX_STRAFE_PWR);
//
//                    if (Math.abs(xpos) > MIN_ALIGNED && detector.isFound())
//                    {
//                        //Outside the align limits so strafe towards mineral for STRAFE_TIME and recheck
//                        m_opModeConfig.LogInfo("Outside window.  Strafing with power " + strafePower);
//                        TeleOpStrafeDrive(0, 0, strafePower, 1, frontMode);
//                        lastSearch = LAST_SEARCH.NONE;
//                        m_opModeConfig.OpMode.sleep(STRAFE_TIME);
//                    } else
//                    {
//                        //Mineral is within align limits.  Stop strafing, calculate final displacement, and return location
//                        m_opModeConfig.LogInfo("Within alignment limits. Stopping");
//                        StopRobot();
//
//                        //NEED TO CONFIRM if encoder sign is correct for the statements below.  Might need leftFrontMotor instead
//                        double encoderPos = (leftFrontMotor.getCurrentPosition()-rightFrontMotor.getCurrentPosition()) / 2 / MEC_DRIVE_CARDINAL_COUNTS_PER_INCH;
//                        m_opModeConfig.LogInfo("Encoder Position = " + encoderPos);
//
//                        if (encoderPos < -CENTER_RANGE)
//                        {
//                            return MEC_DRIVE_AUTO_DIRECTION.LEFT;
//                        } else if (encoderPos > CENTER_RANGE)
//                        {
//                            return MEC_DRIVE_AUTO_DIRECTION.RIGHT;
//                        } else
//                        {
//                            return MEC_DRIVE_AUTO_DIRECTION.CENTER;
//                        }
//                    }
//                } else
//                {
//                    //Mineral is not detected.  Try strafing to find it.
//                    m_opModeConfig.LogInfo("Mineral not detected");
//                    switch (lastSearch)
//                    {
//                        case NONE:
//                            //Haven't strafed yet or lost mineral while strafing.  Strafe right to look for gold mineral
//                            m_opModeConfig.LogInfo("Have not searched yet.  Strafe right.");
//                            StopRobot();
//                            m_opModeConfig.OpMode.sleep(100);
//
//                            //Use timed strafe
//                            TeleOpStrafeDrive(0, 0, MAX_STRAFE_PWR, 1, frontMode);
//                            m_opModeConfig.OpMode.sleep(STRAFE_SEARCH);
//                            StopRobot();
//
//                            //AutonomousDrive(MEC_DRIVE_AUTO_DIRECTION.RIGHT, 12, MAX_STRAFE_PWR, 10);
//
//                            lastSearch = LAST_SEARCH.RIGHT;
//                            break;
//
//                        case RIGHT:
//                            //Tried strafing right.  Stop strafing and pause to reduce wheel slippage then strafe left
//                            m_opModeConfig.LogInfo("Tried strafing right. Check Left");
//                            StopRobot();
//                            m_opModeConfig.OpMode.sleep(100);
//
//                            //Use timed strafe
//                            TeleOpStrafeDrive(0, 0, -MAX_STRAFE_PWR, 1, frontMode);
//                            m_opModeConfig.OpMode.sleep(2 * STRAFE_SEARCH);
//                            StopRobot();
//
//                            //AutonomousDrive(MEC_DRIVE_AUTO_DIRECTION.LEFT, 24, MAX_STRAFE_PWR, 10);
//                            lastSearch = LAST_SEARCH.LEFT;
//                            break;
//
//
//                        case LEFT:
//                            //Already tried both directions and could not locate mineral
//                            m_opModeConfig.LogInfo("tried both directions and could not locate");
//                            StopRobot();
//                            m_opModeConfig.OpMode.sleep(100);
//
//
//                            double displacement = (leftFrontMotor.getCurrentPosition() - rightFrontMotor.getCurrentPosition()) / 2 / MEC_DRIVE_CARDINAL_COUNTS_PER_INCH;
//                            AUTO_DRIVE_DIRECTION dispDirection;
//
//                            m_opModeConfig.LogInfo("Moved " + displacement + "inches");
//
//                            //If we moved more than 2 inches, return to starting position
//                            if (displacement > 2) {
//                                //We moved right, so move left to return to start
//                                m_opModeConfig.LogInfo("We moved right, so move left to return to start");
//                                dispDirection = AUTO_DRIVE_DIRECTION.LEFT;
//                                AutonomousDrive(dispDirection, Math.abs(displacement), MAX_STRAFE_PWR, 2);
//                            } else if (displacement < -2) {
//                                //We moved left, so move right to return to start
//                                m_opModeConfig.LogInfo("We moved left, so move right to return to start");
//                                dispDirection = AUTO_DRIVE_DIRECTION.RIGHT;
//                                AutonomousDrive(dispDirection, Math.abs(displacement), MAX_STRAFE_PWR, 2);
//                            }
//
//                            return MEC_DRIVE_AUTO_DIRECTION.UNKNOWN;
//
//                    }
//                }
//            }
//            //Reached timeout or OpMode stopped.  Return Unknown
//            return MEC_DRIVE_AUTO_DIRECTION.UNKNOWN;
//        }
//    }

    public MINERAL_VISION_DIRECTIONS FindMineralWithStrafe(double timeout) {
        m_opModeConfig.LogInfo("Entry");

        double xpos;
        double strafePower = 0;
        final double MIN_STRAFE_PWR = 0.35;
        final double MAX_STRAFE_PWR = 0.55;
        final double MAX_ALIGN_LIMIT = 200;
        final double MIN_ALIGNED = 25;
        final int STRAFE_TIME = 100;  //milliseconds
        final double STRAFE_SEARCH = 1.5; //seconds
        final double CENTER_RANGE = 3.5; //+- inches for gold mineral at center
        LAST_SEARCH lastSearch = LAST_SEARCH.NONE;

        // check to make sure enabled, if not error
        if (!detectorIsEnabled)
        {
            m_opModeConfig.LogError("No active GoldAlignDetector for FindMineralWithStrafe()");
            return MINERAL_VISION_DIRECTIONS.UNKNOWN;
        }
        else
        {
            TrySetDriveMotorsRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            TrySetDriveMotorsRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            m_elapsedTime.reset();
            while (m_opModeConfig.OpMode.opModeIsActive() && (m_elapsedTime.seconds() < timeout))
            {
                if (detector.isFound())
                {
                    // Mineral is detected.  Determine offset from center and set power to strafe to align
                    TrySetDriveMotorsRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    m_opModeConfig.LogInfo("Found Mineral, Calculating final correction strafe...");

                    // determine which side of the center line the mineral has been detected on so we know which way we need to move
                    //xpos = detector.getXPosition() - (detector.getAdjustedSize().width / 2);
                    xpos = detector.getXPosition() - ((detector.getAdjustedSize().width / 100) * virtualCenterPos);
                    m_opModeConfig.LogInfo("xpos = " + xpos);

                    // determine how much power we should user (and which direction we should go) in order to get to the mineral
                    strafePower = Math.signum(xpos) * Range.clip(MAX_STRAFE_PWR * Math.abs(xpos) / MAX_ALIGN_LIMIT, MIN_STRAFE_PWR, MAX_STRAFE_PWR);
                    //  (Math.signum(xpos) returns a +1 or -1 depending on the sign of xpos)

                    // if the abs value of xpos is greater than the MIN_ALIGNED, the mineral is too far from the center of the detector/screen
                    if (Math.abs(xpos) > MIN_ALIGNED && detector.isFound())
                    {
                        //Outside the align limits so strafe towards mineral for STRAFE_TIME and recheck
                        m_opModeConfig.LogInfo("Outside window.  Strafing with power " + strafePower);
                        TeleOpStrafeDrive(0, 0, strafePower, .8, KAOS_TeleOp_OmniDrive.ROBOT_FRONT_MODE.PHONE);
                        lastSearch = LAST_SEARCH.NONE;
                        m_opModeConfig.OpMode.sleep(STRAFE_TIME);
                    } else
                    {
                        //Mineral is within align limits.  Stop strafing, calculate final displacement, and return location of mineral
                        m_opModeConfig.LogInfo("Within alignment limits. Stopping");
                        StopRobot();

                        //NEED TO CONFIRM if encoder sign is correct for the statements below.  Might need leftFrontMotor instead
                        double encoderPos = (leftFrontMotor.getCurrentPosition() - rightFrontMotor.getCurrentPosition()) / 2 / MEC_DRIVE_CARDINAL_COUNTS_PER_INCH;
                        m_opModeConfig.LogInfo("Encoder Position = " + encoderPos);

                        // this tells the caller which way we had to go in order to find the mineral (not used anywhere else in this routine)
                        if (encoderPos < -CENTER_RANGE + 2)
                        {
                            StrafeRight(3.5, .6, 1);
                            return MINERAL_VISION_DIRECTIONS.LEFT;
                        }
                        else if (encoderPos > CENTER_RANGE + 7)
                        {
                            return MINERAL_VISION_DIRECTIONS.RIGHT;
                        }
                        else
                        {
                            return MINERAL_VISION_DIRECTIONS.CENTER;
                        }
                    }
                } else
                {
                    //Mineral is not detected.  Try strafing to find it.
                    double startTime = m_elapsedTime.seconds();
                    m_opModeConfig.LogInfo("Mineral not detected");
                    switch (lastSearch)
                    {
                        case NONE:
                            //Haven't strafed yet or lost mineral while strafing.  Strafe right to look for gold mineral
                            m_opModeConfig.LogInfo("Have not searched yet.  Strafe right.");
                            StopRobot();
                            m_opModeConfig.OpMode.sleep(100);

                            //Use timed strafe
                            m_opModeConfig.LogInfo("Initiating strafe");
                            while (!detector.isFound() && m_elapsedTime.seconds()-startTime < STRAFE_SEARCH)
                            {
                                TeleOpStrafeDrive(0, 0, MAX_STRAFE_PWR, .8, KAOS_TeleOp_OmniDrive.ROBOT_FRONT_MODE.PHONE);
                            }
                            m_opModeConfig.LogInfo("Stop strafe");
                            StopRobot();
                            lastSearch = LAST_SEARCH.FIRST;
                            break;
                        case FIRST:
                            //Tried strafing right.  Stop strafing and pause to reduce wheel slippage then strafe left
                            m_opModeConfig.LogInfo("Tried strafing right. Check Left");
                            StopRobot();
                            m_opModeConfig.OpMode.sleep(50);

                            //Use timed strafe
                            m_opModeConfig.LogInfo("Initiating strafe");
                            while (!detector.isFound() && m_elapsedTime.seconds()-startTime < 2 * STRAFE_SEARCH)
                            {
                                TeleOpStrafeDrive(0, 0, -MAX_STRAFE_PWR, .8, KAOS_TeleOp_OmniDrive.ROBOT_FRONT_MODE.PHONE);
                            }
                            m_opModeConfig.LogInfo("Stop strafe");
                            StopRobot();
                            lastSearch = LAST_SEARCH.SECOND;
                            break;
                        case SECOND:
                            //Already tried both directions and could not locate mineral
                            m_opModeConfig.LogInfo("tried both directions and could not locate");
                            StopRobot();
                            m_opModeConfig.OpMode.sleep(100);

                            double displacement = (leftFrontMotor.getCurrentPosition() - rightFrontMotor.getCurrentPosition()) / 2 / MEC_DRIVE_CARDINAL_COUNTS_PER_INCH;
                            MEC_DRIVE_AUTO_DIRECTION dispDirection;

                            m_opModeConfig.LogInfo("Moved " + displacement + "inches");

                            //If we moved more than 2 inches, return to starting position
                            if (displacement > 2) {
                                //We moved right, so move left to return to start
                                m_opModeConfig.LogInfo("We moved right, so move left to return to start");
                                dispDirection = MEC_DRIVE_AUTO_DIRECTION.LEFT;
                                AutonomousDrive(dispDirection, Math.abs(displacement), MAX_STRAFE_PWR, 2);
                            } else if (displacement < -2) {
                                //We moved left, so move right to return to start
                                m_opModeConfig.LogInfo("We moved left, so move right to return to start");
                                dispDirection = MEC_DRIVE_AUTO_DIRECTION.RIGHT;
                                AutonomousDrive(dispDirection, Math.abs(displacement), MAX_STRAFE_PWR, 2);
                            }

                            return MINERAL_VISION_DIRECTIONS.UNKNOWN;
                    }
                }
            }
            //Reached timeout or OpMode stopped.  Return Unknown
            return MINERAL_VISION_DIRECTIONS.UNKNOWN;
        }
    }

    public MINERAL_VISION_DIRECTIONS FindMineralWithStrafe(MINERAL_VISION_DIRECTIONS startingDirection, double timeout) {
        m_opModeConfig.LogInfo("Entry");

        double xpos;
        double strafePower = 0;
        final double MIN_STRAFE_PWR = 0.33;
        final double MAX_STRAFE_PWR = 0.55;
        final double MAX_ALIGN_LIMIT = 200;
        final double MIN_ALIGNED = 25;
        final int STRAFE_TIME = 100;  //milliseconds
        final double STRAFE_SEARCH = 1.5; //Seconds
        final double CENTER_RANGE = 3.5;

        LAST_SEARCH lastSearch = LAST_SEARCH.NONE;


        // check to make sure enabled, if not error
        if (!detectorIsEnabled)
        {
            m_opModeConfig.LogError("No active GoldAlignDetector for FindMineralWithStrafe()");
            return MINERAL_VISION_DIRECTIONS.UNKNOWN;
        } else
        {
            TrySetDriveMotorsRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            TrySetDriveMotorsRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            m_elapsedTime.reset();
            while (m_opModeConfig.OpMode.opModeIsActive() && (m_elapsedTime.seconds() < timeout))
            {
                if (detector.isFound())
                {
                    // Mineral is detected.  Determine offset from center and set power to strafe to align
                    TrySetDriveMotorsRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    m_opModeConfig.LogInfo("Found Mineral, Calculating Strafe");
                    //xpos = detector.getXPosition() - detector.getAdjustedSize().width / 2;
                    xpos = detector.getXPosition() - ((detector.getAdjustedSize().width / 100) * virtualCenterPos);
                    m_opModeConfig.LogInfo("xpos = " + xpos);
                    strafePower = Math.signum(xpos) * Range.clip(MAX_STRAFE_PWR * Math.abs(xpos) / MAX_ALIGN_LIMIT, MIN_STRAFE_PWR, MAX_STRAFE_PWR);

                    if (Math.abs(xpos) > MIN_ALIGNED && detector.isFound())
                    {
                        //Outside the align limits so strafe towards mineral for STRAFE_TIME and recheck
                        m_opModeConfig.LogInfo("Outside window.  Strafing with power " + strafePower);
                        TeleOpStrafeDrive(0, 0, strafePower, .8, KAOS_TeleOp_OmniDrive.ROBOT_FRONT_MODE.PHONE);
                        lastSearch = LAST_SEARCH.NONE;
                        m_opModeConfig.OpMode.sleep(STRAFE_TIME);
                    } else
                    {
                        //Mineral is within align limits.  Stop strafing, calculate final displacement, and return location
                        m_opModeConfig.LogInfo("Within alignment limits. Stopping");
                        StopRobot();

                        //NEED TO CONFIRM if encoder sign is correct for the statements below.  Might need leftFrontMotor instead
                        double encoderPos = (leftFrontMotor.getCurrentPosition()-rightFrontMotor.getCurrentPosition()) / 2 / MEC_DRIVE_CARDINAL_COUNTS_PER_INCH;
                        m_opModeConfig.LogInfo("Encoder Position = " + encoderPos);

                        if (encoderPos < -CENTER_RANGE + 2)
                        {
                            StrafeRight(3.5, .6, 1);
                            return MINERAL_VISION_DIRECTIONS.LEFT;
                        }
                        else if (encoderPos > CENTER_RANGE + 7)
                        {
                            return MINERAL_VISION_DIRECTIONS.RIGHT;
                        }
                        else
                        {
                            return MINERAL_VISION_DIRECTIONS.CENTER;
                        }
                    }
                } else
                {
                    //Mineral is not detected.  Try strafing to find it.
                    double startTime = m_elapsedTime.seconds();
                    double searchPwr = MAX_STRAFE_PWR;

                    m_opModeConfig.LogInfo("Mineral not detected");
                    switch (lastSearch)
                    {
                        case NONE:
                            //Haven't strafed yet or lost mineral while strafing.  Strafe startDirection to look for mineral
                            m_opModeConfig.LogInfo("Have not searched yet.  Strafe startDirection.");
                            StopRobot();
                            m_opModeConfig.OpMode.sleep(50);

                            //Use timed strafe
                            if (startingDirection == MINERAL_VISION_DIRECTIONS.LEFT){
                                searchPwr = -MAX_STRAFE_PWR;
                            } else {
                                searchPwr = MAX_STRAFE_PWR;
                            }
                            m_opModeConfig.LogInfo("Initiating strafe");
                            while (!detector.isFound() && m_elapsedTime.seconds()-startTime < STRAFE_SEARCH) {
                                TeleOpStrafeDrive(0, 0, searchPwr, .8, KAOS_TeleOp_OmniDrive.ROBOT_FRONT_MODE.PHONE);
                            }
                            m_opModeConfig.LogInfo("Stop strafe");
                            StopRobot();

                            lastSearch = LAST_SEARCH.FIRST;

                            break;

                        case FIRST:
                            //Tried strafing startDirection.  Stop strafing and pause to reduce wheel slippage then strafe opposite
                            m_opModeConfig.LogInfo("Tried strafing startDirection. Check opposite");
                            StopRobot();
                            m_opModeConfig.OpMode.sleep(50);

                            //Use timed strafe
                            if (startingDirection == MINERAL_VISION_DIRECTIONS.LEFT){
                                searchPwr = MAX_STRAFE_PWR;
                            } else {
                                searchPwr = -MAX_STRAFE_PWR;
                            }
                            m_opModeConfig.LogInfo("Initiating strafe");
                            while (!detector.isFound() && m_elapsedTime.seconds()-startTime < 2 * STRAFE_SEARCH) {
                                TeleOpStrafeDrive(0, 0, searchPwr, .8, KAOS_TeleOp_OmniDrive.ROBOT_FRONT_MODE.PHONE);
                            }
                            m_opModeConfig.LogInfo("Stop strafe");
                            StopRobot();

                            //AutonomousDrive(MEC_DRIVE_AUTO_DIRECTION.LEFT, 24, MAX_STRAFE_PWR, 10);
                            lastSearch = LAST_SEARCH.SECOND;
                            break;


                        case SECOND:
                            //Already tried both directions and could not locate mineral
                            m_opModeConfig.LogInfo("tried both directions and could not locate");
                            StopRobot();
                            m_opModeConfig.OpMode.sleep(50);

                            double displacement = (leftFrontMotor.getCurrentPosition() - rightFrontMotor.getCurrentPosition()) / 2 / MEC_DRIVE_CARDINAL_COUNTS_PER_INCH;
                            MEC_DRIVE_AUTO_DIRECTION dispDirection;

                            m_opModeConfig.LogInfo("Moved " + displacement + "inches");

                            //If we moved more than 2 inches, return to starting position
                            if (displacement > 2) {
                                //We moved right, so move left to return to start
                                m_opModeConfig.LogInfo("We moved right, so move left to return to start");
                                dispDirection = MEC_DRIVE_AUTO_DIRECTION.LEFT;
                                AutonomousDrive(dispDirection, Math.abs(displacement), MAX_STRAFE_PWR, 2);
                            } else if (displacement < -2) {
                                //We moved left, so move right to return to start
                                m_opModeConfig.LogInfo("We moved left, so move right to return to start");
                                dispDirection = MEC_DRIVE_AUTO_DIRECTION.RIGHT;
                                AutonomousDrive(dispDirection, Math.abs(displacement), MAX_STRAFE_PWR, 2);
                            }

                            return MINERAL_VISION_DIRECTIONS.UNKNOWN;
                    }
                }
            }
            //Reached timeout or OpMode stopped.  Return Unknown
            return MINERAL_VISION_DIRECTIONS.UNKNOWN;
        }
    }

    public MINERAL_VISION_DIRECTIONS FindMineral() {
        m_opModeConfig.LogInfo("Entry");

        double xpos;
        double rightThreshold = 100;
        double leftThreshold = -100;

        // check to make sure enabled, if not error
        if (!detectorIsEnabled)
        {
            m_opModeConfig.LogError("No active GoldAlignDetector for FindMineralWithStrafe()");
            return MINERAL_VISION_DIRECTIONS.UNKNOWN;
        } else
        {
            MINERAL_VISION_DIRECTIONS retVal;
            xpos = detector.getXPosition() - ((detector.getAdjustedSize().width / 100) * virtualCenterPos);
           /* This section can be used to evaluate mineral location values
            m_opModeConfig.LogInfo("raw = " + detector.getXPosition());
            m_opModeConfig.LogInfo("width = " + detector.getAdjustedSize().width);
            m_opModeConfig.LogInfo("xpos = " + xpos);
            m_opModeConfig.LogInfo("ypos = " + detector.getYPosition());
            m_opModeConfig.OpMode.sleep(20000);*/
            if (detector.isFound())
            {
//                if (xpos < leftThreshold)
//                {
//                    retVal = MINERAL_VISION_DIRECTIONS.LEFT;
//                } else if (xpos > rightThreshold)
//                {
//                    retVal = MINERAL_VISION_DIRECTIONS.RIGHT;
//                } else
//                    {
//                    retVal = MINERAL_VISION_DIRECTIONS.CENTER;
//                }

                // chances are we will only be able to see the left and center minerals, not the right
                if (xpos < leftThreshold)
                {
                    retVal = MINERAL_VISION_DIRECTIONS.LEFT;
                }
                else if (xpos > rightThreshold)
                {
                    retVal = MINERAL_VISION_DIRECTIONS.RIGHT;
                }
                else
                {
                    retVal = MINERAL_VISION_DIRECTIONS.CENTER;
                }
            }
            else
            {
                retVal = MINERAL_VISION_DIRECTIONS.UNKNOWN;
            }
            return retVal;
        }
    }

    public void TeleOpStrafeDrive(double left_stick_y_drive, double right_stick_x_turn, double left_stick_x_strafe,
                                  double appliedPower, KAOS_TeleOp_OmniDrive.ROBOT_FRONT_MODE frontMode)
    {
        // make sure the applied power is within the expected range & has to be positive (.01 - 1.00)
        appliedPower = Math.abs(Range.clip(appliedPower, -1.0, 1.0));

        // applied the power to the raw inputs before any additional scaling is
        double drive = -left_stick_y_drive * appliedPower;
        double strafe = left_stick_x_strafe * appliedPower;
        double turn = 0.0;
        switch (frontMode)
        {
            case PHONE:
                turn = right_stick_x_turn * appliedPower;
                break;
            case INTAKE:
                turn = -right_stick_x_turn * appliedPower;  // invert sign
                break;
            case LIFT_BASE:
                turn = -right_stick_x_turn * appliedPower;  // invert sign
                break;
            case ELEVATOR_BASE:
                turn = right_stick_x_turn * appliedPower;
                break;
        }

        // do the math to get the correct power for the correct wheels
        double leftFrontPower = drive + turn + strafe;
        double rightFrontPower = drive - turn - strafe;
        double leftRearPower = drive + turn - strafe;
        double rightRearPower = drive - turn + strafe;

        // scale the power so that it never exceeds the accepted range of the motors
        double scaleFactor = RobotUtil.ScalePower(leftFrontPower, rightFrontPower, leftRearPower, rightRearPower);
        leftFrontPower = leftFrontPower * scaleFactor;
        rightFrontPower = rightFrontPower * scaleFactor;
        leftRearPower = leftRearPower * scaleFactor;
        rightRearPower = rightRearPower * scaleFactor;

        // apply the adjusted power to the motors
        switch (frontMode)
        {
            case PHONE:
                // (the way we started - unchanged, DO NOT CHANGE)
                leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
                rightRearMotor.setDirection(DcMotor.Direction.FORWARD);
                leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
                rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
                TrySetDriveMotorsPower(rightFrontPower, rightRearPower, leftFrontPower, leftRearPower);
                break;
            case INTAKE:
                // the side that the intake rests when down
                leftRearMotor.setDirection(DcMotor.Direction.FORWARD);
                rightRearMotor.setDirection(DcMotor.Direction.FORWARD);
                leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
                rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
                TrySetDriveMotorsPower(rightRearPower, leftRearPower, rightFrontPower, leftFrontPower);
                break;
            case LIFT_BASE:
                // the side that the base of the lift (with the arm)
                leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
                rightRearMotor.setDirection(DcMotor.Direction.REVERSE);
                leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
                rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
                TrySetDriveMotorsPower(leftFrontPower, rightFrontPower, leftRearPower, rightRearPower);
                break;
            case ELEVATOR_BASE:
                // the side that the base of the elevator is
                leftRearMotor.setDirection(DcMotor.Direction.FORWARD);
                rightRearMotor.setDirection(DcMotor.Direction.REVERSE);
                leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
                rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
                TrySetDriveMotorsPower(leftRearPower, leftFrontPower, rightRearPower, rightFrontPower);
                break;
        }
    }

    public void RecalibrateGryo()
    {
        // initialize the gyro
        if (Gyro == null)
        {
            m_opModeConfig.LogError("Gyro is null in RecalibrateGryo()");
        }
        else
        {
            // just make a call that uses the gyro, and it will self initialize
            Gyro.initialize(GyroParameters);
            while (!Gyro.isGyroCalibrated())
            {
                m_opModeConfig.OpMode.sleep(50);
                m_opModeConfig.OpMode.idle();
            }

            ComposeGyroTelemetry();
        }
    }

    public void ComposeGyroTelemetry()
    {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        m_opModeConfig.Telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the GyroAngles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            GyroAngles = Gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            GyroGravity = Gyro.getGravity();
        }
        });

        if (!m_opModeConfig.CompetitionMode)
        {
            m_opModeConfig.Telemetry.addLine()
                    .addData("status", new Func<String>() {
                        @Override
                        public String value() {
                            return Gyro.getSystemStatus().toShortString();
                        }
                    })
                    .addData("calib", new Func<String>() {
                        @Override
                        public String value() {
                            return Gyro.getCalibrationStatus().toString();
                        }
                    });

            m_opModeConfig.Telemetry.addLine()
                    .addData("heading", new Func<String>() {
                        @Override
                        public String value() {
                            return FormatGyroAngle(GyroAngles.angleUnit, GyroAngles.firstAngle);
                        }
                    })
                    .addData("roll", new Func<String>() {
                        @Override
                        public String value() {
                            return FormatGyroAngle(GyroAngles.angleUnit, GyroAngles.secondAngle);
                        }
                    })
                    .addData("pitch", new Func<String>() {
                        @Override
                        public String value() {
                            return FormatGyroAngle(GyroAngles.angleUnit, GyroAngles.thirdAngle);
                        }
                    });
        }
    }

    public String FormatGyroAngle(AngleUnit angleUnit, double angle)
    {
        return FormatGyroDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    public String FormatGyroDegrees(double degrees)
    {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public boolean IsLiftAtLowerLimit()
    {
        if (lowerLimitTouchSensor != null)
        {
            if (lowerLimitTouchSensor.getState() == true)
            {
                return false;
            }
            else
            {
                return true;
            }
        }
        else
        {
            return true;
        }
    }
    public void TrySetDriveMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior powerBehavior)
    {
        // turn on motor break mode or float mode
        m_opModeConfig.LogInfo("Set:" + powerBehavior.toString());
        try
        {
            rightFrontMotor.setZeroPowerBehavior(powerBehavior);
            rightRearMotor.setZeroPowerBehavior(powerBehavior);
            leftFrontMotor.setZeroPowerBehavior(powerBehavior);
            leftRearMotor.setZeroPowerBehavior(powerBehavior);
        }
        catch (Exception ex)
        {
            // something went wrong
            m_opModeConfig.LogError("Setting ZeroDriveMotorsPowerBehavior Exception: " + ex.getMessage() );
        }
    }

    public void TrySetDriveMotorsRunMode(DcMotor.RunMode runMode)
    {
        m_opModeConfig.LogInfo("Set:" + runMode.toString());
        try
        {
            rightFrontMotor.setMode(runMode);
            rightRearMotor.setMode(runMode);
            leftFrontMotor.setMode(runMode);
            leftRearMotor.setMode(runMode);
        }
        catch (Exception ex)
        {
            // something went wrong
            m_opModeConfig.LogError("Setting SetDriveMotorsRunMode Exception: " + ex.getMessage() );
        }
    }

    public void TrySetDriveMotorsPower(double power)
    {
        power = Range.clip(power, -1.0, 1.0);
        m_opModeConfig.LogInfo("Setting all motor powers to: " + power);
        try
        {
            rightFrontMotor.setPower(power);
            rightRearMotor.setPower(power);
            leftFrontMotor.setPower(power);
            leftRearMotor.setPower(power);
        }
        catch (Exception ex)
        {
            // something went wrong
            m_opModeConfig.LogError("Setting SetDriveMotorsPower Exception: " + ex.getMessage() );
        }
    }

    public void TrySetDriveMotorsPower(double rightFront, double rightRear, double leftFront, double leftRear)
    {
        m_opModeConfig.LogInfo("Setting motor powers, rightFront: " + rightFront + ", rightRear: " + rightRear
                + ", leftFront: " + leftFront + ", leftRear: " + leftRear);
        rightFront = Range.clip(rightFront, -1.0, 1.0);
        rightRear = Range.clip(rightRear, -1.0, 1.0);
        leftFront = Range.clip(leftFront, -1.0, 1.0);
        leftRear = Range.clip(leftRear, -1.0, 1.0);
        try
        {
            rightFrontMotor.setPower(rightFront);
            rightRearMotor.setPower(rightRear);
            leftFrontMotor.setPower(leftFront);
            leftRearMotor.setPower(leftRear);
        }
        catch (Exception ex)
        {
            // something went wrong
            m_opModeConfig.LogError("Setting SetDriveMotorsPower Exception: " + ex.getMessage() );
        }
    }

    public void TrySetDriveMotorsToPosition(int position)
    {
        m_opModeConfig.LogInfo("Setting all motor target positions to: " + position);
        try
        {
            rightFrontMotor.setTargetPosition(position);
            rightRearMotor.setTargetPosition(position);
            leftFrontMotor.setTargetPosition(position);
            leftRearMotor.setTargetPosition(position);
        }
        catch (Exception ex)
        {
            // something went wrong
            m_opModeConfig.LogError("Setting TrySetDriveMotorsToPosition Exception: " + ex.getMessage() );
        }
    }

    public void TrySetDriveMotorsToPosition(int rightFront, int rightRear, int leftFront, int leftRear)
    {
        m_opModeConfig.LogInfo("Setting motor positions, rightFront: " + rightFront + ", rightRear: " + rightRear
                + ", leftFront: " + leftFront + ", leftRear: " + leftRear);
        try
        {
            rightFrontMotor.setTargetPosition(rightFront);
            rightRearMotor.setTargetPosition(rightRear);
            leftFrontMotor.setTargetPosition(leftFront);
            leftRearMotor.setTargetPosition(leftRear);
        }
        catch (Exception ex)
        {
            // something went wrong
            m_opModeConfig.LogError("Setting TrySetDriveMotorsToPosition Exception: " + ex.getMessage() );
        }
    }

    
    public void StopRobot()
    {
        m_opModeConfig.LogInfo("Entry");
        TrySetDriveMotorsPower(0);
    }

    public boolean IsBusy(DcMotor motor)
    {
        return IsBusy(motor, 40);
    }

    public boolean IsBusy(DcMotor motor, int allowedVariance)
    {
        //int allowedVariance = 45;
        if (motor != null)
        {
            int currentPosition = motor.getCurrentPosition();
            int targetPosition = motor.getTargetPosition();

            if (((currentPosition - allowedVariance) <= targetPosition) && (targetPosition <= (currentPosition + allowedVariance)))
            {
                return false;
            }else
                {
                return motor.isBusy();
            }
        }
        else
        {
            return false;
        }
    }

    public void AutonomousDrive(MEC_DRIVE_AUTO_DIRECTION direction, double inchesToTravel, double power, double timeout)
    {
        power = Range.clip(Math.abs(power), -1.0, 1.0);
        m_opModeConfig.LogInfo("Autonomously driving, direction: " + direction.toString() + ", inchedToTravel: " + inchesToTravel +
                ", power: " + power + ", timeout: " + timeout + "...");
        TrySetDriveMotorsRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TrySetDriveMotorsRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        // assume that the front of the robot is where the phone is when in autonomous
        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotor.Direction.FORWARD);
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);

        int targetCardinalEncoderCount = (int) (inchesToTravel * MEC_DRIVE_CARDINAL_COUNTS_PER_INCH);
        int targetDiagonalEncoderCount = (int) (inchesToTravel * MEC_DRIVE_CARDINAL_COUNTS_PER_INCH);
        m_opModeConfig.LogInfo("targetCardinalEncoderCount: " + targetCardinalEncoderCount +
                ", targetDiagonalEncoderCount: " + targetDiagonalEncoderCount);
        switch (direction) {
            case FORWARD:
                TrySetDriveMotorsToPosition(targetCardinalEncoderCount);
                TrySetDriveMotorsPower(power);
                m_elapsedTime.reset();
                while (m_opModeConfig.OpMode.opModeIsActive() && (m_elapsedTime.seconds() < timeout) &&
                        (IsBusy(leftFrontMotor) || IsBusy(leftRearMotor) ||
                                IsBusy(rightFrontMotor) || IsBusy(rightRearMotor)))
                //    (leftFrontMotor.isBusy() || leftRearMotor.isBusy() ||
                //            rightFrontMotor.isBusy() || rightRearMotor.isBusy()))
                {
                    // wait for the loop to complete
                }
                if (m_elapsedTime.seconds() >= timeout)
                {
                    m_opModeConfig.LogError("Timeout exceeded: " + timeout +
                            ", leftFrontMotor: " + targetCardinalEncoderCount + "/" + leftFrontMotor.getCurrentPosition() +
                            ", leftRearMotor: " + targetCardinalEncoderCount + "/" + leftRearMotor.getCurrentPosition() +
                            ", rightFrontMotor: " + targetCardinalEncoderCount + "/" + rightFrontMotor.getCurrentPosition() +
                            ", rightRearMotor: " + targetCardinalEncoderCount + "/" + rightRearMotor.getCurrentPosition());
                }
                break;
            case REVERSE:
                TrySetDriveMotorsToPosition(-targetCardinalEncoderCount);
                TrySetDriveMotorsPower(-power);
                m_elapsedTime.reset();
                while (m_opModeConfig.OpMode.opModeIsActive() && (m_elapsedTime.seconds() < timeout) &&
                        (IsBusy(leftFrontMotor) || IsBusy(leftRearMotor) ||
                                IsBusy(rightFrontMotor) || IsBusy(rightRearMotor)))
                {
                    // wait for the loop to complete
                }
                if (m_elapsedTime.seconds() >= timeout)
                {
                    m_opModeConfig.LogError("Timeout exceeded: " + timeout +
                            ", leftFrontMotor: " + -targetCardinalEncoderCount + "/" + leftFrontMotor.getCurrentPosition() +
                            ", leftRearMotor: " + -targetCardinalEncoderCount + "/" + leftRearMotor.getCurrentPosition() +
                            ", rightFrontMotor: " + -targetCardinalEncoderCount + "/" + rightFrontMotor.getCurrentPosition() +
                            ", rightRearMotor: " + -targetCardinalEncoderCount + "/" + rightRearMotor.getCurrentPosition());
                }
                break;
            case LEFT:
                TrySetDriveMotorsToPosition(targetCardinalEncoderCount, -targetCardinalEncoderCount, -targetCardinalEncoderCount, targetCardinalEncoderCount);
                TrySetDriveMotorsPower(power, -power, -power, power);
                m_elapsedTime.reset();
                while (m_opModeConfig.OpMode.opModeIsActive() && (m_elapsedTime.seconds() < timeout) &&
                        (IsBusy(leftFrontMotor) || IsBusy(leftRearMotor) ||
                                IsBusy(rightFrontMotor) || IsBusy(rightRearMotor)))
                {
                    // log something if you want
                }
                if (m_elapsedTime.seconds() >= timeout)
                {
                    m_opModeConfig.LogError("Timeout exceeded: " + timeout +
                            ", leftFrontMotor: " + -targetCardinalEncoderCount + "/" + leftFrontMotor.getCurrentPosition() +
                            ", leftRearMotor: " + targetCardinalEncoderCount + "/" + leftRearMotor.getCurrentPosition() +
                            ", rightFrontMotor: " + targetCardinalEncoderCount + "/" + rightFrontMotor.getCurrentPosition() +
                            ", rightRearMotor: " + -targetCardinalEncoderCount + "/" + rightRearMotor.getCurrentPosition());
                }
                break;
            case RIGHT:
                TrySetDriveMotorsToPosition(-targetCardinalEncoderCount, targetCardinalEncoderCount, targetCardinalEncoderCount, -targetCardinalEncoderCount);
                TrySetDriveMotorsPower(-power, power, power, -power);
                m_elapsedTime.reset();
                while (m_opModeConfig.OpMode.opModeIsActive() && (m_elapsedTime.seconds() < timeout) &&
                        (IsBusy(leftFrontMotor) || IsBusy(leftRearMotor) ||
                                IsBusy(rightFrontMotor) || IsBusy(rightRearMotor)))
                {
                    // log something if you want
                }
                if (m_elapsedTime.seconds() >= timeout)
                {
                    m_opModeConfig.LogError("Timeout exceeded: " + timeout +
                            ", leftFrontMotor: " + targetCardinalEncoderCount + "/" + leftFrontMotor.getCurrentPosition() +
                            ", leftRearMotor: " + -targetCardinalEncoderCount + "/" + leftRearMotor.getCurrentPosition() +
                            ", rightFrontMotor: " + -targetCardinalEncoderCount + "/" + rightFrontMotor.getCurrentPosition() +
                            ", rightRearMotor: " + targetCardinalEncoderCount + "/" + rightRearMotor.getCurrentPosition());
                }
                break;
            case DIAG_REVERSE_LEFT:
                TrySetDriveMotorsToPosition(0,-targetDiagonalEncoderCount , -targetDiagonalEncoderCount, 0);
                TrySetDriveMotorsPower(0, -power, -power, 0);
                m_elapsedTime.reset();
                while (m_opModeConfig.OpMode.opModeIsActive() && (m_elapsedTime.seconds() < timeout) &&
                        (leftFrontMotor.isBusy() || rightRearMotor.isBusy()))
                {
                    // log something if you want
                }
                if (m_elapsedTime.seconds() >= timeout)
                {
                    m_opModeConfig.LogError("Timeout exceeded: " + timeout +
                            ", leftFrontMotor: " + -targetCardinalEncoderCount + "/" + leftFrontMotor.getCurrentPosition() +
                            ", rightRearMotor: " + -targetCardinalEncoderCount + "/" + rightRearMotor.getCurrentPosition());
                }
                break;
            case DIAG_REVERSE_RIGHT:
                TrySetDriveMotorsToPosition(-targetDiagonalEncoderCount, 0,  0, -targetDiagonalEncoderCount);
                TrySetDriveMotorsPower(-power, 0, 0, -power);
                m_elapsedTime.reset();
                while (m_opModeConfig.OpMode.opModeIsActive() && (m_elapsedTime.seconds() < timeout) &&
                        (leftRearMotor.isBusy() || rightFrontMotor.isBusy() ))
                {
                    // log something if you want
                }
                if (m_elapsedTime.seconds() >= timeout)
                {
                    m_opModeConfig.LogError("Timeout exceeded: " + timeout +
                            ", leftRearMotor: " + -targetCardinalEncoderCount + "/" + leftRearMotor.getCurrentPosition() +
                            ", rightFrontMotor: " + -targetCardinalEncoderCount + "/" + rightFrontMotor.getCurrentPosition());
                }
                break;
            case DIAG_FORWARD_LEFT:
                TrySetDriveMotorsToPosition(targetDiagonalEncoderCount,0 , 0, targetDiagonalEncoderCount);
                TrySetDriveMotorsPower(power, 0, 0, power);
                while (m_opModeConfig.OpMode.opModeIsActive() && (m_elapsedTime.seconds() < timeout) &&
                        (leftRearMotor.isBusy() || rightFrontMotor.isBusy() ))
                {
                    // log something if you want
                }
                if (m_elapsedTime.seconds() >= timeout)
                {
                    m_opModeConfig.LogError("Timeout exceeded: " + timeout +
                            ", leftRearMotor: " + targetCardinalEncoderCount + "/" + leftRearMotor.getCurrentPosition() +
                            ", rightFrontMotor: " + targetCardinalEncoderCount + "/" + rightFrontMotor.getCurrentPosition());
                }
                break;
            case DIAG_FORWARD_RIGHT:
                TrySetDriveMotorsToPosition(0, targetDiagonalEncoderCount, targetDiagonalEncoderCount, 0);
                TrySetDriveMotorsPower(0, power, power, 0);
                m_elapsedTime.reset();
                while (m_opModeConfig.OpMode.opModeIsActive() && (m_elapsedTime.seconds() < timeout) &&
                        (leftFrontMotor.isBusy() ||  rightRearMotor.isBusy()))
                {
                    // log something if you want
                }
                if (m_elapsedTime.seconds() >= timeout)
                {
                    m_opModeConfig.LogError("Timeout exceeded: " + timeout +
                            ", leftFrontMotor: " + targetCardinalEncoderCount + "/" + leftFrontMotor.getCurrentPosition() +
                            ", rightRearMotor: " + targetCardinalEncoderCount + "/" + rightRearMotor.getCurrentPosition());
                }
                break;
        }
        // stop the robot when done
        StopRobot();
        TrySetDriveMotorsRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_opModeConfig.LogInfo("Completed autonomous step, direction: " + direction.toString());
    }

    public void DriveForward(double inchesToTravel, double power, double timeout)
    {
        AutonomousDrive(MEC_DRIVE_AUTO_DIRECTION.FORWARD, inchesToTravel, power, timeout);
    }

    public void DriveBackward(double inchesToTravel, double power, double timeout)
    {
        AutonomousDrive(MEC_DRIVE_AUTO_DIRECTION.REVERSE, inchesToTravel, power, timeout);
    }

    public void StrafeLeft(double inchesToTravel, double power, double timeout)
    {
        AutonomousDrive(MEC_DRIVE_AUTO_DIRECTION.LEFT, inchesToTravel, power, timeout);
    }

    public void StrafeRight(double inchesToTravel, double power, double timeout)
    {

        AutonomousDrive(MEC_DRIVE_AUTO_DIRECTION.RIGHT, inchesToTravel, power, timeout);
    }

    public void StrafeDiagonalForwardRight(double inchesToTravel, double power, double timeout)
    {
        AutonomousDrive(MEC_DRIVE_AUTO_DIRECTION.DIAG_FORWARD_RIGHT, inchesToTravel, power, timeout);
    }

    public void StrafeDiagonalForwardLeft(double inchesToTravel, double power, double timeout)
    {
        AutonomousDrive(MEC_DRIVE_AUTO_DIRECTION.DIAG_FORWARD_LEFT, inchesToTravel, power, timeout);
    }

    public void StrafeDiagonalReverseLeft(double inchesToTravel, double power, double timeout)
    {
        AutonomousDrive(MEC_DRIVE_AUTO_DIRECTION.DIAG_REVERSE_LEFT, inchesToTravel, power, timeout);
    }

    public void StrafeDiagonalReverseRight(double inchesToTravel, double power, double timeout)
    {
        AutonomousDrive(MEC_DRIVE_AUTO_DIRECTION.DIAG_REVERSE_RIGHT, inchesToTravel, power, timeout);
    }

    public boolean IsLiftAtUpperLimit()
    {
        if (upperLimitTouchSensor != null)
        {
            if (upperLimitTouchSensor.getState() == true)
            {
                return false;
            }
            else
            {
                return true;
            }
        }
        else
        {
            return true;
        }
    }

    /* getError determines the error between the target angle and the robot's current heading
        @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
        @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
        +ve error means the robot should turn LEFT (CCW) to reduce error.
    */
    public double getError(double targetAngle)
    {
        double robotError;

        // calculate error in -179 to +180 range
        if (Gyro == null)
        {
            m_opModeConfig.Telemetry.addData(">", "Gyro is null in getError()");
        }
        robotError = targetAngle - Gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /* returns desired steering force. +/- 1 range. +ve = steer left
        @param error Error angle in robot relative degrees *
        @param PCoeff Proportional Gain Coefficient * @return
    */
    public double getSteer(double error, double PCoeff)
    {
        return Range.clip(error * PCoeff, -1, 1);
    }

    /*  Method to spin on central axis to point in a new direction.
        Move will stop if either of these conditions occur:
        1) Move gets to the heading (angle)
        2) Driver stops the opmode running.
            @param speed Desired speed of turn.
            @param angle Absolute Angle (in Degrees) relative to last gyro reset.
            0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
            If a relative angle is required, add/subtract from current heading.
    */
    public void gyroTurn (double speed, double angle, double holdTime)
    {
        // keep looping while we are still active, and not on heading.
        TrySetDriveMotorsRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();
        while (m_opModeConfig.OpMode.opModeIsActive()
                && (elapsedTime.time() < holdTime)
                && !onHeading(speed, angle, P_TURN_COEFF))
        {
            // Update telemetry & Allow time for other processes to run.
            m_opModeConfig.Telemetry.update();
            m_opModeConfig.OpMode.idle();
        }
    }

    /* Method to obtain & hold a heading for a finite amount of time
        Move will stop once the requested time has elapsed
        @param speed Desired speed of turn.
        @param angle Absolute Angle (in Degrees) relative to last gyro reset.
            0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
            If a relative angle is required, add/subtract from current heading.
        @param holdTime Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime)
    {
        // keep looping while we have time remaining.
        TrySetDriveMotorsRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();
        while (m_opModeConfig.OpMode.opModeIsActive()
                && (elapsedTime.time() < holdTime))
        {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            m_opModeConfig.Telemetry.update();
            m_opModeConfig.OpMode.idle();
        }

        // Stop all motion;
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightRearMotor.setPower(0);
    }

    /*  Perform one cycle of closed loop heading control.
        @param speed Desired speed of turn.
        @param angle Absolute Angle (in Degrees) relative to last gyro reset.
        0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
        If a relative angle is required, add/subtract from current heading.
        @param PCoeff Proportional Gain coefficient
        @return
    */
    public boolean onHeading2017(double speed, double angle, double PCoeff)
    {
        double error ;
        double steer ;
        boolean onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);
        if (Math.abs(error) <= HEADING_THRESHOLD)
        {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else
        {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        // Send desired speeds to motors.
        /*
        leftFrontMotor.setPower(leftSpeed);
        rightFrontMotor.setPower(rightSpeed);
        leftRearMotor.setPower(leftSpeed);
        rightRearMotor.setPower(rightSpeed);
        */

        if (m_opModeConfig.DriveTrainSet == OpModeConfig.ROBOT_DRIVE_TRAIN_SET.A_LF_RR)
        {
            leftFrontMotor.setPower(leftSpeed);
            rightRearMotor.setPower(rightSpeed);
        }
        else if (m_opModeConfig.DriveTrainSet == OpModeConfig.ROBOT_DRIVE_TRAIN_SET.B_RF_LR)
        {
            rightFrontMotor.setPower(rightSpeed);
            leftRearMotor.setPower(leftSpeed);
        }
        else if (m_opModeConfig.DriveTrainSet == OpModeConfig.ROBOT_DRIVE_TRAIN_SET.C_ALL_FORWARD_BACKWARD)
        {
            leftFrontMotor.setPower(leftSpeed);
            rightFrontMotor.setPower(rightSpeed);
            leftRearMotor.setPower(leftSpeed);
            rightRearMotor.setPower(rightSpeed);
        }
        else //if (m_opModeConfig.DriveTrainSet == OpModeConfig.ROBOT_DRIVE_TRAIN_SET.D_ALL_SIDE_2_SIDE)
        {
            leftFrontMotor.setPower(leftSpeed);
            rightFrontMotor.setPower(rightSpeed);
            leftRearMotor.setPower(leftSpeed);
            rightRearMotor.setPower(rightSpeed);
        }

        // Display it for the driver.
        if (!m_opModeConfig.CompetitionMode)
        {
            m_opModeConfig.Telemetry.addData("Target", "%5.2f", angle);
            m_opModeConfig.Telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
            m_opModeConfig.Telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);
        }
        return onTarget;
    }

    public boolean onHeading(double speed, double angle, double PCoeff)
    {
        double error ;
        double steer ;
        boolean onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);
        if (Math.abs(error) <= HEADING_THRESHOLD)
        {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else
        {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        //                   rightFront   rightRear      leftFront      LeftRear
        TrySetDriveMotorsPower(rightSpeed, rightSpeed, leftSpeed, leftSpeed);

        // Display it for the driver.
//        if (!m_opModeConfig.CompetitionMode)
//        {
//            //TBD - use other logging
//            m_opModeConfig.Telemetry.addData("Target", "%5.2f", angle);
//            m_opModeConfig.Telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
//            m_opModeConfig.Telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);
//            m_opModeConfig.Telemetry.update();
//
//        }
        return onTarget;
    }

    public void ExtendArmIntoCrater()
    {
        mineralMotor.setPower(1);
        RobotUtil.TryControlMotorByEncoder(this.liftMotor, m_opModeConfig, .55 , -4000, 2);
        RobotUtil.TryControlMotorByTime(this.extenderMotor, m_opModeConfig, -.55, 1);
    }

    public void TurnByGyro(double degree)
    {
        if (m_useGryo && m_gyroInitialized)
        {
            // use the difference between the current angle and the angle passed in
            double error = getError(degree);

            if (Math.abs(error) <= 10)
            {
                this.gyroTurn(.4, degree, .25);
            }
            else if ((Math.abs(error) > 10) && (Math.abs(degree) <= 55))
            {
                this.gyroTurn(.42, degree, 1.05);
                this.gyroHold(.32, degree, .40);
            }
            else if ((Math.abs(error) > 55) && (Math.abs(degree) <= 100))
            {
                this.gyroTurn(.42, degree, 1.25);
                this.gyroHold(.32, degree, .50);
            }
            else if ((Math.abs(error) > 100) && (Math.abs(degree) <=145))
            {
                this.gyroTurn(.44, degree, 1.35);
                this.gyroHold(.34, degree, .55);
            }
            else if ((Math.abs(error) > 145) && (Math.abs(degree) <=190))
            {
                this.gyroTurn(.46, degree, 1.45);
                this.gyroHold(.36, degree, .60);
            }
            else
            {
                this.gyroTurn(.48, degree, 1.45);
                this.gyroHold(.38, degree, .65);
            }
        }
    }

    public void TurnByEncoder(double degree, double power, double timeout)
    {
        m_opModeConfig.LogInfo("Autonomously turning, direction: " + degree +
                ", power: " + power + ", timeout: " + timeout + "...");
        power = Range.clip(Math.abs(power), -1.0, 1.0);
        TrySetDriveMotorsRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TrySetDriveMotorsRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        int targetCounts = Math.abs((int) (degree * 9.2));
        if (degree < 0) {
            // turn the front to the left
            //                          rightFront   rightRear      leftFront      LeftRear
            TrySetDriveMotorsToPosition(targetCounts, targetCounts, -targetCounts, -targetCounts);
            TrySetDriveMotorsPower(power, power, -power, -power);
        }
        else {
            // turn the front to the right
            TrySetDriveMotorsToPosition(-targetCounts, -targetCounts, targetCounts, targetCounts);
            TrySetDriveMotorsPower(-power, -power, power, power);
        }

        m_elapsedTime.reset();
        while (m_opModeConfig.OpMode.opModeIsActive() && (m_elapsedTime.seconds() < timeout) &&
            //    (leftFrontMotor.isBusy() || leftRearMotor.isBusy() ||
            //            rightFrontMotor.isBusy() || rightRearMotor.isBusy()))
            (IsBusy(leftFrontMotor, 30) || IsBusy(leftRearMotor, 30) ||
                    IsBusy(rightFrontMotor, 30) || IsBusy(rightRearMotor, 30)))
        {
            // log something if you want
        }

        // when rotating the front to the left:
        // front left: -
        // front right: +

        // rear left: -
        // rear right: +

        // when rotating the front to the right:
        // front left: +
        // front right: -

        // rear left: +
        // rear right: -

        // stop the robot when done
        StopRobot();
        m_opModeConfig.LogInfo("Completed autonomous step, direction: " + degree);
    }

    public void DriveByEncoder(double speed,
                               double leftInches, double rightInches,
                               double timeoutSeconds)
    {
        int newLeftTarget;
        int newRightTarget;
        DcMotor rightMotor;
        DcMotor leftMotor;

        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Ensure that the opmode is still active
        if (m_opModeConfig.OpMode.opModeIsActive())
        {
            // Determine new target position, and pass to motor controller
            if (m_opModeConfig.DriveTrainSet == OpModeConfig.ROBOT_DRIVE_TRAIN_SET.A_LF_RR)
            {
                newLeftTarget = leftFrontMotor.getCurrentPosition() + (int) (leftInches * MEC_DRIVE_CARDINAL_COUNTS_PER_INCH);
                newRightTarget = rightRearMotor.getCurrentPosition() + (int) (rightInches * MEC_DRIVE_CARDINAL_COUNTS_PER_INCH);
                leftFrontMotor.setTargetPosition(newLeftTarget);
                rightRearMotor.setTargetPosition(newRightTarget);
                rightMotor = rightRearMotor;
                leftMotor = leftFrontMotor;
            }
            else if (m_opModeConfig.DriveTrainSet == OpModeConfig.ROBOT_DRIVE_TRAIN_SET.B_RF_LR)
            {
                newRightTarget = rightFrontMotor.getCurrentPosition() + (int) (rightInches * MEC_DRIVE_CARDINAL_COUNTS_PER_INCH);
                newLeftTarget = leftRearMotor.getCurrentPosition() + (int) (leftInches * MEC_DRIVE_CARDINAL_COUNTS_PER_INCH);
                rightFrontMotor.setTargetPosition(newRightTarget);
                leftRearMotor.setTargetPosition(newLeftTarget);
                rightMotor = rightFrontMotor;
                leftMotor = leftRearMotor;
            }
            else if (m_opModeConfig.DriveTrainSet == OpModeConfig.ROBOT_DRIVE_TRAIN_SET.C_ALL_FORWARD_BACKWARD)
            {
                newLeftTarget = leftFrontMotor.getCurrentPosition() + (int) (leftInches * MEC_DRIVE_CARDINAL_COUNTS_PER_INCH);
                newRightTarget = rightFrontMotor.getCurrentPosition() + (int) (rightInches * MEC_DRIVE_CARDINAL_COUNTS_PER_INCH);
                leftFrontMotor.setTargetPosition(newLeftTarget);
                rightFrontMotor.setTargetPosition(newRightTarget);
                leftRearMotor.setTargetPosition(newLeftTarget);
                rightRearMotor.setTargetPosition(newRightTarget);
                rightMotor = rightFrontMotor;
                leftMotor = leftFrontMotor;
            }
            else //if (m_opModeConfig.DriveTrainSet == OpModeConfig.ROBOT_DRIVE_TRAIN_SET.D_ALL_SIDE_2_SIDE)
            {
                // ROBOT_DRIVE_TRAIN_SET.D_ALL_SIDE_2_SIDE (this is clear as mud, do not change)
                // remember that when in drive to encoders, sign of the power is ignored and sign iof counts is used!!
                newLeftTarget = rightFrontMotor.getCurrentPosition() + (int) (leftInches * MEC_DRIVE_CARDINAL_COUNTS_PER_INCH);
                newRightTarget = rightRearMotor.getCurrentPosition() + (int) (rightInches * MEC_DRIVE_CARDINAL_COUNTS_PER_INCH);
                leftFrontMotor.setTargetPosition(newLeftTarget);
                rightFrontMotor.setTargetPosition(-newLeftTarget);
                leftRearMotor.setTargetPosition(-newRightTarget);
                rightRearMotor.setTargetPosition(newRightTarget);
                rightMotor = rightRearMotor;
                leftMotor = rightFrontMotor;
            }

            // reset the timeout time and start motion.
            if (m_opModeConfig.DriveTrainSet == OpModeConfig.ROBOT_DRIVE_TRAIN_SET.A_LF_RR)
            {
                leftFrontMotor.setPower(Math.abs(speed));
                rightRearMotor.setPower(Math.abs(speed));
            }
            else if (m_opModeConfig.DriveTrainSet == OpModeConfig.ROBOT_DRIVE_TRAIN_SET.B_RF_LR)
            {
                rightFrontMotor.setPower(Math.abs(speed));
                leftRearMotor.setPower(Math.abs(speed));
            }
            else if (m_opModeConfig.DriveTrainSet == OpModeConfig.ROBOT_DRIVE_TRAIN_SET.C_ALL_FORWARD_BACKWARD)
            {
                leftFrontMotor.setPower(Math.abs(speed));   //R
                rightFrontMotor.setPower(Math.abs(speed)); //F
                leftRearMotor.setPower(Math.abs(speed));    //R
                rightRearMotor.setPower(Math.abs(speed));  //F
            }
            else //if (m_opModeConfig.DriveTrainSet == OpModeConfig.ROBOT_DRIVE_TRAIN_SET.D_ALL_SIDE_2_SIDE)
            {
                // OpModeConfig.ROBOT_DRIVE_TRAIN_SET.D_ALL_SIDE_2_SIDE
                leftFrontMotor.setPower(Math.abs(speed));    //R
                rightFrontMotor.setPower(Math.abs(speed));   //F
                leftRearMotor.setPower(Math.abs(speed));      //R
                rightRearMotor.setPower(Math.abs(speed));     //F
            }

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the m_robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the m_robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            ElapsedTime elapsedTime = new ElapsedTime();
            elapsedTime.reset();
            while (m_opModeConfig.OpMode.opModeIsActive() &&
                    (elapsedTime.seconds() < timeoutSeconds) &&
                    ((leftMotor.isBusy() && rightMotor.isBusy())))
            {
                if (!m_opModeConfig.CompetitionMode) {
                    //m_opModeConfig.Telemetry.addData(">", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                    //m_opModeConfig.Telemetry.addData(">", "Running at %7d :%7d",
                    //        leftMotor.getCurrentPosition(),
                    //        rightMotor.getCurrentPosition());

                    m_opModeConfig.Telemetry.addData(">", "RunningF at %7d :%7d",
                            leftFrontMotor.getCurrentPosition(),
                            rightFrontMotor.getCurrentPosition());
                    m_opModeConfig.Telemetry.addData(">", "RunningR at %7d :%7d",
                            leftRearMotor.getCurrentPosition(),
                            rightRearMotor.getCurrentPosition());
                    m_opModeConfig.Telemetry.update();
                }
            }

            // Stop all motion;
            leftFrontMotor.setPower(0);
            rightFrontMotor.setPower(0);
            leftRearMotor.setPower(0);
            rightRearMotor.setPower(0);

            // Turn off RUN_TO_POSITION, RUN_WITHOUT_ENCODER runs faster in TeleOp
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }


    public void RunLiftMotorByEncoder(double speed, int encoderCounts, double timeout)
    {
        int newTarget;
        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Turn On RUN_TO_POSITION
        elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Ensure that the opmode is still active
        if (m_opModeConfig.OpMode.opModeIsActive())
        {
            // Determine new target position, and pass to motor controller
            newTarget = elevatorMotor.getCurrentPosition() + encoderCounts;
            elevatorMotor.setTargetPosition(newTarget);

            // reset the timeout time and start motion.
            ElapsedTime elapsedTime = new ElapsedTime();
            elapsedTime.reset();
            elevatorMotor.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the m_robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the m_robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (m_opModeConfig.OpMode.opModeIsActive() && (elapsedTime.seconds() < timeout)
                    && (elevatorMotor.isBusy())
                    && (!IsLiftAtLowerLimit() || encoderCounts > 0 )
                    && (!IsLiftAtUpperLimit() || encoderCounts < 0 ))
            {
                // Display it for the driver.
                if (!m_opModeConfig.CompetitionMode)
                {
                    m_opModeConfig.Telemetry.addData("Path1", "Running to %7d", newTarget);
                    m_opModeConfig.Telemetry.addData("Path2", "Running at %7d", elevatorMotor.getCurrentPosition());
                }
                m_opModeConfig.OpMode.idle();
            }

            // Stop all motion;
            elevatorMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            elevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void LowerRobot(boolean moveAfterLower){

        // lower lift by a small bit so it unbinds the servo motor
        // ADDED: 12/5/2018 before states
        PlaySound("eaglehaslanded");
        RunLiftMotorByEncoder(1, -500, 1);

        // release brake
        if (brakeServo != null)
        {
            brakeServo.setPosition(BRAKE_SERVO_UNLOCK);
            m_opModeConfig.OpMode.sleep(500);
        }

        // lower robot (raise lift)
        RunLiftMotorByEncoder(1, 4800, 6);
        m_opModeConfig.OpMode.sleep(100);

        if (moveAfterLower)
        {
            // move left
            StrafeLeft(2, .55, 2);
            m_opModeConfig.OpMode.sleep(100);

            // lower lift by a small bit so it stays mostly up
            //RunLiftMotorByEncoder(1, -750, 1);

            // re-align and move forward to clear the lander's legs
            TurnByGyro(0);
            DriveForward(10, .6, 2);
        }
    }

    public void RaiseRobot(boolean playSound){

        // release brake
        if (playSound)
        {
            PlaySound("liftoff");
        }
        if (brakeServo != null)
        {
            brakeServo.setPosition(BRAKE_SERVO_UNLOCK);
            m_opModeConfig.OpMode.sleep(350);
        }

        // set brake mode
        if (elevatorMotor != null)
        {
            elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // raise robot (raise lift)
        RunLiftMotorByEncoder(1, -4800, 6);

        // lock brake
        if (brakeServo != null)
        {
            brakeServo.setPosition(BRAKE_SERVO_LOCK);
        }
    }

    public void DropMarker()
    {
        if (markerServo != null)
        {
            markerServo.setDirection(Servo.Direction.REVERSE);
            markerServo.setPosition(.25);
            m_opModeConfig.OpMode.sleep(500);
            markerServo.setPosition(0);
        }
    }

    public void PlaySound(String fileName)
    {
        try
        {
            int soundId = m_opModeConfig.HardwareMap.appContext.getResources().getIdentifier(
                    fileName, "raw", m_opModeConfig.HardwareMap.appContext.getPackageName());
            if (soundId != 0)
            {
                if (SoundPlayer.getInstance().preload(m_opModeConfig.HardwareMap.appContext, soundId))
                {
                    SoundPlayer.getInstance().startPlaying(m_opModeConfig.HardwareMap.appContext, soundId);
                }
            }
        }
        catch (Exception ex)
        {
            // do nothing
        }
    }
}
