package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;


// this class represents an instance of the robot and all of the hardware
// and sensors that make up the robot
public class KUDOS_RobotMecCore
{
    // public variables that represent the main drive motors
    public DcMotor leftRearMotor = null;
    public DcMotor rightRearMotor = null;
    public DcMotor leftFrontMotor = null;
    public DcMotor rightFrontMotor = null;

  // lifting motor
    public DcMotor liftMotor = null;

    // private constants that represent the names of the devices as
    // programmed in to the configuration on the robot controller
    private final String LEFT_REAR_MOTOR = "left_rear";
    private final String LEFT_FRONT_MOTOR = "left_front";
    private final String RIGHT_REAR_MOTOR = "right_rear";
    private final String RIGHT_FRONT_MOTOR = "right_front";
    private final String LIFT_MOTOR = "lift_motor";

    private final String IMU_GYRO = "imu";

    private final int MEC_DRIVE_CARDINAL_COUNTS_PER_INCH = 70;
    private final int MEC_DRIVE_DIAG_COUNTS_PER_INCH = 68;

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
        CENTER,
        LEFT,
        RIGHT
    }

    // private references that need to be kept locally
    private OpModeConfig m_opModeConfig;
    private ElapsedTime m_elapsedTime = new ElapsedTime();

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double HEADING_THRESHOLD = 1 ; // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.09;    // Larger is more responsive, but also less stable (default was .09)
    static final double P_DRIVE_COEFF = 0.15;   // Larger is more responsive, but also less stable (default was .15)

    BNO055IMU Gyro = null;
    Orientation GyroAngles = null;
    Acceleration GyroGravity = null;
    BNO055IMU.Parameters GyroParameters = null;

    public KUDOS_RobotMecCore(OpModeConfig opModeConfig)
    {
        // we must know which m_robot we are working with
        m_opModeConfig = opModeConfig;
    }

    public boolean init()
    {
        boolean result = true;
        m_opModeConfig.LogInfo("Entry");

        // do we know which m_robot we are?
        if (m_opModeConfig.Team == OpModeConfig.ROBOT_TEAM.UNKNOWN)
        {
            m_opModeConfig.LogError("Team is not set");
            return false;
        }

        // do we have a reference to the opMode?
        if (m_opModeConfig.OpMode == null)
        {
            m_opModeConfig.LogError("Reference to OpMode is null.");
            return false;
        }

        // do we have a valid reference to the Hardware map?
        if (m_opModeConfig.OpMode.hardwareMap == null)
        {
            m_opModeConfig.LogError("Reference to OpMode.HardwareMap is null.");
            //return false;
        }

        // initialize the drive motors
        m_opModeConfig.LogInfo("Init drive motors...");
        try
        {
            leftRearMotor = m_opModeConfig.OpMode.hardwareMap.get(DcMotor.class, LEFT_REAR_MOTOR);
            rightRearMotor = m_opModeConfig.OpMode.hardwareMap.get(DcMotor.class, RIGHT_REAR_MOTOR);
            leftFrontMotor = m_opModeConfig.OpMode.hardwareMap.get(DcMotor.class, LEFT_FRONT_MOTOR);
            rightFrontMotor = m_opModeConfig.OpMode.hardwareMap.get(DcMotor.class, RIGHT_FRONT_MOTOR);

            // set all the drive motors to zero power
            TrySetDriveMotorsRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            TrySetDriveMotorsPower(0);

            // set default direction of motors
            leftRearMotor.setDirection(DcMotor.Direction.FORWARD);
            rightRearMotor.setDirection(DcMotor.Direction.REVERSE);
            leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
            rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);

            // set all motors to run without encoders for now - this is faster
            TrySetDriveMotorsRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            TrySetDriveMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        catch (Exception ex)
        {
            // something went wrong
            m_opModeConfig.LogError("Initializing Drive Motors Exception: " + ex.getMessage());
            result = false;
        }

        // initialize the lift motors
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
            m_opModeConfig.LogError("Initializing lift Motor Exception: " + ex.getMessage());
           //result = false;
        }

        // initialize the imu gyro
        m_opModeConfig.LogInfo("Init gyro...");
        try
        {
            Gyro = m_opModeConfig.OpMode.hardwareMap.get(BNO055IMU.class, IMU_GYRO);
        }
        catch (Exception ex)
        {
            // something went wrong
            m_opModeConfig.LogError("Initializing IMU Gyro Exception: " + ex.getMessage());
            result = false;
        }

        // the rest of the sensors and motors and servos...


        m_opModeConfig.LogInfo("Init complete...");
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

    public void TeleOpStrafeDrive(double left_stick_y_drive, double right_stick_x_turn, double left_stick_x_strafe, double appliedPower)
    {
        // make sure the applied power is within the expected range & has to be positive (.01 - 1.00)
        appliedPower = Math.abs(Range.clip(appliedPower, -1.0, 1.0));

        // applied the power to the raw inputs before any additional scaling is done
        double drive = -left_stick_y_drive * appliedPower;
        double turn = right_stick_x_turn * appliedPower;
        double strafe = left_stick_x_strafe * appliedPower;

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
        TrySetDriveMotorsPower(rightFrontPower, rightRearPower, leftFrontPower, leftRearPower);

        // log these values using SetWatchValue rather than individual log items since these will repeat over and over
        m_opModeConfig.SetWatchValue("Motors Power","left front: (%.3f), right front: (%.3f), left rear: (%.3f), right rear: (%.3f)",
                leftFrontPower, rightFrontPower, leftRearPower, rightRearPower);
        m_opModeConfig.SetWatchValue("LeftFront Counts","%7d",leftFrontMotor.getCurrentPosition());
        m_opModeConfig.SetWatchValue("RightFront Counts","%7d",rightFrontMotor.getCurrentPosition());
        m_opModeConfig.SetWatchValue("LeftRear Counts","%7d",leftRearMotor.getCurrentPosition());
        m_opModeConfig.SetWatchValue("RightRear Counts","%7d",rightRearMotor.getCurrentPosition());
    }

    public MINERAL_VISION_DIRECTIONS StrafeToMineral()
    {
        // determine which direction to go
        m_opModeConfig.OpMode.sleep(1000);
        return MINERAL_VISION_DIRECTIONS.LEFT;
    }

    public void LowerRobot()
    {
        // lower the bot
        m_opModeConfig.OpMode.sleep(1500);

        // open the latch

        // retract the arm
        m_opModeConfig.OpMode.sleep(1500);
    }

    public void TurnByEncoder(double degree, double power, double timeout)
    {
        m_opModeConfig.LogInfo("Autonomously turning, direction: " + degree +
                ", power: " + power + ", timeout: " + timeout + "...");
        power = Range.clip(Math.abs(power), -1.0, 1.0);
        TrySetDriveMotorsRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TrySetDriveMotorsRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        int targetCounts = Math.abs((int) (degree * 17.6));
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
                (leftFrontMotor.isBusy() || leftRearMotor.isBusy() ||
                rightFrontMotor.isBusy() || rightRearMotor.isBusy()))
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

    public void AutonomousDrive(MEC_DRIVE_AUTO_DIRECTION direction, double inchesToTravel, double power, double timeout)
    {
        power = Range.clip(Math.abs(power), -1.0, 1.0);
        m_opModeConfig.LogInfo("Autonomously driving, direction: " + direction.toString() + ", inchedToTravel: " + inchesToTravel +
            ", power: " + power + ", timeout: " + timeout + "...");
        TrySetDriveMotorsRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TrySetDriveMotorsRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        int targetCardinalEncoderCount = (int) (inchesToTravel * MEC_DRIVE_CARDINAL_COUNTS_PER_INCH);
        int targetDiagonalEncoderCount = (int) (inchesToTravel * MEC_DRIVE_DIAG_COUNTS_PER_INCH);
        m_opModeConfig.LogInfo("targetCardinalEncoderCount: " + targetCardinalEncoderCount +
                ", targetDiagonalEncoderCount: " + targetDiagonalEncoderCount);
        switch (direction) {
            case FORWARD:
                TrySetDriveMotorsToPosition(targetCardinalEncoderCount);
                TrySetDriveMotorsPower(power);
                m_elapsedTime.reset();
                while (m_opModeConfig.OpMode.opModeIsActive() && (m_elapsedTime.seconds() < timeout) &&
                        (leftFrontMotor.isBusy() || leftRearMotor.isBusy() ||
                        rightFrontMotor.isBusy() || rightRearMotor.isBusy()))
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
                        (leftFrontMotor.isBusy() || leftRearMotor.isBusy() ||
                        rightFrontMotor.isBusy() || rightRearMotor.isBusy()))
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
                        (leftFrontMotor.isBusy() || leftRearMotor.isBusy() ||
                        rightFrontMotor.isBusy() || rightRearMotor.isBusy()))
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
                        (leftFrontMotor.isBusy() || leftRearMotor.isBusy() ||
                        rightFrontMotor.isBusy() || rightRearMotor.isBusy()))
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

    public void StafeDiagonalReverseLeft(double inchesToTravel, double power, double timeout)
    {
        AutonomousDrive(MEC_DRIVE_AUTO_DIRECTION.DIAG_REVERSE_LEFT, inchesToTravel, power, timeout);
    }

    public void StrafeDiagonalReverseRight(double inchesToTravel, double power, double timeout)
    {
        AutonomousDrive(MEC_DRIVE_AUTO_DIRECTION.DIAG_REVERSE_RIGHT, inchesToTravel, power, timeout);
    }

    public void TrySetDriveMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior powerBehavior)
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
            m_opModeConfig.LogError( "Gyro is null in getError()");
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
        if (!m_opModeConfig.CompetitionMode)
        {
            //TBD - use other logging
            m_opModeConfig.Telemetry.addData("Target", "%5.2f", angle);
            m_opModeConfig.Telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
            m_opModeConfig.Telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);
            m_opModeConfig.Telemetry.update();

        }
        return onTarget;
    }
}
