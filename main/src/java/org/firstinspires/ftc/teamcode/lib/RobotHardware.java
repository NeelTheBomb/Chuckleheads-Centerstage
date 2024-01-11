package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BNO055IMU;


/**
 * Initializes hardware variables. Makes it so that one function call is 
 * required in our teleop and autonomous files to get hardware variables.
 * @author Nathan W
 */
public class RobotHardware {
    public static DcMotorEx fl, bl, fr, br, opv, oph;
    // ia, is, slide, arm = null;
    // public static Servo iwl, iwr, lg, rg = null;
    public static BNO055IMU imu = null;

    /**
     * Initialize the hardware variables
     * @param hardwareMap HardwareMap from the Opmode
     */
    public static void robotHardwareInit(HardwareMap hardwareMap) {
        // Declare our motors
        fl = hardwareMap.get(DcMotorEx.class, "frontLeft");     // front left
        bl = hardwareMap.get(DcMotorEx.class, "backLeft");      // back left
        fr = hardwareMap.get(DcMotorEx.class, "frontRight");    // front right
        br = hardwareMap.get(DcMotorEx.class, "backRight");     // back right
        oph = hardwareMap.get(DcMotorEx.class, "frontLeft");
        opv = hardwareMap.get(DcMotorEx.class, "frontRight");
         slide = hardwareMap.get(DcMotorEx.class, "slide");      // slide
         ia = hardwareMap.get(DcMotorEx.class, "intakeArm");     // intake arm
         is = hardwareMap.get(DcMotorEx.class, "intakeSpinner"); // instake spinner
         arm = hardwareMap.get(DcMotorEx.class, "arm");          // worm gear arm
         iwl = hardwareMap.get(Servo.class, "intakeWristLeft");  // intake wrist left
         iwr = hardwareMap.get(Servo.class, "intakeWristRight"); // intake wrist right
         lg = hardwareMap.get(Servo.class, "lGrip");             // left gripper
         rg = hardwareMap.get(Servo.class, "rGrip");             // right gripper

        // reset encoders
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         ia.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         is.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set modes
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // reverse stuff
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
         iwl.setDirection(Servo.Direction.REVERSE);
         rg.setDirection(Servo.Direction.REVERSE);

         set zero power behavior
         arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         ia.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(BNO055IMU.class, "gyro");
        imuInit();
    }

    /**
     * Initialize the IMU. Teleop requires we be able to do this on demand,
     * hence making it a seperate function. Will fail if imuInit() is called before hardwareInit()
     * @return Whether or not the IMU was successfully initialized.
     */
    public static boolean imuInit() {
        if (imu == null) {
            return false;
        }

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);

        return true;
    }


    /**
     * Opens the gripper
     */
    public static void gripperOpen() {
        lg.setPosition(0.2);
        rg.setPosition(0.2);
    }


    /**
     * Closes the gripper
     */
    public static void gripperClose() {
        lg.setPosition(0.0);
        rg.setPosition(0.0);
    }
}