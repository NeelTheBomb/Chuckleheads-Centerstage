package org.firstinspires.ftc.teamcode.centerstage;

// first
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

// our classes
import org.firstinspires.ftc.teamcode.lib.PreciseMovement;

// our static functions
import static org.firstinspires.ftc.teamcode.lib.MathStuff.remapRange;
import static org.firstinspires.ftc.teamcode.lib.MathStuff.shortestAngleRemapped;
import static org.firstinspires.ftc.teamcode.lib.MathStuff.sqr;


@TeleOp(name="Teleop", group="Linear Opmode")
public class Teleop extends LinearOpMode {
    // initialize hardware variables
    private DcMotorEx fl, bl, fr, br, ia, is, slide, arm = null;
    private Servo iwl, iwr, lg, rg = null;
    private BNO055IMU imu = null;
    
    private double moveGripperServo = 0.0; // fix: rename
    
    private final double rxMultiplier = 0.9;
    
    private final boolean gunnerActive = false;

    @Override
    public void runOpMode() {
        // Declare our motors
        fl = hardwareMap.get(DcMotorEx.class, "frontLeft");      // front left
        bl = hardwareMap.get(DcMotorEx.class, "backLeft");       // back left
        fr = hardwareMap.get(DcMotorEx.class, "frontRight");     // front right
        br = hardwareMap.get(DcMotorEx.class, "backRight");      // back right
        slide = hardwareMap.get(DcMotorEx.class, "slide");       // slide
        ia = hardwareMap.get(DcMotorEx.class, "intakeArm");      // intake arm
        is = hardwareMap.get(DcMotorEx.class, "intakeSpinner");  // instaks spinner
        arm = hardwareMap.get(DcMotorEx.class, "arm");           //
        iwl = hardwareMap.get(Servo.class, "intakeWristLeft");   // intake wrist left
        iwr = hardwareMap.get(Servo.class, "intakeWristRight");  // intake wrist right
        lg = hardwareMap.get(Servo.class, "lGrip");           // left gripper
        rg = hardwareMap.get(Servo.class, "rGrip");           // right gripper
        
        // reset encoders
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ia.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // reverse stuff
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        iwl.setDirection(Servo.Direction.REVERSE);
        rg.setDirection(Servo.Direction.REVERSE);
        
        // set zero power behavior
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ia.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(BNO055IMU.class, "gyro");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);
        
        // wait until init is pressed
        waitForStart();
        
        while (opModeIsActive()) {
            move();
            intake();
            intakeArm();
            intakeWrist();
            intakeLaunch();
            intakeSpinnerMove();
            slide();
            armMove();
            gripsForward();
            sucks(); // fix: rename
            
            telemetry.update();
        }
    }
    
    
    private void move() {
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        double maxSpeed = 0.7;
        double botHeading = -(imu.getAngularOrientation().firstAngle);
        
        double rx = ((gamepad1.right_stick_y) * Math.sin(botHeading) + (gamepad1.right_stick_x) *Math.cos(botHeading));

        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing. fix: magic number
        double y = -gamepad1.left_stick_y * .9; // Remember, this is reversed! fix: magic number
        double theta = Math.atan2(y, x);
        double power = Math.hypot(x, y);
        
        theta += botHeading;
        double sin = Math.sin(theta - Math.PI/4);
        double cos = Math.cos(theta - Math.PI/4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));
        
        double frontLeftPower = power * cos / max + rx;
        double frontRightPower = power * sin / max - rx;
        double backLeftPower = power * sin / max + rx;
        double backRightPower = power * cos / max - rx;

        fl.setPower(frontLeftPower * maxSpeed);
        bl.setPower(backLeftPower * maxSpeed);
        fr.setPower(frontRightPower * maxSpeed);
        br.setPower(backRightPower * maxSpeed);
    }
    
    
    private void slide() {
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        final int LOWEST_POSITION = 0;
        final int HIGHEST_POSITION = 2500;
        
        // left trigger down, right trigger up
        double triggerInput = gamepad2.right_trigger - gamepad2.left_trigger;
        double currentPosition = slide.getCurrentPosition();
        double remapControl = 1.5; // fix: vague name
        
        if (currentPosition < LOWEST_POSITION) {
            slide.setPower(0.1);
        }
        else if (currentPosition > HIGHEST_POSITION) {
            slide.setPower(-0.1);
        }
        else if (triggerInput > 0.3) {
        // if either trigger pressed
            double powerControl = remapRange(LOWEST_POSITION, HIGHEST_POSITION, 0, remapControl, currentPosition);
            powerControl = 1 / Math.pow(Math.abs(powerControl), 3);
            slide.setPower(powerControl * triggerInput);
        }
        else if (triggerInput < -0.3) {
            double powerControl = remapRange(HIGHEST_POSITION, LOWEST_POSITION-20, 0, remapControl, currentPosition);
            powerControl = 1 / Math.pow(Math.abs(powerControl), 3);
            
            slide.setPower(powerControl*triggerInput);
        }
        else {
            slide.setPower(0);
        }
        
        telemetry.addData("currentPosition", currentPosition);
    }
    
    
    void intake() {
        intakeArm();
        intakeWrist();
    }
    

    void intakeArm() {
        // manual control
        if (gamepad2.dpad_down) {
            setMotorMode(ia, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ia.setPower(0.25);
            return;
        }
        else if (gamepad2.dpad_up) {
            setMotorMode(ia, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ia.setPower(-0.25);
            return;
        }
        else {
            ia.setPower(0);
        }
        
        // auto positioning
        if (gamepad2.b && gamepad2.x) {
            ia.setTargetPosition(400);
            ia.setVelocity(500);
            ia.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if (gamepad2.x) {
            ia.setVelocity(600.0);
            ia.setTargetPosition(1679);
            ia.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if (gamepad2.b) {
            ia.setTargetPosition(700);
            ia.setVelocity(700);
            ia.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    
    
    private void intakeWrist() {
        if (gamepad2.a) {
            iwl.setPosition(.04);
            iwr.setPosition(.04);
        }
        else if (gamepad2.y) {
            iwl.setPosition(.281);
            iwr.setPosition(.281);
        }
    }
    
    
    private void intakeStart(){
        iwl.setPosition(0.0);
        iwr.setPosition(0.0);
    }
    
    
    private void intakeLaunch(){
        if (gamepad2.right_stick_button && gamepad2.left_stick_button){
            iwl.setPosition(20.0);
            iwr.setPosition(-20.0);
            if (servoPosCheck())
            {
                sleep(200);
                iwl.setPosition(0.0);
                iwr.setPosition(0.0);
            }
        }
    }
    
    
    private boolean servoPosCheck() {
        final double tolerance = 0.1;
        if (((iwl.getPosition() - 20.0)< tolerance) && ((iwr.getPosition() + 20.0) < tolerance)) {
            return true;
        }   
        else {
            return false;
        }
    }
    
    
    void setMotorMode(DcMotorEx motor, DcMotor.RunMode mode) {
        if (motor.getMode() != mode) {
            motor.setMode(mode);
        }
    }
    
    
    private void intakeSpinnerMove() {
        
        if (gamepad2.dpad_left) {
            while (gamepad2.dpad_left) {
                is.setPower(0.2);
            }
        }
        else {
            if (gamepad2.dpad_right) {
                while (gamepad2.dpad_right) {
                   is.setPower(-0.2) ;
                }
            }
        }
    }
    
    
    private void gripsForward() {
        if (gamepad2.left_stick_button && !gamepad2.right_stick_button) {
            moveGripperServo = 0;
        }
        if (gamepad2.right_stick_button && !gamepad2.left_stick_button) {
            moveGripperServo += .005;
        }
        
        lg.setPosition(moveGripperServo);
        rg.setPosition(moveGripperServo);
    }
    
    
    private void sucks() { // fix: rename
        if ((gamepad2.left_stick_y) > 0.2) {
            is.setPower(gamepad2.left_stick_y/4);
        }
        else if (gamepad2.left_stick_y < 0.2) {
            is.setPower(gamepad2.left_stick_y);
        }
        else {
            is.setPower(0);
        }
    }
    
    
    private void armMove() {
        if (gamepad2.right_bumper && gamepad2.left_bumper) {
            arm.setTargetPosition(0);
            arm.setVelocity(500);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if (gamepad2.right_bumper) {
            while (gamepad2.right_bumper) {
                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                arm.setPower(0.2);
            }
        }
        else if (gamepad2.left_bumper) {
            while (gamepad2.left_bumper) { // fix: under no circumstances should we use whiles. they break the flow of the entire opmode
                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                arm.setPower(-0.2);
            }
        }
        else {
            arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            arm.setPower(0);
        }
        
    }
}
