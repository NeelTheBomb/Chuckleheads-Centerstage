package org.firstinspires.ftc.teamcode.lib;

// first
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;

// our static stuff
import static org.firstinspires.ftc.teamcode.lib.MathStuff.sqr;
import static org.firstinspires.ftc.teamcode.lib.MathStuff.shortestAngleRemapped;
import static org.firstinspires.ftc.teamcode.lib.RobotHardware.*;


/**
 * This being static could make it broken. Further testing is needed. It may have to be turned into an object you initialize.
 * @author Neel N
 */
public class PreciseMovement {
    // MovementPID
    private static double movementKp, movementKi, movementKd, movementIntegralPrior, movementErrorPrior;

    // AnglePID
    private static double angleKp, angleKi, angleKd, angleIntegralPrior, angleErrorPrior, movementTolerance, angleTolerance;

    // Positions and Motor Ticks
    private static double xPos, yPos, flPastTick, frPastTick, blPastTick, brPastTick, opvPastTick, ophPastTick;


    /******* PUBLIC FUNCTIONS ********/
    public static void preciseMovementInit() {
        PreciseMovement.movementKp = 0.5;
        PreciseMovement.movementKi = 0.0;
        PreciseMovement.movementKd = 0.0;
        PreciseMovement.movementIntegralPrior = 0.0;
        PreciseMovement.movementErrorPrior = 0.0;

        PreciseMovement.angleKp = 0.5;
        PreciseMovement.angleKi = 0.0;
        PreciseMovement.angleKd = 0.0;
        PreciseMovement.angleIntegralPrior = 0.0;
        PreciseMovement.angleErrorPrior = 0.0;

        PreciseMovement.movementTolerance = 10.0;
        PreciseMovement.angleTolerance = 0.05;

        PreciseMovement.xPos = 0.0;
        PreciseMovement.yPos = 0.0;
        PreciseMovement.flPastTick = 0.0;
        PreciseMovement.frPastTick = 0.0;
        PreciseMovement.blPastTick = 0.0;
        PreciseMovement.brPastTick = 0.0;
    }
    /**
     * moves to a predefined position
     * @param targetPosX the target x position you want to robot to move to
     * @param targetPosY the target y position you want the robot to move to
     * @param iterationTime the time between each time this function is called. Have to use a timer in main code this is being used in. Used to make PID more accurate.
     * @return returns a boolean that is true if the current position is less than 10 away from the target position
     */
    public static boolean moveToPos(double targetPosX, double targetPosY, double iterationTime) {
        double[] moveConstants = desiredVector(xPos, yPos, targetPosX, targetPosY, iterationTime);

        robotMove(moveConstants[0], moveConstants[1], 0);

        return distanceFromPos(targetPosX, targetPosY) < movementTolerance;
    }

    /**
     * This function moves the robot closer to a target angle from its current angle each time it is called.
     * @param targetAngle the angle the robot should be facing
     * @param iterationTime the time between each time this function is called. Used for PID
     * @return returns a boolean that is true of the currentanle is within a set tolerance of the target angle
     */
    public static boolean turnToAngle(double targetAngle, double iterationTime) {
        double[] angleValues = PID(angleKp, angleKd, angleKi, -imu.getAngularOrientation().firstAngle, targetAngle,
                angleIntegralPrior, angleErrorPrior, iterationTime);

        double angleDifference = angleValues[0];
        angleErrorPrior = angleValues[1];
        angleIntegralPrior = angleValues[2];
        robotMove(0, 0, angleDifference);

        return Math.abs(shortestAngleRemapped(currentAngle(), targetAngle)) < angleTolerance;
    }

    /**
     * Updates the position of the robot. Uses all 4 motor encoders to do this. Finds the delta of encoder value of the motors and turns this into a vector.
     * Adds these vectors together to find the change in the robots position. Adds this change to the current position to find the updated current position of the robot.
     */
    public static void updatePos() {
        double flTick = fl.getCurrentPosition() - flPastTick;
        double frTick = fr.getCurrentPosition() - frPastTick;
        double blTick = bl.getCurrentPosition() - blPastTick;
        double brTick = br.getCurrentPosition() - brPastTick;

        MathVector currentVector = finalWheelVector(flTick, frTick, blTick, brTick, -imu.getAngularOrientation().firstAngle);
        double[] pos = newPosition(xPos, yPos, currentVector);
        xPos = pos[0];
        yPos = pos[1];

        // remember past encoder values
        flPastTick = fl.getCurrentPosition();
        frPastTick = fr.getCurrentPosition();
        blPastTick = bl.getCurrentPosition();
        brPastTick = br.getCurrentPosition();
    }


    /**
     * same as update pos but uses deadwheel encoder values instead.
     * @return returns current position of dead wheels for TESTING purposes. Return can be gotton rid of after.
     */
    public static double[] updateDeadWheelPos() {
        double ophTick = oph.getCurrentPosition() - ophPastTick;
        double opvTick = opv.getCurrentPosition() - opvPastTick;

        MathVector currentVector = finalDeadWheelVector(ophTick, opvTick, -imu.getAngularOrientation().firstAngle);
        double[] pos = newPosition(xPos, yPos, currentVector);
        xPos = pos[0];
        yPos = pos[1];

        // remember past encoder values
        ophPastTick = oph.getCurrentPosition();
        opvPastTick = opv.getCurrentPosition();
        return new double[] {ophTick, opvTick, opv.getCurrentPosition(), oph.getCurrentPosition()};
    }


    /**
     * converts enoder changes to vectors. Adds imu angle to translate vectors to angle of robot. Adds vectors to find change in robots position.
     * @param deltaOph Encoder change of horizontal dead wheel
     * @param deltaOpv Encoder change of vertical dead wheel
     * @param robotAngle angle of robot gotton from imu in radians
     * @return returns vector representing change in robots position
     */
    private static MathVector finalDeadWheelVector(double deltaOph, double deltaOpv, double robotAngle) {
        MathVector ophVector = new MathVector(deltaOph, Math.PI + robotAngle);
        MathVector opvVector = new MathVector(deltaOpv, Math.PI/2 + robotAngle);

        MathVector[] deadWheelVectors = new MathVector[] {ophVector, opvVector};
        // just using a random vector to use the add function in the vector class
        MathVector finalVector = ophVector.add(deadWheelVectors);
        return new MathVector(finalVector.magnitude, finalVector.angle + robotAngle);
    }


    /**
     * returns the distance the current pos is from the target pos
     * @param targetPosX the x target position
     * @param targetPosY the y target position
     * @return returns the distance the currentposition is from the target position
     */
    public static double distanceFromPos(double targetPosX, double targetPosY) {
        return Math.sqrt(sqr(targetPosX-xPos) + sqr(targetPosY-yPos));
    }


    /******* GETTERS ********/
    /**
     * getter for position
     * @return returns the current x and y position as array
     */
    public static double[] getPos() { return new double[] { xPos, yPos }; }


    /******* SETTERS ********/
    /**
     * sets tolerance movement that gets used in the robot movement function to determine wheter to return true of not
     * @param t tolerance you want for distance from currentPosition to targetPosition. (How precise the movement has to be to a position)
     */
    public static void setMovementTolerance(double t) { movementTolerance = t; }

    /**
     * controls the tolerance of whether the turnToAngle returns true. True means it should stop
     * @param t tolerance of how close robot should get to angle in turnToAngle function
     */
    public static void setAngleTolerance(double t) { angleTolerance = t; }

    /**
     * This controls the Kp, Kd, and Ki values of the PID for the movement of the robot. This control the the curve of how it approaches a target position.
     * @param kp Increasing this makes the robot move to the target position in a more proportional matter. It will move faster when further from the target position and slower when closer
     * @param kd This makes the robot slow down if it reaches the targetPosition to quickly by adding a negative force.
     * @param ki This makes the robot speed up if it does not move to the targetPosition quick enough
     */
    public static void setMovementPID(double kp, double kd, double ki) {
        movementKp = kp;
        movementKd = kd;
        movementKi = ki;
        movementIntegralPrior = 0;
        movementErrorPrior = 0;
    }

    /**
     * This controls the Kp, Kd, and Ki values of the PID for the movement of angle turning of the robot. This control the the curve of how it turns to a target angle
     * @param Kp Increasing this makes the robot move to the target angle in a more proportional matter. It will move faster when further from the target angle and slower when closer
     * @param Kd This makes the robot slow down if it reaches the target angle to quickly by adding a force in the opposite direction.
     * @param Ki This makes the robot speed up if it does not move to the target angle quick enough
     */
    public static void setAnglePID(double Kp, double Kd, double Ki) {
        angleKp = Kp;
        angleKd = Kd;
        angleKi = Ki;
        angleIntegralPrior = 0;
        angleErrorPrior = 0;
    }


    /******* PRIVATE FUNCTIONS ********/
    /**
     * FILL ME OUT
     * @param deltaFl change in front left motor encoder
     * @param deltaFr change in front right motor encoder
     * @param deltaBl change in back left motor encoder
     * @param deltaBr change in back right motor encoder
     * @param robotAngle current angle of robot
     * @return returns vector representing change in robots position
     */
    private static MathVector finalWheelVector(double deltaFl, double deltaFr, double deltaBl, double deltaBr, double robotAngle) {
        MathVector flvector = new MathVector(deltaFl, Math.PI*0.25 + robotAngle);
        MathVector frvector = new MathVector(deltaFr, Math.PI*0.75 + robotAngle);
        MathVector blvector = new MathVector(deltaBl, Math.PI*0.75 + robotAngle);
        MathVector brvector = new MathVector(deltaBr, Math.PI*0.25 + robotAngle);

        MathVector[] wheelVectors = new MathVector[] { flvector, frvector, blvector, brvector };
        // just using a random vector to use the add function in the vector class
        MathVector finalVector = blvector.add(wheelVectors);
        return new MathVector(finalVector.magnitude, finalVector.angle + robotAngle);
    }


    /**
     * FILL ME OUT
     * @param oldX FILL ME OUT
     * @param oldY FILL ME OUT
     * @param finalVector FILL ME OUT
     * @return FILL ME OUT
     */
    private static double[] newPosition(double oldX, double oldY, MathVector finalVector) {
        return new double[] { oldX + finalVector.x, oldY + finalVector.y };
    }


    /**
     * FILL ME OUT
     * @param currentX FILL ME OUT
     * @param currentY FILL ME OUT
     * @param desiredX FILL ME OUT
     * @param desiredY FILL ME OUT
     * @param iterationTime FILL ME OUT
     * @return FILL ME OUT
     */
    private static double[] desiredVector(double currentX, double currentY, double desiredX, double desiredY, double iterationTime) {
        double targetDistance = Math.sqrt(sqr(desiredX-currentX) * sqr(desiredY-currentY));
        double angle = Math.atan2((desiredX-currentX), (desiredY-currentY));

        double[] wheelValues = PID(movementKp, movementKd, movementKi, 0, targetDistance,
                movementIntegralPrior, movementErrorPrior, iterationTime);

        movementErrorPrior = wheelValues[1];
        movementIntegralPrior = wheelValues[2];

        return new double[] { Math.cos(angle) * wheelValues[0], Math.sin(angle) * wheelValues[0] };
    }


    /**
     * converts enoder changes to vectors. Adds imu angle to translate vectors to angle of robot. Adds vectors to find change in robots position.
     * @param rotX FILL ME OUT
     * @param rotY FILL ME OUT
     * @param rx FILL ME OUT
     */
    private static void robotMove(double rotX, double rotY, double rx) {
        final double MAX_SPEED = 0.75;

        // Read inverse IMU heading, as the IMU heading is CW positive
        double botHeading = -imu.getAngularOrientation().firstAngle;

        rotX = (rotX * Math.cos(botHeading) - rotY * Math.sin(botHeading)) / 2;
        rotY = (rotX * Math.sin(botHeading) + rotY * Math.cos(botHeading)) / 2;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + (rx), 1);
        double frontLeftPower = (rotY + rotX - rx) / denominator;
        double backLeftPower = (rotY - rotX - rx) / denominator;
        double frontRightPower = (rotY - rotX + rx) / denominator;
        double backRightPower = (rotY + rotX  + rx) / denominator;

        fl.setPower(frontLeftPower * MAX_SPEED);
        bl.setPower(backLeftPower * MAX_SPEED);
        fr.setPower(frontRightPower * MAX_SPEED);
        br.setPower(backRightPower * MAX_SPEED);
    }


    /**
     * FILL ME OUT
     * @param kp causes actualValue to move to desiredValue in a more proportional matter based on the error(or difference between them)
     * @param kd causes actual value to decrease if it is moving to quickly toward desiredValue
     * @param ki causes actual value to increase if it is moving to slowly toward desiredValue
     * @param actualValue a current value you pass through
     * @param desiredValue the value you want the actualValue to reach
     * @param integralPrior the previous integral from the last time the function was called
     * @param errorPrior the previous error from the last time the function was called
     * @param iterationTime the time between each time this function is called. Timer will need to be used
     * @return the amount actualValue should change by to reach the desiredValue
     */
    private static double[] PID(double kp, double kd, double ki, double actualValue, double desiredValue,
                                double integralPrior, double errorPrior, double iterationTime) {

        double error = desiredValue - actualValue;
        double integral = integralPrior + error * iterationTime;
        double derivative = (error - errorPrior) / iterationTime;
        double output = kp*error + ki*integral + kd*derivative;

        return new double[] { output, error, integral };
    }


    /**
     * returns the current angle of the robot
     * @return current angle of the robot in radians
     */
    private static double currentAngle() {
        return -imu.getAngularOrientation().firstAngle;
    }
}