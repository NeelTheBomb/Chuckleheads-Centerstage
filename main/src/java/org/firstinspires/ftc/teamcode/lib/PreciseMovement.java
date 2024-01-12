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
 * Autonomous movement.
 * @author Neel N
 */
public class PreciseMovement {
    // MovementPID
    private static double movementKp, movementKi, movementKd, movementIntegralPrior, movementErrorPrior;

    // AnglePID
    private static double angleKp, angleKi, angleKd, angleIntegralPrior, angleErrorPrior, movementTolerance, angleTolerance;

    // Positions and Motor Ticks
    private static double xPos, yPos, ophPastTick, opvPastTick;


    /******* PUBLIC FUNCTIONS ********/
    /**
     *  ialize the starting values. Must be called after RobotHardwareInit().
     * @see RobotHardware
     */
    public static void preciseMovementInit() {
        PreciseMovement.movementKp = 0.1;
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
    }
    /**
     * attempts to move the robot to the given position
     * @param targetPosX the x position you want to robot to move to
     * @param targetPosY the y position you want the robot to move to
     * @param iterationTime delta-time between now and last loop. Used for the PIDs
     * @return whether or not the current position is less than movement tolerance
     */
    public static boolean moveToPos(double targetPosX, double targetPosY, double iterationTime) {
        updatePos();
        double[] moveConstants = desiredVector(xPos, yPos, targetPosX, targetPosY, iterationTime);
        
        if (Math.abs(xPos - targetPosX) > movementTolerance && Math.abs(yPos - targetPosY) > movementTolerance) {
            robotMove(moveConstants[0], moveConstants[1], 0);
        }

        return distanceFromPos(targetPosX, targetPosY) < movementTolerance;
    }

    /**
     * attempts to move the robot closer to a target angle from its current angle
     * @param targetAngle the angle the robot should be facing
     * @param iterationTime delta-time between now and last loop. Used for the PIDs
     * @return whether or not the current angle is within the angle tolerance
     */
    public static boolean turnToAngle(double targetAngle, double iterationTime) {
        double[] angleValues = PID(angleKp, angleKd, angleKi, currentAngle(), targetAngle,
                angleIntegralPrior, angleErrorPrior, iterationTime);

        double angleDifference = angleValues[0];
        angleErrorPrior = angleValues[1];
        angleIntegralPrior = angleValues[2];
        robotMove(0, 0, angleDifference);

        return Math.abs(shortestAngleRemapped(currentAngle(), targetAngle)) < angleTolerance;
    }

    /**
     * Updates the position of the robot using deadwheals. Uses all 4 motor 
     * encoders to do this. Finds the delta of encoder value of the motors and 
     * turns this into a vector. Adds these vectors together to find the change 
     * in the robots position. Adds this change to the current position to find 
     * the updated current position of the robot.
     */
    public static void updatePos() {
        double ophTick = oph.getCurrentPosition() - ophPastTick;
        double opvTick = opv.getCurrentPosition() - opvPastTick;

        MathVector currentVector = finalDeadWheelVector(ophTick, opvTick, currentAngle());
        double[] pos = newPosition(xPos, yPos, currentVector);
        xPos = pos[0];
        yPos = pos[1];

        // remember past encoder values
        ophPastTick = oph.getCurrentPosition();
        opvPastTick = opv.getCurrentPosition();
    }


    /**
     * converts encoder changes to vectors. Adds imu angle to translate vectors to angle of robot. Adds vectors to find change in robots position.
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
        return new MathVector(finalVector.magnitude, finalVector.angle);
    }


    /**
     * finds the distance between the current and target robot positions
     * @param targetPosX the x target position
     * @param targetPosY the y target position
     * @return the distance between the current and target robot positions
     */
    public static double distanceFromPos(double targetPosX, double targetPosY) {
        return Math.sqrt(sqr(targetPosX-xPos) + sqr(targetPosY-yPos));
    }


    /******* GETTERS ********/
    /**
     * gets the current position
     * @return returns the current x and y position
     */
    public static double[] getPos() { return new double[] { xPos, yPos }; }


    /******* SETTERS ********/
    /**
     * sets movement tolerance
     * @param t tolerance you want for distance from currentPosition to targetPosition. (How precise the movement has to be)
     */
    public static void setMovementTolerance(double t) { movementTolerance = t; }

    /**
     * sets angle tolerance
     * @param t tolerance you want for distance from currentAngle to targetAngle. (How precise the turning has to be)
     */
    public static void setAngleTolerance(double t) { angleTolerance = t; }

    /**
     * sets the Kp, Kd, and Ki values of the PID for the movement of the robot. This control the the curve of how it approaches a target position.
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
     * sets the Kp, Kd, and Ki values of the PID for the movement of angle turning of the robot. This control the the curve of how it turns to a target angle
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
     * gets encoder deltas from wheels and makes vector representing change in position
     * @param deltaFl change in front left motor encoder
     * @param deltaFr change in front right motor encoder
     * @param deltaBl change in back left motor encoder
     * @param deltaBr change in back right motor encoder
     * @param robotAngle current angle of robot
     * @return returns vector representing change in robots position
     */
    private static MathVector finalWheelVector(double deltaFl, double deltaFr, 
            double deltaBl, double deltaBr, double robotAngle) {
                
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
     * adds the vector representing change in position to the current position
     * @param oldX old x position
     * @param oldY old y position
     * @param finalVector vector representing change in position
     * @return an array with the updated current x position and current y position
     */
    private static double[] newPosition(double oldX, double oldY, MathVector finalVector) {
        return new double[] { oldX + finalVector.x, oldY + finalVector.y };
    }


    /**
     * gets the current position and target postiion, deciding how much power should be distributed to motor for it to move.
     * @param currentX the current x position of robot
     * @param currentY the current y position of robot
     * @param desiredX target x position
     * @param desiredY target y position
     * @param iterationTime the time between each time this function is called. Used for PID control
     * @return an array representing how much power should be distributed vertically and horizontally to robot
     */
    private static double[] desiredVector(double currentX, double currentY, double desiredX, double desiredY, double iterationTime) {
        double targetDistance = Math.sqrt(sqr(desiredX-currentX) + sqr(desiredY-currentY));
        double angle = Math.atan2((desiredY-currentY), (desiredX-currentX));

        double[] wheelValues = PID(movementKp, movementKd, movementKi, 0, targetDistance,
                movementIntegralPrior, movementErrorPrior, iterationTime);

        movementErrorPrior = wheelValues[1];
        movementIntegralPrior = wheelValues[2];
        // double wheelValues[] = {targetDistance/1000};

        return new double[] { Math.cos(angle) * wheelValues[0], Math.sin(angle) * wheelValues[0] };
        // return new double[] { 0, .4 };
    }


    /**
     * converts encoder changes to vectors. Adds imu angle to translate vectors to angle of robot. Adds vectors to find change in robots position.
     * @param rotX movement power along x axis. This is field centric
     * @param rotY movemnet power along y axis. This is field centric
     * @param rx turning power. Increasing this makes the robot turn while moving
     */
    private static void robotMove(double rotX, double rotY, double rx) {
        final double MAX_SPEED = .7;

        double botHeading = currentAngle();

        // rotX = (rotX * Math.cos(botHeading) - rotY * Math.sin(botHeading)) / 2;
        // rotY = (rotX * Math.sin(botHeading) + rotY * Math.cos(botHeading)) / 2;

        // double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + (rx), 1);
        // double frontLeftPower = (rotY + rotX - rx) / denominator;
        // double frontRightPower = (rotY - rotX + rx) / denominator;
        // double backLeftPower = (rotY - rotX - rx) / denominator;
        // double backRightPower = (rotY + rotX  + rx) / denominator;

        // fl.setPower(frontLeftPower * MAX_SPEED);
        // bl.setPower(backLeftPower * MAX_SPEED);
        // fr.setPower(frontRightPower * MAX_SPEED);
        // br.setPower(backRightPower * MAX_SPEED);

        rotX *= -1;
        double x = rotX * 1.1; // Counteract imperfect strafing. fix: magic number
        double y = rotY * .9; // Remember, this is reversed! fix: magic number
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

        fl.setPower(frontLeftPower * MAX_SPEED);
        bl.setPower(backLeftPower * MAX_SPEED);
        fr.setPower(frontRightPower * MAX_SPEED);
        br.setPower(backRightPower * MAX_SPEED);
    }


    /**
     * Used to control the curve of how much a value should change as it approachs a target.
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
    private static double[] PID(double kp, double kd, double ki, 
            double actualValue, double desiredValue, double integralPrior, 
            double errorPrior, double iterationTime) {

        double error = desiredValue - actualValue;
        double integral = integralPrior + error * iterationTime;
        double derivative = (error - errorPrior) / iterationTime;
        double output = kp*error + ki*integral + kd*derivative;

        return new double[] { output, error, integral };
    }


    /**
     * gets the current angle of the robot in radians
     * @return current angle of the robot in radians
     */
    private static double currentAngle() {
        return -imu.getAngularOrientation().firstAngle;
    }
}
