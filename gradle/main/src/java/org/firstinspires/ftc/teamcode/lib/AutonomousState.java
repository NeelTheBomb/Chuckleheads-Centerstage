package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.Servo;


/**
 * Autonomous state machine. BROKEN, UNTESTED.
 * @author Nathan W
 */
public enum AutonomousState {
    /**
     * The entry enum. All autonomous program should start here instead of at SCAN in order to make
     * possible changes in the future easier.
     */
    ENTRY {
        @Override
        public AutonomousState run() {
            gripperClose();
            return SCAN;
        }
    },

    /**
     * Scans in front of the robot for our team prop, returning LEFT, MIDDLE, or RIGHT when
     * appropriate.
     */
    SCAN {
        @Override
        public AutonomousState run() {
            return LEFT;
            // return MIDDLE;
            // return RIGHT;
        }
    },

    /**
     * Drives towards the left purple pixel position.
     */
    LEFT {
        @Override
        public AutonomousState run() {
            int targetX = 0;
            int targetY = 500;
            int stage = 0;
            
            turnToAngle(-90);
            
            if (stage == 0 && moveToPos(targetX, targetY)) {
                stage = 1;
                targetX = -100;
                targetY = 500;
            }
            else if (stage == 1 && moveToPos(targetX, targetY)) {
                return DROP;
            }
            
            return LEFT;
        }
    },

    /**
     * Drives towards the middle purple pixel position.
     */
    MIDDLE {
        @Override
        public AutonomousState run() {
            int targetX = 0;
            int targetY = 600;
            int stage = 0;
            
            if (stage == 0 && moveToPos(targetX, targetY)) {
                stage = 1;
                targetX = 0;
                targetY = 800;
            }
            else if (stage == 1 && moveToPos(targetX, targetY)) {
                return DROP;
            }
            
            return MIDDLE;
        }
    },

    /**
     * Drives towards the right purple pixel position.
     */
    RIGHT {
        @Override
        public AutonomousState run() {
            int targetX = 0;
            int targetY = 500;
            int stage = 0;
            
            turnToAngle(90);
            
            if (stage == 0 && moveToPos(targetX, targetY)) {
                stage = 1;
                targetX = 100;
                targetY = 500;
            }
            else if (stage == 1 && moveToPos(targetX, targetY)) {
                return DROP;
            }
            
            return RIGHT;
        }
    },

    /**
     * Drops the pixel.
     */
    DROP {
        @Override
        public AutonomousState run() {
            gripperOpen();
            return STOP;
        }
    },

    /**
     * Does nothing and returns itself, effectively stopping the robot for the rest of the runtime.
     */
    STOP {
        @Override
        public AutonomousState run() {
            return STOP;
        }
    };

    /**
     * Runs the process for the current state.
     * @return The next state.
     */
    public abstract AutonomousState run();

    private static Alliance alliance;
    private static Side side;
    private static Servo gl, gr; // gripperLeft, gripperRight
    private static double iterationTime = System.currentTimeMillis();

    /**
     * Initializes the AutonomousState. PreciseMovement should be initialized before this.
     * @param alliance Current alliance
     * @param side Current side
     * @param gl Left gripper servo
     * @param gr Right gripper servo
     * @see PreciseMovement
     */
    public static void init(Alliance alliance, Side side, Servo gl, Servo gr) {
        AutonomousState.alliance = alliance;
        AutonomousState.side = side;
        AutonomousState.gl = gl;
        AutonomousState.gr = gr;
    }


    /**
     * Do nothing for a given amount of seconds.
     * @param seconds The amount of seconds to wait for
     */
    void wait(double seconds) {
        double start = System.currentTimeMillis();
        while (seconds * 1000 > System.currentTimeMillis() - start) {}
    }


    /**
     * Wraps around PreciseMovement.moveToPos(), updating the PreciseMovement position, keeping
     * track of iterationTime, and making calling easier.
     * @param targetX The field-centric X position to go to
     * @param targetY The field-centric Y position to go to
     * @return If the robot's current position is at the target position, with tolerance
     * @see PreciseMovement
     */
    boolean moveToPos(int targetX, int targetY) {
        double currentTime = System.currentTimeMillis();
        PreciseMovement.updatePos();
        boolean atTarget = PreciseMovement.moveToPos(targetX, targetY, currentTime-iterationTime);
        iterationTime = System.currentTimeMillis();
        return atTarget;
    }

    /**
     * Wraps around PreciseMovement.turnToAngle(), updating the PreciseMovement position, keeping
     * track of iterationTime, and making calling easier.
     * @param targetAngle The target angle relative to the robot's starting position
     * @return If the robot's current rotation is at the target rotation, with tolerance
     * @see PreciseMovement
     */
    boolean turnToAngle(double targetAngle) {
        double currentTime = System.currentTimeMillis();
        PreciseMovement.updatePos();
        boolean atTarget = PreciseMovement.turnToAngle(targetAngle, currentTime-iterationTime);
        iterationTime = System.currentTimeMillis();
        return atTarget;
    }

    /**
     * Closes the gripper.
     */
    void gripperClose() {
        gl.setPosition(0.0);
        gr.setPosition(0.0);
    }

    /**
     * Opens the gripper.
     */
    void gripperOpen() {
        gl.setPosition(0.4);
        gr.setPosition(-0.4);
    }
}