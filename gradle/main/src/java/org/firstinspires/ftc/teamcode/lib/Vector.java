package org.firstinspires.ftc.teamcode.lib;


/**
 * FILL ME OUT
 * @author Neil N
 */
public class Vector {
    private final double magnitude;
    private final double angle;
    private final double x;
    private final double y;

    /**
     * FILL ME OUT
     * @param m FILL ME OUT
     * @param a FILL ME OUT
     */
    public Vector(double m, double a) {
        if (m < 0) {
            this.magnitude = -m;
            this.angle = a + Math.PI;
        }
        else {
            this.magnitude = m;
            this.angle = a;
        }
        
        this.x = Math.cos(a) * m;
        this.y = Math.sin(a) * m;
    }

    /**
     * FILL ME OUT
     * @param vectors FILL ME OUT
     * @return FILL ME OUT
     */
    public Vector add(Vector[] vectors) {
        double totaly = 0;
        double totalx = 0;
        for (int i = 0; i < vectors.length; i++) {
            totalx += vectors[i].x;
            totaly += vectors[i].y; 
        }
            
        double vectorangle = Math.atan2(totaly, totalx);
        // if (totalx < 0 && totaly >= 0) {
        //     angle = (Math.PI) - Math.abs(angle); }
        // else if (totalx < 0 && totaly < 0) {
        //     angle = Math.PI + Math.abs(angle); }
        // else if (totalx > 0 && totaly < 0) {
        //     angle = (Math.PI) * 2 - Math.abs(angle);
        // }
        double vectormagnitude = Math.sqrt((totalx*totalx)+(totaly*totaly));
        return new Vector(vectormagnitude, vectorangle);
    }


    /**
     *  FILL ME OUT
     * @return FILL ME OUT
     */
    public double getX() {
        return this.x;
    }

    /**
     * FILL ME OUT
     * @return FILL ME OUT
     */
    public double getY() {
        return this.y;
    }

    /**
     * FILL ME OUT
     * @return FILL ME OUT
     */
    public double getMagnitude() {
        return this.magnitude;
    }

    /**
     * FILL ME OUT
     * @return FILL ME OUT
     */
    public double getAngle() {
        return this.angle;
    }
}