package org.firstinspires.ftc.teamcode.lib;

import java.util.Vector;


/**
 * Represents a vector
 * @author Neil N
 */
public class MathVector {
    public final double magnitude;
    public final double angle;
    public final double x;
    public final double y;

    /**
     * creates vector
     * @param m magnitude
     * @param a angle
     */
    public MathVector(double m, double a) {
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
     * adds an array of vectors together
     * @param Vectors vector array to be added
     * @return returns a new MathVector
     */
    public static MathVector add(MathVector[] vectors) {
        double totaly = 0;
        double totalx = 0;
        for (int i = 0; i < vectors.length; i++) {
            totalx += vectors[i].x;
            totaly += vectors[i].y;
        }

        double vectorangle = Math.atan2(totaly, totalx);
        double vectormagnitude = Math.sqrt((totalx*totalx)+(totaly*totaly));
        return new MathVector(vectormagnitude, vectorangle);
    }
}
