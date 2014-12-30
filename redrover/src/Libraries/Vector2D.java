package Libraries;

import com.sun.squawk.util.MathUtils;

/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/**
 * This class represents a 2 Dimensional vector. It has support for using polar and cartesian coordinates interchangably.
 * All angle measurements are in degrees.
 * The coordinate plane has been shifted 90 degrees, so a vector with an angle of 0 points up, and angle of 90 points left, etc.
 *
 * @author michaelsilver
 */
public class Vector2D {
    
    double x; //The x coordinate of the vector
    double y; //The y coordinate of the vector
    
    /**
     * Constructs a 2D vector object
     * 
     * @param cartesian If true, coordinate should be specified in Cartesian form. IF false, they should be specified in polar form.
     * @param abscissa If cartesian, this is the x coordinate. Else, this is the magnitude.
     * @param ordinate If cartesian, this is the y coordinate. Else, this is the angle in degrees.
     */
    public Vector2D(boolean cartesian, double abscissa, double ordinate){
        if(cartesian){
            x = abscissa;
            y = ordinate;
        } else {
            double[] coords = polarToCart(abscissa, ordinate);
            x = coords[0];
            y = coords[1];
        }
    }
    
    /**
     * Converts polar coordinates into cartesian.
     * @param r The magnitude of the polar coordinates.
     * @param theta The angle of the polar coordinates.
     * @return A double array of size 2. Index 0 is the x value of the coordinates. Index 1 is the y value of the coordinates.
     */
    public static double[] polarToCart(double r, double theta){
        theta = Math.toRadians(theta - 90);
        double[] cartCoords = {r*Math.cos(theta), r*Math.sin(theta)};
        return cartCoords;
    }
    
//    GETTERS
    
    /**
     * Get the x value of the vector.
     * 
     * @return the x value of the vector
     */
    public double getX(){
        return x;
    } 
    
    /**
     * Get the y value of the vector.
     * 
     * @return the y value of the vector
     */
    public double getY(){
        return y;
    }
    
    /**
     * Get the angle value of the vector.
     * 
     * @return the angle of the vector in degrees
     */
    public double getAngle(){
        return Math.toDegrees(MathUtils.atan2(y, x)) + 90;
    }
    
    /**
     * Get the magnitude value of the vector.
     * 
     * @return the magnitude of the vector
     */
    public double getMagnitude() {
        return Math.sqrt(x*x + y*y);
    }
    
//    SETTERS
    
    /**
     * Set the x value of the vector.
     * 
     * @param x the x value of the vector
     */
    public void setX(double x){
        this.x = x;
    }
    
    /**
     * Set the y value of the vector.
     * 
     * @param y the y value of the vector
     */
    public void setY(double y){
        this.y = y;
    }
    
    /**
     * Set the angle value of the vector.
     * 
     * @param angle the angle of the vector in degrees
     */
    public void setAngle(double angle)
    {
        double[] newCoords = Vector2D.polarToCart(getMagnitude(), angle);
        x = newCoords[0];
        y = newCoords[1];
    }
    
    /**
     * Set the magnitude value of the vector.
     * 
     * @param magnitude the magnitude of the vector
     */
    public void setMagnitude(double magnitude)
    {
        double[] newCoords = Vector2D.polarToCart(magnitude, getAngle());
        x = newCoords[0];
        y = newCoords[1];
    }
       
//    MATH OPERATIONS
    
    /**
     * Adds the vectors represented by 2 Vector2D objects.
     * 
     * @param vector1 The first vector to add
     * @param vector2 The second vector to add
     * @return The sum of the 2 vectors
     */
    public static Vector2D addVectors(Vector2D vector1, Vector2D vector2){
        Vector2D sum = new Vector2D(true, vector1.getX() + vector2.getX(), vector1.getY() + vector2.getY());
        return sum;
    }
    
    /**
     * Subtracts the vectors represented by 2 Vector2D objects.
     * 
     * @param vector1 The first vector to subtract
     * @param vector2 The second vector to subtract
     * @return The difference of the 2 vectors
     */
    public static Vector2D subtractVectors(Vector2D vector1, Vector2D vector2){
        Vector2D sum = new Vector2D(true, vector1.getX() - vector2.getX(), vector1.getY() - vector2.getY());
        return sum;
    }
    
    /**
     * Get the unit vector of the vector
     * 
     * @return A vector with the same angle as this vector with a magnitude of 1
     */
    public Vector2D unitVector(){
        Vector2D unitVector = new Vector2D(true, x/getMagnitude(), y/getMagnitude());
        return unitVector;
    }
}
