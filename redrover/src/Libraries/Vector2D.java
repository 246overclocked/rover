/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package Libraries;

import com.sun.squawk.util.MathUtils;

/**
 *
 * @author michaelsilver
 */
public class Vector2D {
    
    double x;
    double y;
    
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
    
    public static double[] polarToCart(double r, double theta){
        double[] cartCoords = {r*Math.cos(theta), r*Math.sin(theta)};
        return cartCoords;
    }
    
//    GETTERS
    public double getX(){
        return x;
    } 
    
    public double getY(){
        return y;
    }
    
    public double getAngle(){
        return MathUtils.atan2(y, x);
    }
    
    public double getMagnitude() {
        return Math.sqrt(x*x + y*y);
    }
    
//    SETTERS
    public void setX(double x){
        this.x = x;
    }
    
    public void setY(double y){
        this.y = y;
    }
    
    public void setAngle(double angle)
    {
        double[] newCoords = Vector2D.polarToCart(getMagnitude(), angle);
        x = newCoords[0];
        y = newCoords[1];
    }
    
    public void setMagnitude(double magnitude)
    {
        double[] newCoords = Vector2D.polarToCart(magnitude, getAngle());
        x = newCoords[0];
        y = newCoords[1];
    }
       
//    MATH OPERATIONS
    public static Vector2D addVectors(Vector2D vector1, Vector2D vector2){
        Vector2D sum = new Vector2D(true, vector1.getX() + vector2.getX(), vector1.getY() + vector2.getY());
        return sum;
    }
    
    public static Vector2D subtractVectors(Vector2D vector1, Vector2D vector2){
        Vector2D sum = new Vector2D(true, vector1.getX() - vector2.getX(), vector1.getY() - vector2.getY());
        return sum;
    }
    
    public Vector2D unitVector(){
        Vector2D unitVector = new Vector2D(true, x/getMagnitude(), y/getMagnitude());
        return unitVector;
    }
}
