package project;

import javax.vecmath.Vector3d;

// Spring class for 599 assignment 1
public class Spring {

    public Particle p1 = null;
    public Particle p2 = null;
    
    // All springs share the same stiffness coefficient
    public static double k = 1;
    
    // All springs share the same damping coefficient
    public static double b = 1;
    
    // Rest length
    public double l0 = 0;
    
    // Creates a spring connecting two particles.
    public Spring( Particle p1, Particle p2 ) {
        this.p1 = p1;
        this.p2 = p2;
        computeRestLength();
        p1.springs.add(this);
        p2.springs.add(this);
    }
    
    // Computes the rest length of the connected particles
    public void computeRestLength() {
        l0 = p1.p0.distance( p2.p0 );
    }
    
    // Sets the rest length of the connected particles with their current positions
    public void setRestLength() {
        l0 = p1.p.distance( p2.p );
    }
    
    // Applies spring forces to the two particles
    public void apply() {
        Vector3d force = new Vector3d();
        
        force.sub( p2.p, p1.p );
        double l = force.length();
        force.normalize();
        force.scale( (l-l0)*k );
        p1.addForce(force);
        force.scale(-1);
        p2.addForce(force);
        
        force.sub( p2.p, p1.p );
        force.normalize();
        Vector3d v = new Vector3d();
        v.sub(p2.v, p1.v);
        double rv = force.dot(v);
        force.scale( b * rv );
        p1.addForce(force);
        force.scale(-1);
        p2.addForce(force);            
    }    
}
