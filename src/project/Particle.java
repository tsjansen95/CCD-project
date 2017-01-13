package project;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

// Particle class
public class Particle {
    
    // True means that the particle can not move
    public boolean pinned = false;
        
    // The mass of the particle
    public double mass = 1;
    
    // Current position of the particle
    public Point3d p = new Point3d();
    
    // current velocity of the particle
    public Vector3d v = new Vector3d();
    
    // initial position of the particle
    public Point3d p0 = new Point3d();
    
    // initial velocity of the particle
    public Vector3d v0 = new Vector3d();
    
    // force acting on this particle
    public Vector3d f = new Vector3d();
    
	// A list of springs to which this particle is attached
    public ArrayList<Spring> springs = new ArrayList<Spring>();

    // Creates a particle with the given position and velocity
    public Particle( double x, double y, double z, double vx, double vy, double vz ) {
        p0.set(x,y,z);
        v0.set(vx,vy,vz);
        reset();
    }
    
    // Resets the position of this particle
    public void reset() {
        p.set(p0);
        v.set(v0);
        f.set(0,0,0);
    }
    
    // Adds the given force to this particle.
    public void addForce( Vector3d force ) {
        f.add(force);
    }
    
    // Computes the distance of a point to this particle
    public double distance( double x, double y, double z ) {
        Point3d tmp = new Point3d( x, y, z );
        return tmp.distance(p);
    } 
}
