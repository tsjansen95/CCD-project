package project;

import java.util.LinkedList;
import java.util.List;

import javax.vecmath.Vector3d;

// Cable class
public class Cable {

	// A list of particles and springs that compose the cable
    public List<Particle> particles = new LinkedList<Particle>();
    
    public List<Spring> springs = new LinkedList<Spring>();
    
	// Rest length
    public double l0 = 0;
    
    // The number of segments that compose the cable
    public int segments;
    
    //Creates a cable connecting two particles, the number of segments is an argument
    public Cable( Particle p1, Particle p2, int s ) {
    	
    	// add the first particle
        particles.add(p1);
        p1.pinned = true;
        
        // calculate the direction between the two particles
        Vector3d dir = new Vector3d();
        dir.set(p2.p);
        dir.sub(p1.p);
        
        // set the rest length and the number of segments
        l0 = dir.length();
        segments = s;
        
        // temporary particle pointers
        Particle temp_p1 = p1;
        Particle temp_p2;
        
        // connect and create all the particles correctly
        for (int i = 1; i < segments; i++) {
        	temp_p2 = new Particle( p1.p.x + (double)i * (dir.x / (double)segments), p1.p.y + (double)i * (dir.y / (double)segments), 
        			p1.p.z + (double)i * (dir.z / (double)segments), 0.0, 0.0, 0.0 );
        	
        	particles.add(temp_p2);
        	springs.add( new Spring(temp_p1, temp_p2));
        	
        	temp_p1 = temp_p2;
        }
        
        // finally add the last particle
        particles.add(p2);
        p2.pinned = true;
        springs.add( new Spring(temp_p1, p2));
    }
}
