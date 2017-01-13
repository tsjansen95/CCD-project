package project;

import java.util.LinkedList;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

public class RobustCCD {

    double restitution;
    
    double H;
    
    // Creates the new continuous collision detection and response object
    public RobustCCD() {
        // do nothing
    }
    
    // Try to deal with contacts before they happen
    public void applyRepulsion( double h, ParticleSystem system ) {
    	
    	// A set of variables to be used in the loop
    	double distance = 0;
    	double alpha, beta;
    	
    	Vector3d pa = new Vector3d();
    	Vector3d pb = new Vector3d();
    	Vector3d pc = new Vector3d();
    	Vector3d pd = new Vector3d();
    	
    	Vector3d dir1 = new Vector3d();
		Vector3d dir2 = new Vector3d();
		Vector3d normal = new Vector3d();
    	
    	Spring s1, s2;
    	
    	// Test every spring against every other spring
    	for (int i = 0; i < system.springs.size(); i++){
    		for (int j = 0; j < system.springs.size(); j++) {
    			
    			// If springs are the same forget it
    			if (j <= i) {	    				
    				continue;
    			}
    			
    			s1 = system.springs.get(i);
    			s2 = system.springs.get(j);
    			
    			// Make sure the springs don't share a particle
    			if (s1.p1.p.equals(s2.p1.p) || s1.p1.p.equals(s2.p2.p) || 
    					s1.p2.p.equals(s2.p1.p) || s1.p2.p.equals(s2.p2.p)) {
    				continue;
    			}
    			
    			// Find the directions of the springs
    			pa.set(s1.p1.p);
    			pb.set(s1.p2.p);
    			pc.set(s2.p1.p);
    			pd.set(s2.p2.p);
    			
    			dir1.set(pb);
    			dir1.sub(pa);
    			dir2.set(pd);
    			dir2.sub(pc);
    			
    			// Get the normal to the plane
    			normal.cross(dir2, dir1);
    			normal.normalize();
    			
    			// Find the distance between the springs
    			pc.sub(pa);
    			distance = pc.dot(normal);
    			pc.set(s2.p1.p);
    			  			
    			// Ignore cases that are too far
    			if (distance > H || distance < 0.0001) { continue; }
    			
    			pa.sub(pc);
    			
    			// Find the points where both springs intersect the normal
    			double a = dir1.dot(dir1);
    			double b = dir1.dot(dir2);
    			double c = dir2.dot(dir2);
    			double d = dir1.dot(pa);
    			double e = dir2.dot(pa);
    			
    			pa.set(s1.p1.p);
    			
    			// Set alpha and beta
    			if (Math.abs(a*c - b*b) < 0.0001) { continue; }
    			alpha = (b*e - c*d) / (a*c - b*b);
    			beta = (a*e - b*d) / (a*c - b*b);
    			
    			// Check epsilon intersections where springs are just not intersecting
    			double epsilon = 0.01;
    			if (alpha <= 0.0 && alpha > -epsilon) {
    				alpha = 0.001;
    			}
    			if (alpha >= 1.0 && alpha < 1.0 + epsilon) {
    				alpha = 0.999;
    			}
    			if (beta <= 0.0 && beta > -epsilon) {
    				beta = 0.001;
    			}
    			if (beta >= 1.0 && beta < 1.0 + epsilon) {
    				beta = 0.999;
    			}
    			
    			// Make sure the points of intersection lie within the springs
    			if (alpha <= 0 || alpha >= 1 || beta <= 0 || beta >= 1) { continue; }
    			
    			// Compute the relative velocity in the normal direction
    			pa.set(s1.p1.v);
	    		pa.scale(1 - alpha);
	    			
	    		pb.set(s1.p2.v);
	   			pb.scale(alpha);
	    			
	   			pc.set(s2.p1.v);
	   			pc.scale(1 - beta);
	   			
	   			pd.set(s2.p2.v);
	   			pd.scale(beta);
	   			
	   			pd.add(pc);
	   			pd.sub(pa);
	   			pd.sub(pb);
	    			
	    		double vrnm = normal.dot(pd);
    			
    			// If the particles are separating continue
    			if (vrnm >= 0) { continue; }
    			
    			// Get the impulse
    			double ma,mb,mc,md;
    			
    			if (s1.p1.pinned) { ma = Double.POSITIVE_INFINITY; }
	   			else { ma = s1.p1.mass; }
	    			
	   			if (s1.p2.pinned) { mb = Double.POSITIVE_INFINITY; }
	   			else { mb = s1.p2.mass; }
	    			
	   			if (s2.p1.pinned) { mc = Double.POSITIVE_INFINITY; }
	    		else { mc = s2.p1.mass; }
	   			
	   			if (s2.p2.pinned) { md = Double.POSITIVE_INFINITY; }
	   			else { md = s2.p2.mass; }
	    			
	    		double jimp = -Spring.k * (H - distance) * h;	    				
	    				
	    		// Apply the impulse to the particles
	    		pa.set(normal);
	    		pb.set(normal);
	   			pc.set(normal);
	   			pd.set(normal);
	    			
	   			pa.scale(-jimp * (1-alpha) / ma);
	   			pb.scale(-jimp * alpha / mb);
	   			pc.scale(jimp * (1 - beta) / mc);
	   			pd.scale(jimp * beta / md);
	    			
	    		pa.add(s1.p1.v);
	    		pb.add(s1.p2.v);
	    		pc.add(s2.p1.v);
	    		pd.add(s2.p2.v);
	    			
	   			s1.p1.v.set(pa);
	   			s1.p2.v.set(pb);
	   			s2.p1.v.set(pc);
	   			s2.p2.v.set(pd);
    			
    		}
    	}               
    }  
    
    // This function returns alpha given a particle, spring, and time t
    // This is for planar collisions i.e. springs that share a particle
    public double planar(Spring s1, Spring s2, double t){
    	
    	// Create all necessary new points and velocities
    	Point3d a = new Point3d();
		Point3d b = new Point3d();
		Point3d c = new Point3d();
		Vector3d av = new Vector3d();
		Vector3d bv = new Vector3d();
		Vector3d cv = new Vector3d();
		
		// Find spring directions
		Vector3d dir1 = new Vector3d();
		Vector3d dir2 = new Vector3d();
		
		// Initialize points
		a.set(s1.p1.p);
		b.set(s1.p2.p);
		
		av.set(s1.p1.v);
		av.scale(t);
		bv.set(s1.p2.v);
		bv.scale(t);
		
		// Find the particle that isn't shared by the first spring
		if (s2.p1.p.equals(a) || s2.p1.p.equals(b)) { 
			c.set(s2.p2.p);
			cv.set(s2.p2.v);
		}
		else { 
			c.set(s2.p1.p);
			cv.set(s2.p1.v);
		}
		cv.scale(t);
		c.add(cv);		
		a.add(av);
		b.add(bv);
		
		// Find the direction of the spring
		dir1.set(b);
		dir1.sub(a);
		dir1.normalize();
		
		// Find the direction from a to the particle
		dir2.set(c);
		dir2.sub(a);
		dir2.normalize();
		
		// If the springs are not nearly on top of each, forget about it
		if (Math.abs(dir1.x - dir2.x) > 0.0001) { return 2.0; }
		if (Math.abs(dir1.y - dir2.y) > 0.0001) { return 2.0; }
		if (Math.abs(dir1.z - dir2.z) > 0.0001) { return 2.0; }
		
		// Find where along the spring the collision occurs
		dir1.set(b);
		dir1.sub(a);

		double dist1;
		
		if (dir1.x != 0) { dist1 = (c.x - a.x) / dir1.x; }
		else if (dir1.y != 0) { dist1 = (c.y - a.y) / dir1.y; }
		else  { dist1 = (c.z - a.z) / dir1.z; }
		
		if (dist1 >= 1.0) { dist1 = 0.999; }
		return dist1;
    }
        
    // This function returns alpha for s1 given two springs, and time t
    public double between(Spring s1, Spring s2, double t){
    	
    	// Create all necessary new points and velocities
    	Point3d a = new Point3d();
		Point3d b = new Point3d();
		Point3d c = new Point3d();
		Point3d d = new Point3d();
		Vector3d av = new Vector3d();
		Vector3d bv = new Vector3d();
		Vector3d cv = new Vector3d();
		Vector3d dv = new Vector3d();
		
		// Initialize points
		a.set(s1.p1.p);
		b.set(s1.p2.p);
		c.set(s2.p1.p);
		d.set(s2.p2.p);
		
		// Initialize and scale velocities
		av.set(s1.p1.v);
		av.scale(t);
		bv.set(s1.p2.v);
		bv.scale(t);
		cv.set(s2.p1.v);
		cv.scale(t);
		dv.set(s2.p2.v);
		dv.scale(t);
		
		// Get location of points at collision
		a.add(av);
		b.add(bv);
		c.add(cv);
		d.add(dv);
		
		// Find spring directions
		Vector3d dir1 = new Vector3d();
		Vector3d dir2 = new Vector3d();
		
		dir1.set(b);
		dir1.sub(a);
		dir1.normalize();
		
		dir2.set(d);
		dir2.sub(c);
		dir2.normalize();
		
		// If the springs are parallel, forgat about it
		if (dir1.equals(dir2)) { return 2.0; }
			
		dir1.set(b);
		dir1.sub(a);
		
		dir2.set(d);
		dir2.sub(c);
		
		// Find the alpha and beta values
		double dist1, dist2;
		
		// Make sure to use values that don't return a NAN
		if (dir2.x != 0 && dir1.y != 0) {
			dist1 = (c.y - a.y + (a.x/dir2.x - c.x/dir2.x) * dir2.y) / (dir1.y - dir1.x * dir2.y / dir2.x);
			dist2 = (a.x + dist1*dir1.x - c.x) / dir2.x;
			if (Math.abs(a.z + dist1*dir1.z - c.z - dist2*dir2.z) > 1.0) { return 2.0; }
		}
		
		else if (dir2.x != 0 && dir1.z != 0) {
			dist1 = (c.z - a.z + (a.x/dir2.x - c.x/dir2.x) * dir2.z) / (dir1.z - dir1.x * dir2.z / dir2.x);
			dist2 = (a.x + dist1*dir1.x - c.x) / dir2.x;
			if (Math.abs(a.y + dist1*dir1.y - c.y - dist2*dir2.y) > 1.0) { return 2.0; }
		}
		
		else if (dir2.y != 0 && dir1.x != 0) {
			dist1 = (c.x - a.x + (a.y/dir2.y - c.y/dir2.y) * dir2.x) / (dir1.x - dir1.y * dir2.x / dir2.y);
			dist2 = (a.y + dist1*dir1.y - c.y) / dir2.y;
			if (Math.abs(a.z + dist1*dir1.z - c.z - dist2*dir2.z) > 1.0) { return 2.0; }
		}
		
		else if (dir2.y != 0 && dir1.z != 0) {
			dist1 = (c.z - a.z + (a.y/dir2.y - c.y/dir2.y) * dir2.z) / (dir1.z - dir1.y * dir2.z / dir2.y);
			dist2 = (a.y + dist1*dir1.y - c.y) / dir2.y;
			if (Math.abs(a.x + dist1*dir1.x - c.x - dist2*dir2.x) > 1.0) { return 2.0; }
		}
		
		else if (dir2.z != 0 && dir1.y != 0) {
			dist1 = (c.y - a.y + (a.z/dir2.z - c.z/dir2.z) * dir2.y) / (dir1.y - dir1.z * dir2.y / dir2.z);
			dist2 = (a.z + dist1*dir1.z - c.z) / dir2.z;
			if (Math.abs(a.x + dist1*dir1.x - c.x - dist2*dir2.x) > 1.0) { return 2.0; }
		}
		
		else {
			dist1 = (c.x - a.x + (a.z/dir2.z - c.z/dir2.z) * dir2.x) / (dir1.x - dir1.z * dir2.x / dir2.z);
			dist2 = (a.z + dist1*dir1.z - c.x) / dir2.z;
			if (Math.abs(a.y + dist1*dir1.y - c.y - dist2*dir2.y) > 1.0) { return 2.0; }
		}
	
		// Return the alpha value
		return dist1;
		
    }
    
    // Checks all collisions in interval t to t+h
    public boolean check( double h, ParticleSystem system ) {        
        
    	// Variables to end the loop
    	boolean collision = false;
    	int n = 0;
    	
    	// Variables to be used inside the loop
    	double t = h + 0.001;
    	boolean threeD = true;
    	Spring x = null;
    	Spring y = null;
    	
    	double alpha = 0;
    	double beta = 0;
    	
    	double a, b, c, d;
    	
    	Vector3d A = new Vector3d();
    	Vector3d B = new Vector3d();
    	Vector3d C = new Vector3d();
    	Vector3d D = new Vector3d();
    	
    	Spring s1, s2;
    	
    	do {    		
    		// Reset variables
    		collision = false;
    		t = h;
    		
    		// Test each sprint against each other spring
	    	for (int i = 0; i < system.springs.size(); i++){
	    		for (int j = 0; j < system.springs.size(); j++) {
	    			
	    			// If springs are the same forget it
	    			if (j <= i) {	    				
	    				continue;
	    			}
	    			
	    			s1 = system.springs.get(i);
	    			s2 = system.springs.get(j);
	    			
	    			// If springs share a particle, check for planar collisions
	    			if (s1.p1.p.equals(s2.p1.p) || s1.p1.p.equals(s2.p2.p) || 
	    					s1.p2.p.equals(s2.p1.p) || s1.p2.p.equals(s2.p2.p)) {		    				
	    				
	    				// Find the particle that isn't shared
	    				Particle T = s1.p1;
	    				Particle S = s1.p2;
	    				Particle J;
	    				if (T.p.equals(s2.p1.p) || S.p.equals(s2.p1.p)) { J = s2.p2; }
	    				else { J = s2.p1; }
		    			
		    			// Get a,b,c for the quadratic function for t
		    			a = J.v.y*T.v.x - S.v.y*T.v.x + S.v.y*J.v.x
		    					- T.v.y*J.v.x + T.v.y*S.v.x - J.v.y*S.v.x;
		    				
		    			b = J.p.y*T.v.x + T.p.x*J.v.y - S.p.y*T.v.x
		   						- T.p.x*S.v.y + S.p.y*J.v.x + J.p.x*S.v.y
		   						- T.p.y*J.v.x - J.p.x*T.v.y + T.p.y*S.v.x
		    					+ S.p.x*T.v.y - J.p.y*S.v.x - S.p.x*J.v.y;
		    				
		    			c = J.p.y*T.p.x - S.p.y*T.p.x + S.p.y*J.p.x
		    					- T.p.y*J.p.x + T.p.y*S.p.x - J.p.y*S.p.x;
		    			
		    			if (Math.abs(a) < 0.0001) { a = 0.0; }
		    			if (Math.abs(b) < 0.0001) { b = 0.0; }
		    			if (Math.abs(c) < 0.0001) { c = 0.0; }
		    				
		    			// Find the determinant	
		    			double sqr = Math.pow(b, 2) - (4*a*c);
		    			
		    			// If a is zero, do not compute the quadratic function
		    			if (a == 0) {
		    				
		    				// If t is small enough, and the point goes between a & b, set all values
		    				if (((-c/b) >= 0) && ((-c/b) < t)){
		    					
		    					// Check for an intersection
		    					double ta = planar(s1, s2, (-c/b));
		    					double tb = planar(s2, s1, (-c/b));
		    					
		    					// If it is valid, set all the values
		    					if (ta > 0 && ta < 1 && tb > 0 && tb < 1) {
		    						alpha = ta;
			    					beta = tb;
		    						t = (-c/b);
		    						x = s1;
		    						y = s2;
		    						
		    						// Take the point that is between the springs
		    						if (tb < ta) {
		    							alpha = tb;
		    							y = s1;
		    							x = s2;
		    						}
		    						
		    						collision = true;
		    						threeD = false;
		   						}
		    				}
		    			}
		    			
		    			// Now if a is not equal to 0
		    			else if (sqr >= 0){
		    				
		    				// Find the two values for t
		    				double t1 = (-b + Math.sqrt(sqr))/(2*a);
		    				double t2 = (-b - Math.sqrt(sqr))/(2*a);
		    					
		    				// Check the first option
		    				if ((t1 >= 0) && (t1 < t)) {
		    						
		    					double ta = planar(s1, s2, t1);
		    					double tb = planar(s2, s1, t1);
		    					
		    					// If it is valid, set all the values
		    					if (ta > 0 && ta < 1 && tb > 0 && tb < 1) {
		    						alpha = ta;
			    					beta = tb;
		    						t = t1;
		    						x = s1;
		    						y = s2;
		    						
		    						if (tb < ta) {
		    							alpha = tb;
		    							y = s1;
		    							x = s2;
		    						}
		    						
		    						collision = true;
		    						threeD = false;
		   						}
		    				}
		    					
		    				// Check the second option
		    				if ((t2 >= 0) && (t2 < t)) {
		    					
		    					double ta = planar(s1, s2, t2);
		    					double tb = planar(s2, s1, t2);
		    					
		    					// If it is valid, set all the values
		    					if (ta > 0 && ta < 1 && tb > 0 && tb < 1) {
		    						alpha = ta;
			    					beta = tb;
		    						t = t2;
		    						x = s1;
		    						y = s2;
		    						
		    						if (tb < ta) {
		    							alpha = tb;
		    							y = s1;
		    							x = s2;
		    						}
		    						
		    						collision = true;
		    						threeD = false;
		   						}		    					
		    				}
		    			}
	    				
	    				continue;
	    			}
	    			
	    			
	    			// Get a,b,c,d for the cubic function for t	    			
	    			A.set(s1.p1.p);
	    			B.set(s1.p2.p);
	    			C.set(s2.p1.p);
	    			D.set(s2.p2.p);
	    			
	    			d = D.x*B.y*C.z + D.x*A.y*B.z + D.x*C.y*A.z + C.x*D.y*B.z + B.x*D.y*A.z + A.x*D.y*C.z +
	    					B.x*C.y*D.z + A.x*B.y*D.z + C.x*A.y*D.z + A.x*C.y*B.z + B.x*A.y*C.z + C.x*B.y*A.z -
	    					D.x*B.y*A.z - D.x*A.y*C.z - D.x*C.y*B.z - A.x*D.y*B.z - C.x*D.y*A.z - B.x*D.y*C.z -
	    					B.x*A.y*D.z - A.x*C.y*D.z - C.x*B.y*D.z - A.x*B.y*C.z - C.x*A.y*B.z - B.x*C.y*A.z;
	    			
	    			// Find b
	    			A.set(s1.p1.p);
	    			B.set(s1.p2.p);
	    			C.set(s2.p1.p);
	    			D.set(s2.p2.v);
	    			
	    			c = D.x*B.y*C.z + D.x*A.y*B.z + D.x*C.y*A.z + C.x*D.y*B.z + B.x*D.y*A.z + A.x*D.y*C.z +
	    					B.x*C.y*D.z + A.x*B.y*D.z + C.x*A.y*D.z - D.x*B.y*A.z - D.x*A.y*C.z - D.x*C.y*B.z - 
	    					A.x*D.y*B.z - C.x*D.y*A.z - B.x*D.y*C.z - B.x*A.y*D.z - A.x*C.y*D.z* - C.x*B.y*D.z;
	    			
	    			A.set(s1.p1.p);
	    			B.set(s1.p2.p);
	    			C.set(s2.p1.v);
	    			D.set(s2.p2.p);
	    			
	    			c += D.x*B.y*C.z + D.x*C.y*A.z + C.x*D.y*B.z + A.x*D.y*C.z +B.x*C.y*D.z + C.x*A.y*D.z + 
	    					A.x*C.y*B.z + B.x*A.y*C.z + C.x*B.y*A.z - D.x*A.y*C.z - D.x*C.y*B.z - C.x*D.y*A.z - 
	    					B.x*D.y*C.z - A.x*C.y*D.z - C.x*B.y*D.z - A.x*B.y*C.z - C.x*A.y*B.z - B.x*C.y*A.z;
	    			
	    			A.set(s1.p1.p);
	    			B.set(s1.p2.v);
	    			C.set(s2.p1.p);
	    			D.set(s2.p2.p);
	    			
	    			c += D.x*B.y*C.z + D.x*A.y*B.z + C.x*D.y*B.z + B.x*D.y*A.z + B.x*C.y*D.z + A.x*B.y*D.z + 
	    					A.x*C.y*B.z + B.x*A.y*C.z + C.x*B.y*A.z - D.x*B.y*A.z - D.x*C.y*B.z - A.x*D.y*B.z - 
	    					B.x*D.y*C.z - B.x*A.y*D.z - C.x*B.y*D.z - A.x*B.y*C.z - C.x*A.y*B.z - B.x*C.y*A.z;
	    			
	    			A.set(s1.p1.v);
	    			B.set(s1.p2.p);
	    			C.set(s2.p1.p);
	    			D.set(s2.p2.p);
	    			
	    			c += D.x*A.y*B.z + D.x*C.y*A.z + B.x*D.y*A.z + A.x*D.y*C.z + A.x*B.y*D.z + C.x*A.y*D.z + 
	    					A.x*C.y*B.z + B.x*A.y*C.z + C.x*B.y*A.z - D.x*B.y*A.z - D.x*A.y*C.z - A.x*D.y*B.z - 
	    					C.x*D.y*A.z - B.x*A.y*D.z - A.x*C.y*D.z - A.x*B.y*C.z - C.x*A.y*B.z - B.x*C.y*A.z;
	    			
	    			// Find d
	    			A.set(s1.p1.v);
	    			B.set(s1.p2.v);
	    			C.set(s2.p1.v);
	    			D.set(s2.p2.p);
	    			
	    			b = D.x*B.y*C.z + D.x*A.y*B.z + D.x*C.y*A.z + C.x*D.y*B.z + B.x*D.y*A.z + A.x*D.y*C.z +
	    					B.x*C.y*D.z + A.x*B.y*D.z + C.x*A.y*D.z - D.x*B.y*A.z - D.x*A.y*C.z - D.x*C.y*B.z - 
	    					A.x*D.y*B.z - C.x*D.y*A.z - B.x*D.y*C.z - B.x*A.y*D.z - A.x*C.y*D.z* - C.x*B.y*D.z;
	    			
	    			A.set(s1.p1.v);
	    			B.set(s1.p2.v);
	    			C.set(s2.p1.p);
	    			D.set(s2.p2.v);
	    			
	    			b += D.x*B.y*C.z + D.x*C.y*A.z + C.x*D.y*B.z + A.x*D.y*C.z +B.x*C.y*D.z + C.x*A.y*D.z + 
	    					A.x*C.y*B.z + B.x*A.y*C.z + C.x*B.y*A.z - D.x*A.y*C.z - D.x*C.y*B.z - C.x*D.y*A.z - 
	    					B.x*D.y*C.z - A.x*C.y*D.z - C.x*B.y*D.z - A.x*B.y*C.z - C.x*A.y*B.z - B.x*C.y*A.z;
	    			
	    			A.set(s1.p1.v);
	    			B.set(s1.p2.p);
	    			C.set(s2.p1.v);
	    			D.set(s2.p2.v);
	    			
	    			b += D.x*B.y*C.z + D.x*A.y*B.z + C.x*D.y*B.z + B.x*D.y*A.z + B.x*C.y*D.z + A.x*B.y*D.z + 
	    					A.x*C.y*B.z + B.x*A.y*C.z + C.x*B.y*A.z - D.x*B.y*A.z - D.x*C.y*B.z - A.x*D.y*B.z - 
	    					B.x*D.y*C.z - B.x*A.y*D.z - C.x*B.y*D.z - A.x*B.y*C.z - C.x*A.y*B.z - B.x*C.y*A.z;
	    			
	    			A.set(s1.p1.p);
	    			B.set(s1.p2.v);
	    			C.set(s2.p1.v);
	    			D.set(s2.p2.v);
	    			
	    			b += D.x*A.y*B.z + D.x*C.y*A.z + B.x*D.y*A.z + A.x*D.y*C.z + A.x*B.y*D.z + C.x*A.y*D.z + 
	    					A.x*C.y*B.z + B.x*A.y*C.z + C.x*B.y*A.z - D.x*B.y*A.z - D.x*A.y*C.z - A.x*D.y*B.z - 
	    					C.x*D.y*A.z - B.x*A.y*D.z - A.x*C.y*D.z - A.x*B.y*C.z - C.x*A.y*B.z - B.x*C.y*A.z;
	    			
	    			// Find d
	    			A.set(s1.p1.v);
	    			B.set(s1.p2.v);
	    			C.set(s2.p1.v);
	    			D.set(s2.p2.v);
	    			
	    			a = D.x*B.y*C.z + D.x*A.y*B.z + D.x*C.y*A.z + C.x*D.y*B.z + B.x*D.y*A.z + A.x*D.y*C.z +
	    					B.x*C.y*D.z + A.x*B.y*D.z + C.x*A.y*D.z + A.x*C.y*B.z + B.x*A.y*C.z + C.x*B.y*A.z -
	    					D.x*B.y*A.z - D.x*A.y*C.z - D.x*C.y*B.z - A.x*D.y*B.z - C.x*D.y*A.z - B.x*D.y*C.z -
	    					B.x*A.y*D.z - A.x*C.y*D.z - C.x*B.y*D.z - A.x*B.y*C.z - C.x*A.y*B.z - B.x*C.y*A.z;
	    			
	    			if (Math.abs(a) < 0.0001) { a = 0.0; }
	    			if (Math.abs(b) < 0.0001) { b = 0.0; }
	    			if (Math.abs(c) < 0.0001) { c = 0.0; }
	    			if (Math.abs(d) < 0.0001) { d = 0.0; }
	    			
	    			// If a is zero, compute the quadratic function
	    			if (a == 0) {
	    				
		    			// Find the determinant	
		    			double sqr = Math.pow(c, 2) - (4.0*b*d);
		    			
		    			if (b == 0) {
		    				
		    				// If it is valid, set all the values
		    				if (((-d/c) >= 0) && ((-d/c) < t)){	
		    					
		    					double ta = between(s1, s2, (-d/c));
		    					double tb = between(s2, s1, (-d/c));
		    					
		    					if (ta > 0 && ta < 1 && tb > 0 && tb < 1) {
		    						alpha = ta;
			    					beta = tb;
		    						t = (-d/c);
		    						x = s1;
		    						y = s2;
		    						collision = true;
		    						threeD = true;
		   						}
		    				}	    				
		    			}
	    			
		    			// Now if b is not equal to 0
		    			else if (sqr >= 0){
		    				
		    				// Find the two values for t
		    				double t1 = (-c + Math.sqrt(sqr))/(2.0*b);
		    				double t2 = (-c - Math.sqrt(sqr))/(2.0*b);
		    					
		    				// Check the first option
		    				if ((t1 >= 0) && (t1 < t)) {
		    							    					
		    					double ta = between(s1, s2, t1);
		    					double tb = between(s2, s1, t1);
		    					
		    					// If it is valid, set all the values
		    					if (ta > 0 && ta < 1 && tb > 0 && tb < 1) {
		    						alpha = ta;
			    					beta = tb;
		    						t = t1;
		    						x = s1;
		    						y = s2;
		    						collision = true;
		    						threeD = true;
		   						}
		    				}
		    					
		    				// Check the second option
		    				if ((t2 >= 0) && (t2 < t)) {
		    					
		    					double ta = between(s1, s2, t2);
		    					double tb = between(s2, s1, t2);
		    					
		    					// If it is valid, set all the values
		    					if (ta > 0 && ta < 1 && tb > 0 && tb < 1) {
		    						alpha = ta;
			    					beta = tb;
		    						t = t2;
		    						x = s1;
		    						y = s2;
		    						collision = true;
		    						threeD = true;
		   						}
		    				}	
		    			}	    			
	    			}
	    			
	    			// If a != 0, compute the cubic formula
	    			else {
	    				
	    				b = b/a;
	    				c = c/a;
	    				d = d/a;
	    				a = 1.0;
	    				
	    				double p = (3.0*c - b*b) / 3.0;
	    				double q = (9.0*c*b - 27.0*d - 2.0*b*b*b) / 27.0;
	    				
	    				double R = q/2.0;
	    				double Q = p/3.0;
	    				
	    				double sqr1 = R*R + Q*Q*Q;
	    				LinkedList<Double> cub = new LinkedList<Double>();	    				
	    				     		
	    				// If the sqrt is positive, find the cube roots
	    				if (sqr1 >= 0) {
	    					
	    					sqr1 = Math.sqrt(sqr1);
	    					double S = Math.cbrt(R + sqr1);
	    					double T = Math.cbrt(R- sqr1);
	    					
	    					cub.add(-b/3.0 + S + T);
	    					if (Math.abs(S-T) < 0.0001) { cub.add(-b/3.0 - S/2.0 - T/2.0); }
	    						    					    					
	    				}
	    				
	    				// Otherwise compute the cube roots for the complex number in the sqrt
	    				else {
	    					sqr1 = -sqr1;
	    					sqr1 = Math.sqrt(sqr1);
	    					
	    					// Get the new r and theta values
	    					double r = Math.sqrt(R*R + sqr1*sqr1);
	    					double St = Math.asin(sqr1/r)/3.0;
	    					double Tt = Math.asin(-sqr1/r)/3.0;
	    					r = Math.pow(r, 1.0/3.0);
	    					
	    					// Plug them back into the a + bi form
	    					double Sa = r * Math.cos(St);
	    					double Sb = r * Math.sin(St);
	    					double Ta = r * Math.cos(Tt);
	    					double Tb = r * Math.sin(Tt);
	    					
	    					double Ba = Sa + Ta;
	    					double Bb = Sb + Tb;
	    					double Aa = Sa - Ta;
	    					double Ab = Sb - Tb;
	    					
	    					if (Math.abs(Ba) < 0.0001) { Ba = 0.0; }
	    					if (Math.abs(Bb) < 0.0001) { Bb = 0.0; }
	    					if (Math.abs(Aa) < 0.0001) { Aa = 0.0; }
	    					if (Math.abs(Ab) < 0.0001) { Ab = 0.0; }
	    					
	    					// Plug the values into the time equations
	    					if (Bb == 0.0) { cub.add(-b/3.0 + Ba); }
	    					if (Bb == 0.0 && Aa == 0.0) {
	    						cub.add(-b/3.0 - Ba/2.0 - Ab*Math.sqrt(3.0)/2.0);
	    						cub.add(-b/3.0 - Ba/2.0 + Ab*Math.sqrt(3.0)/2.0);
	    					}					
	    				}
	    				
	    				// Find the smallest value in the cube roots
	    				for (int k = 0; i < k; k++){
	    					if (cub.size() <= k) { break; }
	    					else if (cub.get(k) < 0) { cub.remove(k); }
    					}
    					
    					if (cub.isEmpty()) { continue; }
    					
    					double tlow = cub.getFirst();
    					for (double val : cub) {
    						if (val < tlow) { tlow = val; }
    					}
    					
    					// If the smallest time is the nearest collision
	    				if ((tlow >= 0) && (tlow < t)) {
	    					
	    					double ta = between(s1, s2, tlow);
	    					double tb = between(s2, s1, tlow);
	    					
	    					// If it is valid, set all the values
	    					if (ta > 0 && ta < 1 && tb > 0 && tb < 1) {
	    						alpha = ta;
		    					beta = tb;
	    						t = tlow;
	    						x = s1;
	    						y = s2;
	    						collision = true;
	    						threeD = true;
	   						}
	    				}
	    			}	    			
	    		}
	    	}
	    	
	    	// If there is no collision, continue
	    	if (!collision) { continue; }
	    		
	    	// A null check
	    	if (!(y == null) && !(x == null)) {
	    		
	    		// Create all necessary new points and velocities
	        	Point3d nA = new Point3d();
	    		Point3d nB = new Point3d();
	    		Point3d nC = new Point3d();
	    		Point3d nD = new Point3d();
	    		Vector3d av = new Vector3d();
	    		Vector3d bv = new Vector3d();
	    		Vector3d cv = new Vector3d();
	    		Vector3d dv = new Vector3d();
	    		
	    		Vector3d dir1 = new Vector3d();
	    		Vector3d dir2 = new Vector3d();
	    		Vector3d normal = new Vector3d();
	    		
	    		Vector3d pa = new Vector3d();
	   			Vector3d pb = new Vector3d();
	   			Vector3d pc = new Vector3d();
	   			Vector3d pd = new Vector3d();
	   			
	    		double ma,mb,mc,md;
	    		
	    		// If the collision is planar
	    		if (!threeD){
	    			   
	    			// Set the positions for time of collision
		    		nA.set(x.p1.p);
		   			nB.set(x.p2.p);
		   			av.set(x.p1.v);
		   			av.scale(t);
		   			bv.set(x.p2.v);
		   			bv.scale(t);
		   			
		   			// Choose the particle that isn't shared as the colliding particle
		   			if (y.p1.p.equals(nA) || y.p1.p.equals(nB)) {
		   				nC.set(y.p2.p);
		   				cv.set(y.p2.v);
		   				pc.set(y.p2.v);
			   			if (y.p2.pinned) { mc = Double.POSITIVE_INFINITY; }
			    		else { mc = y.p2.mass; }
		   			}
		   			else {
		   				nC.set(y.p1.p);
		   				cv.set(y.p1.v);
		   				pc.set(y.p1.v);
		   				if (y.p1.pinned) { mc = Double.POSITIVE_INFINITY; }
			    		else { mc = y.p1.mass; }
		   			}
		   			cv.scale(t);
		   			
		   			// Find the direction of the spring
		   			dir1.set(nB);
		    		dir1.sub(nA);

		    		// Find the direction from A to the particle
		    		dir2.set(nC);
		    		dir2.sub(nA);
		   			
		   			nA.add(av);
		   			nB.add(bv);
		   			nC.add(cv);
		    		
		   			// Find the normal of the collision
		    		normal.cross(dir1, dir2);
		    		normal.cross(normal, dir1);
		    		normal.normalize();
		    		
		    		// Find the relative velocity in the normal direction
		    		pa.set(x.p1.v);
		    		pa.scale(1 - alpha);
		    			
		    		pb.set(x.p2.v);
		   			pb.scale(alpha);
		   				
		   			pc.sub(pa);
		   			pc.sub(pb);
		    			
		    		double vrnm = normal.dot(pc);
		    			
		    		// Calculate the impulse		    			
		    		if (y.p1.pinned) { ma = Double.POSITIVE_INFINITY; }
		   			else { ma = y.p1.mass; }
		    			
		   			if (y.p2.pinned) { mb = Double.POSITIVE_INFINITY; }
		   			else { mb = y.p2.mass; }
		    			
		    		double j = (-(1 - restitution) * vrnm) / 
		    					( (1/mc) + (Math.pow(1 - alpha, 2)/ma) + (Math.pow(alpha, 2)/mb) );
		    			
		    		// Apply the impulse to the particles
		    		pa.set(normal);
		    		pb.set(normal);
		   			pc.set(normal);
		    			
		   			pa.scale(-j * (1-alpha) / ma);
		   			pb.scale(-j * alpha / mb);
		   			pc.scale(j / mc);
		   				   			
		   			if (y.p1.p.equals(x.p1.p) || y.p1.p.equals(x.p2.p)) {
		   				pc.add(y.p2.v);
			   			y.p2.v.set(pc);
		   			}
		   			else {
		   				pc.add(y.p1.v);
			   			y.p1.v.set(pc);
		   			}
		   			
		   			pa.add(x.p1.v);
		    		pb.add(x.p2.v);
		    			
		   			x.p1.v.set(pa);
		   			x.p2.v.set(pb);		   			
	    		}
	    		
	    		// Otherwise compute the impulse for two springs
	    		else {
		    		
		    		// Initialize points
		    		nA.set(x.p1.p);
		    		nB.set(x.p2.p);
		    		nC.set(y.p1.p);
		    		nD.set(y.p2.p);
		    		
		    		// Initialize and scale velocities
		    		av.set(x.p1.v);
		    		av.scale(t);
		    		bv.set(x.p2.v);
		    		bv.scale(t);
		    		cv.set(y.p1.v);
		    		cv.scale(t);
		    		dv.set(y.p2.v);
		    		dv.scale(t);
		    		
		    		// Get location of points at collision
		    		nA.add(av);
		    		nB.add(bv);
		    		nC.add(cv);
		    		nD.add(dv);	    		
		    			
		    		// Get the axis along the spring		    		
		    		dir1.set(nB);
		    		dir1.sub(nA);
	
		    		dir2.set(nD);
		    		dir2.sub(nC);
		    		
		   			// Find the normal
		    		normal.cross(dir1, dir2);
		    		normal.normalize();
		    		
		    		// Find the relative velocity in the normal direction
		    		pa.set(x.p1.v);
		    		pa.scale(1 - alpha);
		    			
		    		pb.set(x.p2.v);
		   			pb.scale(alpha);
		    			
		   			pc.set(y.p1.v);
		   			pc.scale(1 - beta);
		   			
		   			pd.set(y.p2.v);
		   			pd.scale(beta);
		   			
		   			pd.add(pc);
		   			pd.sub(pa);
		   			pd.sub(pb);
		    			
		    		double vrnm = normal.dot(pd);
		    			
		    		// Calculate the impulse
		    			
		    		if (x.p1.pinned) { ma = Double.POSITIVE_INFINITY; }
		   			else { ma = x.p1.mass; }
		    			
		   			if (x.p2.pinned) { mb = Double.POSITIVE_INFINITY; }
		   			else { mb = x.p2.mass; }
		    			
		   			if (y.p1.pinned) { mc = Double.POSITIVE_INFINITY; }
		    		else { mc = y.p1.mass; }
		   			
		   			if (y.p2.pinned) { md = Double.POSITIVE_INFINITY; }
		   			else { md = y.p2.mass; }
		    			
		    		double j = (-(1 - restitution) * vrnm) / 
		    					( (Math.pow(1 - beta, 2)/mc) + (Math.pow(beta, 2)/md) + (Math.pow(1 - alpha, 2)/ma) + (Math.pow(alpha, 2)/mb) );
		    			
		    		// Apply the impulse to the particles
		    		pa.set(normal);
		    		pb.set(normal);
		   			pc.set(normal);
		   			pd.set(normal);
		    			
		   			pa.scale(-j * (1-alpha) / ma);
		   			pb.scale(-j * alpha / mb);
		   			pc.scale(j * (1 - beta) / mc);
		   			pd.scale(j * beta / md);
		    			
		    		pa.add(x.p1.v);
		    		pb.add(x.p2.v);
		    		pc.add(y.p1.v);
		    		pd.add(y.p2.v);
		    			
		   			x.p1.v.set(pa);
		   			x.p2.v.set(pb);
		   			y.p1.v.set(pc);
		   			y.p2.v.set(pd);
	    		}
	    	}
   			n++;	
   			
	    // Continue while there is a collision or stop if there are too many
    	} while (collision && (n <= 20));
    	
    	// Return true if all collisions have been properly accounted for
        if (n == 20) { return false; }
        return true;
    }       
}
