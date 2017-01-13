package project;

import java.util.LinkedList;
import java.util.List;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;
import com.jogamp.opengl.util.gl2.GLUT;

import javax.swing.JPanel;
import javax.swing.border.TitledBorder;
import javax.vecmath.Vector3d;

import project.Particle;

import mintools.parameters.BooleanParameter;
import mintools.parameters.DoubleParameter;
import mintools.swing.CollapsiblePanel;
import mintools.swing.VerticalFlowPanel;
import mintools.viewer.SceneGraphNode;

// Implementation of a simple particle system.
public class ParticleSystem implements SceneGraphNode {
    
    // the particle list
    public List<Particle> particles = new LinkedList<Particle>();
    
    // the spring list (treat this as the edge list for geometry
    public List<Spring> springs = new LinkedList<Spring>();
    
    // a list of cables, and a list of points composing a cable
    public List<Cable> cables = new LinkedList<Cable>();
    public List<Particle> cable_pts = new LinkedList<Particle>();
    
    public String name = "";
    
    public ParticleSystem() {
        // creates an empty system!
    }
    
    // Resets the positions of all particles to their initial states
    public void resetParticles() {
        for ( Particle p : particles ) {
            p.reset();
        }
        for (Particle p : cable_pts) {
        	p.reset();
        }
        time = 0;
    }

    // Deletes all particles, and as such removes all springs too.
    public void clear() {        
        particles.clear();
        springs.clear();
        cables.clear();
        cable_pts.clear();
        name = "";
    }    
    
    public double time = 0;
    
    private RobustCCD robustCCD = new RobustCCD();

    // Advances time and updates the position of all particles
    public boolean updateParticles( double h ) {
        boolean resultOK = true;
        // set the global spring properties
        Spring.k = k.getValue();
        Spring.b = b.getValue();
                
        // first compute all forces
        Vector3d tmp = new Vector3d();
        
        double damping  = c.getValue();
        for ( Particle p : particles ) {
            p.f.set( 0, useg.getValue() ? -g.getValue() : 0, 0 );
            tmp.scale( -damping, p.v );
            p.f.add( tmp );                        
        }       
        for ( Particle p : cable_pts ) {
            p.f.set( 0, useg.getValue() ? -g.getValue() : 0, 0 );
            tmp.scale( -damping, p.v );
            p.f.add( tmp );                        
        }
        
        for ( Spring s : springs ) {
            s.apply();
        }
        
        // Update the velocity of the particles as per symplectic Euler
        for ( Particle p : particles ) {
            if ( p.pinned ) {            
                p.f.set(0,0,0); // just to make sure!
                p.v.set(0,0,0);
            } else {
                tmp.scale( h / p.mass, p.f );
                p.v.add( tmp );            
            }
        }
        
        // Update the velocity of the particles as per symplectic Euler
        for ( Particle p : cable_pts ) {
            if ( p.pinned ) {            
                p.f.set(0,0,0); // just to make sure!
                p.v.set(0,0,0);
            } else {
                tmp.scale( h / p.mass, p.f );
                p.v.add( tmp );            
            }
        }
        
        // perform robust collision detection here.        
        robustCCD.restitution = restitution.getValue();
        robustCCD.H = H.getValue();
        if ( repulsion.getValue() ) {
            robustCCD.applyRepulsion( h, this );            
        }
        if ( collision.getValue() ) {
            if ( ! robustCCD.check( h, this ) ) {
                resultOK = false;
            }
        }
        
        // Finally, update the positions using the velocity at the next time step
        for ( Particle p : particles ) {
            if ( p.pinned ) continue;
            // symplectic Euler 
            tmp.scale( h, p.v );
            p.p.add( tmp );
            p.f.set(0,0,0);
        }       
        for ( Particle p : cable_pts ) {
            if ( p.pinned ) continue;
            tmp.scale( h, p.v );
            p.p.add( tmp );
            p.f.set(0,0,0);
        }
                        
        time = time + h;
        return resultOK;
    }

    // Creates a new particle and adds it to the system
    public Particle createParticle( double x, double y, double z, double vx, double vy, double vz ) {
        Particle p = new Particle( x, y, z, vx, vy, vz );
        particles.add( p );
        return p;
    }
    
    // Creates a new spring between two particles and adds it to the system.
    public Spring createSpring( Particle p1, Particle p2 ) {
        Spring s = new Spring( p1, p2 ); 
        springs.add( s );         
        return s;
    }
    
    // Creates a new cable and adds it to the system
    public Cable createCable( Particle p1, Particle p2, int segments) {
    	Cable c = new Cable( p1, p2, segments);
    	cables.add( c );
    	for (int i = 1; i < c.particles.size()-1; i++) {
    		cable_pts.add(c.particles.get(i));
    	}
    	for (int i = 0; i < c.springs.size(); i++) {
    		springs.add(c.springs.get(i));
    	}
    	if (!particles.contains(c.particles.get(0))) { particles.add(c.particles.get(0)); }
    	if (!particles.contains(c.particles.get(c.particles.size()-1))) { particles.add(c.particles.get(c.particles.size()-1)); }
    	
    	return c;
    }
    
    @Override
    public void init(GLAutoDrawable drawable) {
        // do nothing
    }

    // Height of the canvas, useful for wall collisions
    public int height;
    
    // Width of the canvas, useful for wall collisions
    public int width;
    
    // some variables to determine the shape of the objects
    GLUT glut = new GLUT();
    float radius;
    float cyl;
    
    // some colors
    float alpha = 1.0f;
    float[] red = {1, 0, 0, alpha};
    float[] green = {0, 0.95f, 0, alpha};
	float[] grey = {0.75f,0.75f,0.75f, alpha};

    
    @Override
    public void display(GLAutoDrawable drawable) {
        GL2 gl = drawable.getGL().getGL2();

        // We'll keep track of the width and the height 
        // of the drawable as this might be useful for 
        // processing collisions with walls
        height = drawable.getSurfaceHeight();
        width = drawable.getSurfaceWidth();
        cyl = cylinderRadius.getFloatValue();
        
        if ( drawParticles.getValue() ) {
            radius = sphereSize.getFloatValue();
            for ( Particle p : particles ) {
                // transparency is used to get smooth edges on the particles
                if ( p.pinned ) {
                    gl.glMaterialfv( GL.GL_FRONT_AND_BACK, GL2.GL_AMBIENT_AND_DIFFUSE, red, 0 );
                } else {
                    gl.glMaterialfv( GL.GL_FRONT_AND_BACK, GL2.GL_AMBIENT_AND_DIFFUSE, green, 0 );
                }
                gl.glPushMatrix();
                gl.glTranslated(p.p.x, p.p.y, p.p.z);
                glut.glutSolidSphere(radius, 20, 20);
                gl.glPopMatrix();
            }
        }
        
        // draw the lines (represented by cylinders)
        gl.glMaterialfv( GL.GL_FRONT_AND_BACK, GL2.GL_AMBIENT_AND_DIFFUSE, grey, 0 );
        for (Spring s : springs) {
        	gl.glPushMatrix();
        	
        	// translate to the correct location
        	gl.glTranslated(s.p1.p.x, s.p1.p.y, s.p1.p.z);
        	
        	// get the direction from one point to the next
        	Vector3d dist = new Vector3d();
        	dist.set(s.p2.p);
        	dist.sub(s.p1.p);
        	
        	// cylinders are drawn along the the z-axis
        	Vector3d axis = new Vector3d();
        	axis.set(0,0,1);
        	
        	// calculate the angle by which the cylinder needs to rotate
        	double angle = Math.acos(axis.dot(dist) / dist.length());
        	angle = angle * 180 / Math.PI;
        	
        	// find the axis of rotation
        	axis.cross(axis, dist);
        	
        	// rotate
        	if (axis.length() != 0.0) { 
        		axis.normalize();
        		gl.glRotated(angle, axis.x, axis.y, axis.z);
        	}
        	
        	// if the axis and direction align, rotate along arbitrary axis
        	else if (angle != 0.0) {
        		gl.glRotated(angle, 1.0, 0.0, 0.0);
        	}

            glut.glutSolidCylinder(cyl, dist.length(), 20, 20);
            gl.glPopMatrix();
        }
    }    
    
    private BooleanParameter drawParticles = new BooleanParameter( "draw Particles", true ) ;
    
    private DoubleParameter sphereSize = new DoubleParameter("sphere size", 0.1, 0.01, 0.4);
    
    private DoubleParameter cylinderRadius = new DoubleParameter("cylinder radius", 0.05, 0.01, 0.2);
    
    private BooleanParameter useg = new BooleanParameter( "use gravity", true );
    
    private DoubleParameter g = new DoubleParameter( "gravity", 1.0, 0.01, 1000 );
    
    private DoubleParameter k = new DoubleParameter( "spring stiffness", 100,0.01, 100000 );
        
    private DoubleParameter b = new DoubleParameter( "spring damping", 1, 0, 10 );
    
    private DoubleParameter c = new DoubleParameter( "viscous damping", .01, 0, 10 );

    private DoubleParameter restitution = new DoubleParameter( "restitution", .0001, 0, 1 );
    
    private DoubleParameter H = new DoubleParameter( "min distance (H)", 0.1, 0.01, 0.5 );
    
    private BooleanParameter repulsion = new BooleanParameter( "apply repulsion impulses", true );
    
    private BooleanParameter collision = new BooleanParameter( "apply collision impulses", true );
        
    public JPanel getControls() {
        VerticalFlowPanel vfp = new VerticalFlowPanel();
        
        VerticalFlowPanel vfp0 = new VerticalFlowPanel();
        vfp0.setBorder( new TitledBorder("Viewing Parameters" ) );
        vfp0.add( drawParticles.getControls() );
        vfp0.add( sphereSize.getSliderControls(false) );
        vfp0.add( cylinderRadius.getSliderControls(false) );
        CollapsiblePanel cp0 = new CollapsiblePanel( vfp0.getPanel() );
        cp0.collapse();
        vfp.add( cp0 );
        
        VerticalFlowPanel vfp1 = new VerticalFlowPanel();
        vfp1.setBorder( new TitledBorder("Simulation Parameters"));
        vfp1.add( repulsion.getControls() );
        vfp1.add( collision.getControls() );
        vfp1.add( useg.getControls() );
        vfp1.add( g.getSliderControls(true) );
        vfp1.add( k.getSliderControls(true) );
        vfp1.add( b.getSliderControls(false) );
        vfp1.add( c.getSliderControls(false) );        
        vfp1.add( restitution.getSliderControls(false) );
        vfp1.add( H.getSliderControls(false) );
        CollapsiblePanel cp1 = new CollapsiblePanel( vfp1.getPanel() );
        cp1.collapse();
        vfp.add( cp1 );
        
        return vfp.getPanel();        
    }
    
    public double[] pack()
    {
    	double[] state = new double[9*particles.size()];
    	int i = 0;
    	for(Particle p : particles) {
    		state[i++] = p.p.x;
    		state[i++] = p.p.y;
    		state[i++] = p.p.z;
    		state[i++] = p.v.x;
    		state[i++] = p.v.y;
    		state[i++] = p.v.z;
    		state[i++] = p.f.x;
    		state[i++] = p.f.y;
    		state[i++] = p.f.z;
    	}
    	return state;
    }
}
