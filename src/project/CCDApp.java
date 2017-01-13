package project;

import java.awt.Component;
import java.awt.Dimension;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;
import com.jogamp.opengl.util.gl2.GLUT;

import javax.swing.JPanel;
import javax.swing.border.TitledBorder;

import mintools.parameters.BooleanParameter;
import mintools.parameters.DoubleParameter;
import mintools.parameters.IntParameter;
import mintools.swing.CollapsiblePanel;
import mintools.swing.VerticalFlowPanel;
import mintools.viewer.EasyViewer;
import mintools.viewer.Interactor;
import mintools.viewer.SceneGraphNode;

// The application
public class CCDApp implements SceneGraphNode, Interactor {

    private EasyViewer ev;
    
    public ParticleSystem system;
    
    public TestSystems testSystems;
        
    public static void main(String[] args) {
        new CCDApp();        
    }
        
    // Creates the application / scene instance
    public CCDApp() {
        system = new ParticleSystem();
        testSystems = new TestSystems( system );
        ev = new EasyViewer( "CCD Project - Thomas Jansen", this, new Dimension(640,480), new Dimension(640,480) );
        
        // we add ourselves as an interactor to set up mouse and keyboard controls
        ev.addInteractor(this);
    }
     
    @Override
    public void init(GLAutoDrawable drawable) {
        GL gl = drawable.getGL();
        gl.glEnable( GL.GL_BLEND );
        gl.glBlendFunc( GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA );
        system.init(drawable);
    }
    
    // Constants for the lights
    final float[] white = {1,1,1,1};
	final float[] black = {0,0,0,1};
	final float[] grey = {0.75f,0.75f,0.75f,1f};
	final float[] lightPos = {2, 5, 5};
	    
	// A method that sets the lights
    void setLights( GLAutoDrawable drawable ) {
    	GL2 gl = drawable.getGL().getGL2();
		gl.glEnable( GL2.GL_LIGHTING );
		gl.glEnable( GL2.GL_LIGHT0 );
		// WATCH OUT: need to provide homogeneous coordinates to many calls!! 
		float[] lightPosition = {lightPos[0], lightPos[1], lightPos[2], 1}; 
		float[] dark = new float[] {0.1f,0.1f,0.1f,1};
		gl.glLightfv( GL2.GL_LIGHT0, GL2.GL_POSITION, lightPosition, 0 );
		gl.glLightfv( GL2.GL_LIGHT0, GL2.GL_AMBIENT, black, 0);
		gl.glLightModelfv( GL2.GL_LIGHT_MODEL_AMBIENT, dark, 0); 
    }   
    
    GLUT glut = new GLUT();
    
    @Override
    public void display(GLAutoDrawable drawable) {
        GL2 gl = drawable.getGL().getGL2();        
        gl.glClear( GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT );
        
        // First advance the system (if it is running or wants to be stepped)
        if ( run.getValue() || stepRequest ) { 
        	stepRequest = false;
            for ( int i = 0; i < substeps.getValue(); i++ ) {
                if ( ! system.updateParticles( stepsize.getValue() / substeps.getValue() )) {
                    run.setValue( false );
                    break;
                }
            }
        }
        
        setLights( drawable );
        
        int list = -1;
        
        // Create the plane
        if ( list != -1 ) {
        	gl.glCallList(list);
        } else {
	        list = gl.glGenLists(1);
	    	gl.glNewList(list, GL2.GL_COMPILE_AND_EXECUTE );        
	        gl.glMaterialfv( GL.GL_FRONT_AND_BACK, GL2.GL_SPECULAR, white, 0 );
	        gl.glMaterialf( GL.GL_FRONT_AND_BACK, GL2.GL_SHININESS, 127 );
	        for ( int i = -20; i < 20; i++ ) {
	        	for ( int j = -20; j <= 10; j++ ) {
	                gl.glBegin( GL2.GL_QUAD_STRIP );
	                gl.glMaterialfv( GL.GL_FRONT_AND_BACK, GL2.GL_AMBIENT_AND_DIFFUSE, ((i+j)%2)==0?grey:white, 0 );
	                gl.glNormal3f(0,1,0);
	                gl.glVertex3d( i, -5, j );
	                gl.glVertex3d( i, -5, j+1 );
	                gl.glVertex3d( i+1, -5, j );
	                gl.glVertex3d( i+1, -5, j+1 );        
	                gl.glEnd();
	        	}
	        }
	        gl.glEndList();
        }
        
        system.display( drawable );

    }
    
    //boolean to signal that the system was stepped
    private boolean stepRequest = false;
        
    private BooleanParameter run = new BooleanParameter( "simulate (press SPACE in canvas to toggle)", false );
    
    private DoubleParameter stepsize = new DoubleParameter( "step size", 0.05, 1e-5, 1 );
    
    private IntParameter substeps = new IntParameter( "sub steps (integer)", 1, 1, 100);
            
    @Override
    public JPanel getControls() {
        VerticalFlowPanel vfp = new VerticalFlowPanel();
               
        VerticalFlowPanel vfp0 = new VerticalFlowPanel();
        vfp0.setBorder( new TitledBorder("Numerical Integration Controls"));
        vfp0.add( run.getControls() );        
        vfp0.add( stepsize.getSliderControls(true) );
        vfp0.add( substeps.getControls() );
        CollapsiblePanel cp0 = new CollapsiblePanel( vfp0.getPanel() );
        cp0.collapse();
        vfp.add( cp0 );
        
        vfp.add( system.getControls() );       
        vfp.add( testSystems.getControls() );
                
        return vfp.getPanel();
    } 
    
    @Override
    public void attach(Component component) {

        component.addKeyListener( new KeyAdapter() {
            @Override
            public void keyPressed(KeyEvent e) {
                if ( e.getKeyCode() == KeyEvent.VK_SPACE ) {
                    run.setValue( ! run.getValue() ); 
                } else if ( e.getKeyCode() == KeyEvent.VK_S ) {                    
                    stepRequest = true;
                } else if ( e.getKeyCode() == KeyEvent.VK_R ) {
                    system.resetParticles();
                } else if ( e.getKeyCode() == KeyEvent.VK_C ) {                   
                    system.clear();
                } else if ( e.getKeyCode() == KeyEvent.VK_ESCAPE ) {
                    // quit the program
                    ev.stop();
                } 
                if ( e.getKeyCode() != KeyEvent.VK_ESCAPE ) ev.redisplay();
            }
        } );
    }    
}
