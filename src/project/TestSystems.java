package project;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.LinkedList;
import java.util.List;
import java.util.Random;

import javax.swing.JButton;
import javax.swing.JPanel;
import javax.swing.border.TitledBorder;

import mintools.parameters.BooleanParameter;
import mintools.parameters.IntParameter;
import mintools.swing.CollapsiblePanel;
import mintools.swing.VerticalFlowPanel;

// Helper code to build a number of different test systems.  
public class TestSystems {
            
    private BooleanParameter clearFirst = new BooleanParameter( "clear current system before creating new systems", true );
    
    // Parameters to adjust the settings for the random system generator
    private IntParameter pNum = new IntParameter("Number of particles", 10, 2, 100);
    private IntParameter pinNum = new IntParameter("How many pinned", 3, 0, 100);
    private IntParameter eNum = new IntParameter("Number of springs", 5, 1, 60);
    
    private ParticleSystem system;
    
    private int s = 5;
    
    // for convenience we'll keep a copy of the particles, springs, and leaf springs inside a system,
    // though this is a bit gross        
    private List<Particle> particles;
    private List<Spring> springs;
    
    // Creates a new test system 
    public TestSystems( ParticleSystem system ) {
        this.system = system;
        particles = system.particles;
        springs = system.springs;        
    }
    
    /**
     * Quick and dirty generic test generation button
     * @author kry
     */
    private class TestButton extends JButton implements ActionListener {
        private static final long serialVersionUID = 1L;
        private int testNumber;
        public TestButton( String name, int testNumber ) {
            super( name );
            this.testNumber = testNumber;
            addActionListener( this );
        }
        @Override
        public void actionPerformed(ActionEvent e) {
            createSystem(this.testNumber);
        }
    }
    
    // Gets the control panel for setting different systems.
    public JPanel getControls() {
        VerticalFlowPanel vfp = new VerticalFlowPanel();
        vfp.setBorder( new TitledBorder("Particle System Test Systems"));
        vfp.add( clearFirst.getControls() );
        
        for ( int i = 0; i < tests.length; i++ ) {            
            vfp.add( new TestButton(tests[i], i) );             
        }
        
        vfp.add(pNum.getControls());
        vfp.add(pinNum.getControls());
        vfp.add(eNum.getControls());
        
        CollapsiblePanel cp = new CollapsiblePanel( vfp.getPanel() );
        cp.collapse();
        return cp;   
    }
    
    // Test system names
    public String[] tests = {            
            "Simple edge-edge",
            "Simple edge-cable",
            "Simple cable-cable",
            "Swinger",
            "Together",
            "Hoop",
            "Hoop and basketball",
            "Random system"
    };
    
    // Creates one of a number of simple test systems.
    public void createSystem( int which ) {
        if ( clearFirst.getValue() ) {
        	system.clear();
        }
  
        // Simple 4 particle test system
        if (which == 0 ) {
        	Particle p1 = new Particle( 0, 1, 0, 0, 0, 0 );
            Particle p2 = new Particle( 4, 6, 0, 0, 0, 0 );
            Particle p3 = new Particle( 3, 1, 2, 0, 0, 0 );
            Particle p4 = new Particle( 3, 2, -2, 0, 0, 0);
            particles.add( p1 );
            particles.add( p2 );
            particles.add( p3 );
            particles.add( p4 );
            springs.add( new Spring( p1, p2 ) );
            springs.add( new Spring( p3, p4 ) );
            p1.pinned = true;
            p3.pinned = true;
            p4.pinned = true;
            system.name = tests[which];           
        } 
        
        // Simple one cable, one edge test system
        else if (which == 1) {       	
        	Particle p1 = new Particle( 1, 3, 2, 0, 0, 0 );
            Particle p2 = new Particle( 1, 2, -5, 0, 0, 0 );
            system.createCable(p1, p2, 70);  
            p1 = new Particle( -5, 1, 0, 0, 0, 0 );
            p2 = new Particle( 5, 1, 0, 0, 0, 0 );
            particles.add(p1);
            particles.add(p2);
            springs.add( new Spring( p1, p2 ) );
            p1.pinned = true;
            p2.pinned = true;
            system.name = tests[which];           
        } 
        
        // Simple two cable test system
        else if (which == 2) {
        	Particle p1 = new Particle( 1, 3, 2, 0, 0, 0 );
            Particle p2 = new Particle( 1, 2, -5, 0, 0, 0 );
            system.createCable(p1, p2, 70); 
            p1 = new Particle( -1, 1, 0, 0, 0, 0 );
            p2 = new Particle( 3, 2, 0, 0, 0, 0 );
            system.createCable(p1, p2, 70); 
            system.name = tests[which]; 
        } 
        
        // A more dynamic, simple system
        else if (which == 3 ) {
        	Particle p1 = new Particle( 0, 6, 0, 0, 0, 0 );
            Particle p2 = new Particle( -4, 10, 0, 0, 0, 0 );
            Particle p3 = new Particle( 2, 25, 2, 0, 0, 0 );
            Particle p4 = new Particle( 2, 23, -2, 0, 0, 0);
            particles.add( p1 );
            particles.add( p2 );
            particles.add( p3 );
            particles.add( p4 );
            springs.add( new Spring( p1, p2 ) );
            springs.add( new Spring( p3, p4 ) );
            p1.pinned = true;

            system.name = tests[which];
        }
        
        // A planar collision system
        else if (which == 4 ) {
        	Particle p1 = new Particle( 0, 6, 0, 0, 0, 0 );
            Particle p2 = new Particle( 2, 5, -2, 0, 0, 0 );
            Particle p3 = new Particle( -2, 4, 2, 0, 0, 0 );
            particles.add( p1 );
            particles.add( p2 );
            particles.add( p3 );
            springs.add( new Spring( p1, p2 ) );
            springs.add( new Spring( p1, p3 ) );
            p1.pinned = true;
            p2.mass = 3.0;
            system.name = tests[which];
        }
        
        // Basketball hoop test system
        else if (which == 5 || which == 6) {
        	
        	Particle p1;
        	Particle p2;
        	Particle[] plist = new Particle[10];
        	Particle hold1;
        	Particle hold2;
        	double pi = Math.PI;
        	
        	// First create the rim of the hoop, and the first loop of free particles
    		p1 = new Particle(2*Math.cos(0), 5.0, 2*Math.sin(0), 0.0, 0.0, 0.0 );
    		particles.add(p1);
    		p1.pinned = true;
    		hold1 = p1;
    		
    		plist[0] = new Particle(2*Math.cos(2*pi/10.0 * -0.5), 3.0, 2*Math.sin(2*pi/10.0 * -0.5), 0.0, 0.0, 0.0 );        		
            system.createCable(p1, plist[0], s);
            plist[0].pinned = false;

        	for (double i = 1; i < 10.0; i += 1.0) {
        		p2 = new Particle(2*Math.cos(2*pi/10.0 * i), 5.0, 2*Math.sin(2*pi/10.0 * i), 0.0, 0.0, 0.0 );
        		plist[(int)i] = new Particle(2*Math.cos(2*pi/10.0 * (i - 0.5)), 3.0, 2*Math.sin(2*pi/10.0 * (i - 0.5)), 0.0, 0.0, 0.0 );  
        		
        		particles.add(p2);
        		springs.add( new Spring(p1, p2));
        		p2.pinned = true;
        		
                system.createCable(p1, plist[(int)i], s);
                system.createCable(p2, plist[(int)i], s);
        		plist[(int)i].pinned = false;
        		
        		p1 = p2;        		
        	}
        	springs.add( new Spring(p1, hold1));
        	system.createCable(p1, plist[0], s);
        	plist[0].pinned = false;    	
        	
        	// Now add the rest of the cables for the hoop
        	for (double j = 0; j < 4.0; j += 1.0) {
        		
            	p1 = plist[0];
            	plist[0] = new Particle(2*Math.cos(2*pi/10.0 * -(j*0.5 + 1.0)), 2.5 - j * 0.5, 2*Math.sin(2*pi/10.0 * -(j*0.5 + 1.0)), 0.0, 0.0, 0.0 );
            	system.createCable(p1, plist[0], s);           	
                p1.pinned = false;
                plist[0].pinned = false;
                
        		for (double i = 1; i < 10.0; i += 1.0) {
        			p2 = plist[(int)i];
                	plist[(int)i] = new Particle(2*Math.cos(2*pi/10.0 * (i-(j*0.5 + 1.0))), 2.5 - j * 0.5, 2*Math.sin(2*pi/10.0 * (i-(j*0.5 + 1.0))), 0.0, 0.0, 0.0 );  
                	                		
                    system.createCable(p1, plist[(int)i], s);
                    system.createCable(p2, plist[(int)i], s);
                    plist[(int)i].pinned = false;
                	p1.pinned = false;
                	p2.pinned = false;
                		
                	p1 = p2;
        		}
        		system.createCable(p1, plist[0], s);
        		p1.pinned = false;
        		plist[0].pinned = false;
        	}  
        	
        	if (which == 6) {
	        	// Now create the basketball
	        	double max = 2.0;
	    		int mid = (int) (max/2 - 1);
	    		double scale = 1.5 - 1.5*(-1.0 - mid)*(-1.0 - mid)/(max + 2.0);
	        	
	    		// The top and bottom points of the ball
	        	hold1 = new Particle(0.0, 11.5, 0.0, 0.0, 0.0, 0.0 );
	    		particles.add(hold1);
	    		
	    		hold2 = new Particle(0.0, 8.5, 0.0, 0.0, 0.0, 0.0 );
	    		particles.add(hold2);
	    		
	    		// Create the first loop at the top of the ball
	    		p1 = new Particle(scale*Math.cos(0), 10.0 - 2.0*(-1.0 - mid)/max, scale*Math.sin(0), 0.0, 0.0, 0.0 );
	    		particles.add(p1);
	    		springs.add( new Spring(p1, hold1));
	    		plist[0] = p1;
	    		
	    		for (double i = 1.0; i < 10.0; i += 1.0) {
	    			plist[(int)i] = new Particle(scale*Math.cos(2*pi/10.0 *i), 10.0 - 2.0*(-1.0 - mid)/max, scale*Math.sin(2*pi/10.0 * i), 0.0, 0.0, 0.0 );
	    			particles.add(plist[(int)i]);
	    			springs.add(new Spring(plist[(int)i - 1], plist[(int)i]));
	    			springs.add(new Spring(plist[(int)i], hold1));
	    		}
	    		
	    		springs.add( new Spring(p1, plist[9]));
	   		
	    		// Create the inner loops of the ball
	    		for (double j = 0.0; j < max; j++) {
	    			
	    			scale = 1.5 - 1.5*(j - mid)*(j - mid)/(max + 2.0);
	    			p1 = new Particle(scale*Math.cos(0), 10.0 - 2.0*(j-mid)/max, scale*Math.sin(0), 0.0, 0.0, 0.0 );
	        		particles.add(p1);
	        		springs.add( new Spring(p1, plist[0]));
	        		plist[0] = p1;
	        		
	        		for (double i = 1.0; i < 10.0; i += 1.0) {
	        			p2 = plist[(int)i];
	        			plist[(int)i] = new Particle(scale*Math.cos(2*pi/10.0 *i), 10.0 - 2.0*(j-mid)/max, scale*Math.sin(2*pi/10.0 * i), 0.0, 0.0, 0.0 );
	        			particles.add(plist[(int)i]);
	        			springs.add(new Spring(plist[(int)i - 1], plist[(int)i]));
	        			springs.add(new Spring(plist[(int)i], p2));
	        		}
	        		
	        		springs.add( new Spring(p1, plist[9]));
	    		}
	    		
	    		// Connect the rest of the ball to the bottom point
	    		for (double i = 0.0; i < 10.0; i += 1.0) {
	    			springs.add(new Spring(plist[(int)i], hold2));
	    		}	    		
	    		springs.add( new Spring(plist[0], plist[9]));
        	}    		
    		system.name = tests[which]; 
        }
        
        // Create a random system
        else if (which == 7) {
        	
        	Random rand = new Random();
        	
        	// Randomly generate all the particles
        	List<Particle> p = new LinkedList<Particle>();
        	for(int i = 0; i < pNum.getValue(); i++) {
        		p.add(new Particle(rand.nextDouble() * 8.0 - 4.0, rand.nextDouble() * 4.0 + 4.0, 
        				rand.nextDouble() * 8.0 - 4.0, 0.0, 0.0, 0.0));
        		particles.add(p.get(i));
        		if (pinNum.getValue() >= pNum.getValue()) { p.get(i).pinned = true; }
        	}
        	
        	// Randomly pin the correct number of particles
        	if (pinNum.getValue() < pNum.getValue()) {
        		for (int i = 0; i < pinNum.getValue(); i++) {
        			Particle par = p.get(rand.nextInt(p.size()));        			
        			if (par.pinned != true) { par.pinned = true; }
        			else { i--; }
        		}
        	}
        	
        	// Create edges between all particles if that many edges were asked
        	if (eNum.getValue() >= pNum.getValue() * (pNum.getValue() - 1) / 2) {
        		
        		for (int i = 0; i < pNum.getValue(); i++) {
        			for (int j = 0; j < pNum.getValue(); j++) {
        				if (i < j) {
        					springs.add( new Spring( p.get(i), p.get(j) ) );
        				}
        			}
        		}
        	}
        	
        	// Otherwise randomly assign edges
        	else {
        		
        		for (int i = 0; i < eNum.getValue(); i++) {
        			Particle a = p.get(rand.nextInt(p.size()));
        			Particle b = p.get(rand.nextInt(p.size()));
        			if (b.p.equals(a.p)) { 
        				i--;
        				continue;
        			}

        			boolean dub = false;
        			for (int j = 0; j < springs.size(); j++) {
        				if (a.p.equals(springs.get(j).p1.p) && b.p.equals(springs.get(j).p2.p)) { dub = true; }
        				else if (b.p.equals(springs.get(j).p1.p) && a.p.equals(springs.get(j).p2.p)) { dub = true; }
        			}
        			
        			if (dub) { i--; }
        			else {
        				springs.add( new Spring(a, b));
        			}    			
        		}
        	}
        	
        	// Remove useless particles
        	for (int i = 0; i < particles.size(); i++) {
        		if (particles.get(i).springs.size() == 0) {
        			particles.remove(i);
        			i--;
        		}
        	}
            system.name = tests[which];
        }
    }    
}
