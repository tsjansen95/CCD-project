# CCD-project
Continuous Collision Detection in 3-D

Please read the CCD paper before running the application. The file is titled CCDpaper.pdf

* Note: The simulation is not perfect, and requires some work (specifically in terms of
  optimization and cable collisions). Also the plane only serves as a visual aid, there
  are no collisions with it.

For running the program:

- The src folder contains all necessary classes to run the project. 

- mintools.jar, vecmath.jar, mtj.jar, jogl-all.jar, and gleugen-rt.jar must be included
  in the build path in order for the program to run. 

- jogl-all.jar and gleugen-rt.jar should be kept in the jar folder alongside all the 
  natives jar files.

- In order for the libraries to work properly, a 64-bit JVM should be used.

- To run the program, run the CCDApp class as a Java Application.


For using the user interface:

- Create a system by choosing one of the options in the Particle Test Systems section.

- The random system button at the bottom randomly generates points and springs based on 
  the numbers given in the boxes beneath the button. The system will have the number of
  edges that was inputted, but if there are particles that are not attached to any 
  springs when the system is generated, they will be removed (therefore generating less
  particles than what the box actually suggests).

- The physics, viewing, and numerical integration controls can be adjusted as well, 
  although it is not necessary. Some configurations of these controls can result in an
  inaccurate simulation (Such as maxing out the step size).
