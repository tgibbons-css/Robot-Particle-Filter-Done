package ParticleFilter;

import gui.MapJPanel;

import java.awt.Color;
import java.awt.Graphics2D;
import java.util.ArrayList;
import java.util.Random;

// this implements a particle filter with a set of random particles representing possible robot positions.
public class partFilter {
	
	ArrayList<particle> partList = new ArrayList<particle>(); 
	MapJPanel map;
	int num_part = 5000;
	public particle probablePart;				// most probable particle
	
	public partFilter() {
		for (int i=0; i<num_part; i++) {
			particle p = new particle();
			partList.add(p);
		}
		probablePart = partList.get(0);			// initialize the most probable particle to the first one to start with
	}
	
	public partFilter(MapJPanel mapP, int maxX, int maxY) {
		particle p;
		map = mapP;
		for (int i=0; i<num_part; i++) {
			do {
				p = new particle(maxX, maxY,i);
			} while (map.pixelFree(p.x,p.y));
			partList.add(p);
		}
		probablePart = partList.get(0);			// initialize the most probable particle to the first one to start with

	}
	
	public void display() {
		for(particle p: partList){
			p.print();
		}
	}
	
	public void print() {
		for (int i=0; i<num_part; i++) {
			System.out.print("P "+i+" ");
			partList.get(i).print();
			//System.out.println(" --- fitness  "+partList.get(i).fitness);
		}
	}

	void drawPart(int x, int y, double orient, int size, Graphics2D g, Color c){
		g.setColor(c);
		g.fillRect(x, y, size, size);
		int xc = x + size/2;
		int yc = y + size/2;
		g.drawLine(xc, yc, (int) Math.round(xc + (Math.cos(orient) * size)), (int) Math.round(yc - (Math.sin(orient) * size)));
	}
	
	
	public void drawParticles(Graphics2D g) {
		Color oldColor = g.getColor();
		//g.setColor(Color.RED);
		for(particle p: partList){
			drawPart(p.x, p.y, p.orientation, 4, g, Color.RED);
			Random rand = new Random();
			// only draw lines for a small number of particles
			int change  = rand.nextInt(num_part/5);
			if (change<=1) {
				p.meas.drawlines(p.x, p.y, p.orientation, g);				// draw the measurement lines to obstacles in 1% of particles
				//Double f = p.fitness;
				//g.drawString(f.toString(), p.x+2, p.y);
				
				//Double d = p.distance;
				//g.drawString(d.toString(), p.x, p.y+10);
				//g.drawString(String.format("%.2f",d), p.x+5, p.y+12);
			} 
			
			
		}
		drawPart(probablePart.x, probablePart.y, probablePart.orientation, 10, g, Color.YELLOW);
		g.setColor(oldColor);
	}
	
	// move particle forward dist pixels or units
	public void moveforward(double dist) {
		for(particle p: partList){
			p.moveforward(dist);
		}	
	}
	
	// rotate particle angle radians
	public void rotate(double angle) {
		for(particle p: partList){
			p.rotate(angle);
		}	
	}
	
	public void updateMeasurements(MapJPanel map) {
		for(particle p: partList){
			p.updateMeasurement(map);
		}		
		
	}
		
	public void addNoise() {
		for(particle p: partList){
			p.addNoise();
		}		
	}

	// returns a fitness value between 0 and 1 for each particle.  The higher the number, the more fit
	public void calcFitness(measurement goal) {
		double max = 0;
		double dist = 0;
		double prob = 0;
		double totalProb = 0;
		for(particle p: partList){
			dist = p.calcDistance(goal);
			if(dist>max) {
				max = dist;
			}
		}
		for(particle p: partList){
			prob = p.calcFitness(goal);
			totalProb += prob;
		}
		// normalize fitness so they sum to 1.0 for probabilities
		// also find the most probable particle
		max = 0;
		for(particle p: partList){
			prob = p.normalizeFitness(totalProb);
			if (prob>max) {
				max = prob;
				probablePart = p;
			}
			//System.out.println("Probability of particle "+p.index+" is "+p.probability);
		}
	}
	
	// =====================================================
	// For the assignment modify the code in the method below
	// =====================================================
	// resampleParticles() should select particles based on their probability.  The higher a particle's probability the more often it should be selected
	//  --- partList is the arraylist of particles in the filter
	//  --- give a particle p, p.probability measures how good this particle is.  This ranges from 0.0 to 1.0 and the sum of all the probabilities of all the particles is 1.0
	//  --- Currently this method simply copies each particle into a new arraylist, particlesCopy, and then re-assigns partList to this copy.
	//  --- You must modify the code below so that the chances of copying a particle is related to the probability of that particle
	public void resampleParticles() {
		ArrayList<particle> particlesCopy = new ArrayList<particle>(); 			// Arraylist to copy particles into

		Random rand = new Random();					// needed for random number generation
		int index = rand.nextInt(num_part); 		// random integer between 0 and number of particles
		double b = rand.nextFloat(); 				// random float between 0 and 1
		
		/* ---------- students can modify this version. Currently it just copies random particles into the new array list. */
		/*
		particle pNew;							// will hold the new particle created
		particle pOrig;							// will hold the old particle to copy
		// loop through number of particles. Creating one new particle each time through loop
		for (int i=0; i<num_part; i++) {
			//System.out.println("Selecting particle "+index+" whose prob is "+partList.get(index).probability);
			index = rand.nextInt(num_part); 		// Pick a random particle index... you should not do this
			pOrig = partList.get(index);			// using the index, grab the old particle
			pNew = new particle(pOrig);				// create the new copy of the original particle
			particlesCopy.add(pNew);				// add the new copy to the particle list
		}
		partList = particlesCopy;
		*/
		
		/* -------- working version  */
		particle pNew;
		particle pOrig;
		for (int i=0; i<num_part; i++) {
			b += rand.nextFloat();
			do  {
				index = (index+1)%num_part;
				b = (b - partList.get(index).probability);
			} while (partList.get(index).probability<b);
			//System.out.println("Selecting particle "+index+" whose prob is "+partList.get(index).probability);
			pOrig = partList.get(index);			// using the index, grab the old particle
			pNew = new particle(pOrig);				// create the new copy of the original particle
			particlesCopy.add(pNew);				// add the new copy to the particle list
		}
		partList = particlesCopy;
		/* */
	
	}
	
	

}
