package org.usfirst.frc.team1923.robot.commands.debug;

import java.io.*;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;

/** This command creates a new thread (so it doesn't affect main processes)
 * Roborio can handle multithreading 
 * @author anikaitsingh 
 */
public class Logging extends InstantCommand {
	
	public long last;
	public FileWriter f;
	

	public Logging(){
		try {
			f = new FileWriter("Log.txt");
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	public void initialize() {
		Thread t = new Thread(() -> {
            while (!Thread.interrupted()) {
            	if(System.currentTimeMillis() > (last + 500)){
        			log();
        		}
            }
        });
        t.start();
	}
	
	public void log(){
		try {
			f.write("{" + System.currentTimeMillis() + "}: ");
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}//log
	}
}
