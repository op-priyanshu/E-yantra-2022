while True:
				
				t2=sim.getSimulationTime()
				print(t2-t)
				if t2-t>=46:
					sim.setJointTargetVelocity(lj,1)
					sim.setJointTargetVelocity(rj,1)
					break
				if dist2>0.1:
					sim.setJointTargetVelocity(lj,-0.1) 
					sim.setJointTargetVelocity(rj,0.1)
		        				
			
				else :
					sim.setJointTargetVelocity(lj,0.1) 
					sim.setJointTargetVelocity(rj,-0.1)