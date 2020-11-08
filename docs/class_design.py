# Follower bot using LIDAR
# Design / Psuedo Code / Method Signatures

class Main:

	{world,robot{pos,vel},target{pos,vel},id,pred,recovery}

	def init(scenario_config):
		sub :: /map, /odom, /laser

	def callback_odom(odom_msg):
		robot.update(odom_msg)

	def callback_laser(laser_msg):
		target{pos,vel} <- id.classify(laser_msg)

	def callback_map(map_msg):
		world <- map_msg

	def main_loop():
		while alive:

			target.move()

			switch id.status():
				case CLOSE|OK|FAR:
					pred.update(target{pos,vel})
					next_target{pos,vel} <- pred.predict()
				case OOR|OOS:
					recovery.update()
					next_target{pos,vel} <- recovery..predict()

			robot.move(pred,id)

class Identifier:

	{target{pos, lin_vel, last_pos, last_vel}, [obs], status(CLOSE|OK|FAR|OOR|OOS)}

	def classify(laser_msg):
	    target.pos, [obs] <- laser_msg
	    target.lin_vel <- (target.pos, target.last_pos)
	    return target{pos,vel}

    def status():
    	return status <- |target.pos|

class Predictor:

	{[pose], lin_vel, ang_vel}

	def update(target{pos,vel})
		[pose] <- target{pos,vel}

	def predict()
		return lin vel, ang vel <- [pose]

class Robot:

	{pos, vel{lin,ang}, target_pos, target_vel{lin,ang}}

	def update(id.[obs],odom or vel{lin,ang}):
		pos <- [obs],odom or vel{lin,ang}

	def move(pred,id)
		linvel, angvel <- pred.linvel, pred.angvel, id.obs, id.status
		pub :: /cmd_vel_robot

class Target:

	{pos,vel{lin,ang}}

	def move(): # hardcoded
		pos, vel <- hardcoded
		pub :: /cmd_vel_target

class Recovery:

	{last_known{pos,vel}, lost_time, [expected_pose]}

	def predict():
		if lost_time > thresh:
			vel{lin, ang} <- brute_force_search()
		else:
			vel{lin, ang} <- prediction.predict(expected_poses)
		[expected_pose] <- (vel{lin,ang}, robot.pos)
		pub :: /cmd_vel_robot
		robot.update(vel{lin,ang})

	def brute_force_search():
		if not found()
			return vel{lin, ang} <- {last_known{pos,vel}, lost_time, [expected_pose]}
		return None

	def found():
		return id.target.pos is not None
