Mo 06.05.24 2h

- erster Teil des Tutorials funktioniert vollständig. Problem waren fehlende <depend exec> Einträge in
package.xml .
- hinzufügen eines weiteren Zylinder geht auch
- momentan gibt es ein Problem mit dem Task, der den ersten Zylinder greifen soll:

[mtc_tutorial-1] [INFO] [1715017638.841919824] [moveit.ompl_planning.model_based_planning_context]: Planner configuration 'panda_arm' will use planner 'geometric::RRTConnect'. Additional configuration parameters will be set when the planner is constructed.
[mtc_tutorial-1] [WARN] [1715017639.103917497] [moveit_robot_state.cartesian_interpolator]: The computed trajectory is too short to detect jumps in joint-space Need at least 10 steps, only got 2. Try a lower max_step.

Ich vermute es liegt am cpp-File des Tutorials. Das sehe ich mir dann bis Ende der Woche an.

- die anderen Tasks werden in Rviz geladen aber nicht durchgeführt, da der 2. Schritt (siehe oben) nicht funktioniert

Bemerkung: ich habe alle <depend exec> Referenzen aus dem xml-File (etwa 10) von movit2_tutorial kopiert und nicht getestet, welche davon ich tatsächlich brauche. Die Kompilierzeit ist aber nicht wesentlich größer. 

============================================================================================================
Di 07.05.24 4h

============================================================================================================
Mi 12.05.24 4h

variable max_step bzw. eef_step gefunden in file 
<moveit-workspace> = ws-moveit/src/moveit_task_constructor/core/src/solvers/cartesian_path.cpp
relevantes header files gefunden unter
ws-moveit/src/moveit_task_constructor/core/include/moveit
./task_constructor/solvers/cartesian_path.h
./task_constructor/solvers/planner_interface.h
./task_constructor/properties.h

/opt/ros/humble/include/moveit/robot_state/cartesian_interpolator.h

-> field step_size seems to be synonym for max_step; try with max_step = 0.01*5

- remaining issue in the sampling_planner / interpolation planner 
- task issue name in rviz: Trajectory end-point deviates too much from goal state error;
	apparently the paramter max_distance to target for valid position of the robot got changed from ros1 to ros2, 
	leading to rejection of all calculated paths: 
		https://github.com/moveit/moveit_task_constructor/pull/542
		https://github.com/moveit/moveit_task_constructor/issues/540#issuecomment-1988323674
		https://github.com/ros-planning/moveit_task_constructor/blob/0c4b4fcaa89992f8b0a6f19a46342bd51352c5c9/core/src/stages/connect.cpp#L60

	-> solution adjust max_value to 1e-2 in ws-moveit/src/moveit_task_constructor/core/src/stages/connect.cpp
			p.declare<double>("max_distance", 1e-4, "maximally accepted distance between end and goal sate");
			p.declare<double>("max_distance", 1e-2, "maximally accepted distance between end and goal sate");
			
- solution works; tutorial runs as advertised; continue to adjust tutorial to exercise task

============================================================================================================
Mo 20.05.24 8h
movement to desired position works rudimentary:
- goal is to describe position of first object relative to second
- current problem is collision detection; second object not treated as stable surfa
	-> no viable paths found because of that
	
	-> solution declare as viable surface with 
			moveit::planning_interface::MoveGroupInterface.setSupportSurfaceName("object2")
		
		PROBLEM MoveGroupInterface not declared in the current cpp file -> inherited class instead?
		PROBLEM where to declare the property
		
		https://moveit.picknik.ai/humble/doc/examples/pick_place/pick_place_tutorial.html#setting-place-location-pose
		
		https://moveit.picknik.ai/humble/api/html/classmoveit_1_1planning__interface_1_1MoveGroupInterface.html#a42ec25f3ac294283222cbf5fe33c8907
		
		proceeding with moveit Tutorials/Examples in hope of finding out
	
	-> solution seems depreciated; new solution: in mtc_tutorial.cpp add item "object2" to the Function allowCollisions()
		-> the collision of both objects should not cause the trajectory to be rejected
		seems to work
		
	PROBLEM: retreat-vektor/dirkction (stage retreat) does not fit with placement; arm collides with/retreats into object
	SOLVED: - fixed by inverting retreat vector (adjustment depending on object position required 
				-> consider changing frame to object for this)
			- collision was not the problem; the assumed fix, adding "object2" caused compillation error
	
	TASK REMAINING: switch from absolute positioning, directions to relative ones by using object frames instead of world-frame 
					or accessing the object pose, position with getter-methods if possible

============================================================================================================
Di 21.05.24 4h

uploaded working solution; looked into REMAINING TASK: accessing object poses at runtime might be possible by subscribing to "monitored_planning_scene" topic
  -> have to take a deeper look in the API
