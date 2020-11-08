In order to compile your urdf file to mujoco xml file:
	- insert the <mujoco> part as  in the file 'panda_arm_hand_without)cam_mujoco_v_3.xml
	- rename your urdf file into .xml instead of .urdf
	- go into your mujoco folder (/home/onur/.mujoco/mujoco200/bin
	- open terminal and type 'compile inputfile outputfile'
	- note that you name your output file as .xml
	- then you can add additional elements to your new mujoco .xml file

Notes on the files:
	-panda_arm_hand_without_cam_mujoco_v_3.xml : urdf used to compile into mujoco xml file. Note here mujoco uses the meshes to infer its own inertia values (see compiler infos) and this works very good in the inverse kinematics control!
	-panda_arm_hand_without_cam_mujoco_v_3_use_orig_inertia.xml: urdf used to compile into mujoco xml file. Note here the original inertia values given in the urdf file are used (compile of xml file is not said to infere inertia from meshes) and when usinge inverse kinematics    	 control this does not work!!!
	
	-panda_without_cam_mujoco.xml: the mujoco xml which infers its own inerta values from the meshes
	-panda_without_cam_original_inertia.xml: the mujoco xml which uses original inertia values from urdf (not working well in inverse kinematics!)
	
	-panda_without_cam_mujoco_working_version.xml: copy of 'panda_without_cam_mujoco.xml' in a working version
