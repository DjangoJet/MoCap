import bpy
import pickle
import numpy as np
with open ('/home/mike/Code/Mocap/symulation/symulationPoints', 'rb') as f :
	symPoints = pickle.load(f)
ob = bpy.data.objects["Suzanne"]

frame_num = 0

for symPoint in symPoints :
	bpy.context.scene.frame_set(frame_num)
	ob.location.x = symPoint[0][0,0]/100
	ob.location.y = symPoint[0][1,0]/100
	ob.location.z = symPoint[0][2,0]/100
	ob.rotation_euler = (symPoint[1][0,0]*np.pi/180, symPoint[1][1,0]*np.pi/180 ,symPoint[1][2,0]* np.pi/180)
	ob.keyframe_insert(data_path = "location" , index = -1)
	ob.keyframe_insert (data_path = "rotation_euler", index = -1)
	frame_num += 20
