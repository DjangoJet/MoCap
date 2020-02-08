# import bpy
import pickle
import numpy as np
    
with open('/home/mike/Code/Mocap/symulation/symulationPoints', 'rb') as f:
    symPoints = pickle.load(f)

print(symPoints)
# ob = bpy.data.objects["Suzanne"]

# frame_num = 0

# for symPoint in symPoints:
#     bpy.context.scene.frame_set(frame_num)
#     ob.location.x = symPoint[0][0,0]
#     ob.location.y = symPoint[0][1,0]
#     ob.location.z = symPoint[0][2,0]
#     ob.keyframe_insert(data_path="location", index = -1)
#     frame_num += 20