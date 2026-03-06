# code to caluculate moment of inertia
# To run, please paste into freecad's python console. 

"""
path = "/home/legrs/fksb/cal/rw_inertia.py"
def cal():
    with open(path, "r", encoding="utf-8") as f:
        exec(f.read())
"""

import Draft

name= [
        "fe", 
        "pla",  
        "al",  
        ]
# [g/cm^3]
density= [
        7.85,
        1.25,
        2.77, #A7075
        ]
# [kg*mm^2]
moi = [
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0
        ]
center = [0,0,12.1] # const
mass = 0.0
com = [0,0,0]
doc = App.ActiveDocument
for i in range(len(name)):
    print(name[i])

    # [g/cm^3] → [kg/mm^3]
    density[i] = density[i] / 1000/1000

    obj = doc.getObjectsByLabel(name[i])[0]
    # double
    vol = obj.Shape.Volume
    mass = mass + vol * density[i]

    if name[i]=="fe":
        print(mass*1000)

    for j in range(len(obj.Shape.Solids)):
        # kg*mm*mm ( @ density=1 kg/mm/mm/mm)
        rawI = obj.Shape.Solids[j].MatrixOfInertia
        # mm
        cog = obj.Shape.Solids[j].CenterOfGravity
        d = [center[0] - cog.x, center[1] - cog.y, center[2] - cog.z]

        Ixx =  obj.Shape.Solids[j].Volume * (d[1]**2 + d[2]**2)
        Iyy =  obj.Shape.Solids[j].Volume * (d[0]**2 + d[2]**2)
        Izz =  obj.Shape.Solids[j].Volume * (d[0]**2 + d[1]**2)
        Ixy = -obj.Shape.Solids[j].Volume * d[0] * d[1]
        Ixz = -obj.Shape.Solids[j].Volume * d[0] * d[2]
        Iyz = -obj.Shape.Solids[j].Volume * d[1] * d[2]
        #if i == 3:
            

        moi[0] = moi[0] + density[i]*(rawI.A11 + Ixx)
        moi[1] = moi[1] + density[i]*(rawI.A12 + Ixy)
        moi[2] = moi[2] + density[i]*(rawI.A13 + Ixz)
        moi[3] = moi[3] + density[i]*(rawI.A21 + Ixy)
        moi[4] = moi[4] + density[i]*(rawI.A22 + Iyy)
        moi[5] = moi[5] + density[i]*(rawI.A23 + Iyz)
        moi[6] = moi[6] + density[i]*(rawI.A31 + Ixz)
        moi[7] = moi[7] + density[i]*(rawI.A32 + Iyz)
        moi[8] = moi[8] + density[i]*(rawI.A33 + Izz)

        com[0] = com[0] + cog.x*obj.Shape.Solids[j].Volume*density[i]
        com[1] = com[1] + cog.y*obj.Shape.Solids[j].Volume*density[i]
        com[2] = com[2] + cog.z*obj.Shape.Solids[j].Volume*density[i]
        print()
        

mass = mass * 1000
for i in range(len(moi)):
    # kg → g
    moi[i] = moi[i]*1000

# massがgで vol*densityがkgだから
com[0] = com[0]*1000/mass - center[0]
com[1] = com[1]*1000/mass - center[1]
com[2] = com[2]*1000/mass - center[2]

# print string for pasting into godot_tr/Main.cs
#
print()
print("    static float cm = ",mass,"F; // g", sep="")
print("    static float COMx = ",com[0],"F; // mm", sep="")
print("    static float COMy = ",com[1],"F;", sep="")
print("    static float COMz = ",com[2],"F;", sep="")
print("    static double Ixx = ",moi[0],"; // g*mm^2", sep="")
print("    static double Iyy = ",moi[4],";", sep="")
print("    static double Izz = ",moi[8],";", sep="")
print("    static double Ixy = ",moi[1],";", sep="")
print("    static double Ixz = ",moi[2],";", sep="")
print("    static double Iyz = ",moi[5],";", sep="")
print()
print("    static float wmoi1 = ",moi[0],"F; // g*mm^2  around the axis perpendicular to rotation axis", sep="")
print("    static float wmoi2 = ",moi[8],"F;// around rotation axis", sep="")
print()

