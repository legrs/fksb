# code to caluculate moment of inertia
# To run, please paste into freecad's python console. 

"""
path = "/home/legrs/fksb/cal/cal_inertia.py"
def cal():
    with open(path, "r", encoding="utf-8") as f:
        exec(f.read())
"""

import Draft

objName= [
        "moterMountX",  # mount for Xmotor
        "moterMountY",  # mount for Ymotor
        "MotarMountZ",  # high voltage circuit panel(FR-4)
        "Extrude022",    # wheel of Xmotor
        "Extrude019",    # wheel of Ymotor 分解しないとでてこないよ 警告がでるが無視してやれ
        "wheel",         # wheel of Zmotor
        "m5nutX",       # fixing nut for wheel X こっちで作った
        "m5nutY",       # fixing nut for wheel Y こっちで作った
        "m5nutZ",       # fixing nut for wheel Z こっちで作った
        "Fusion008",     # Xmotor
        "Fusion010",     # Ymotor 分解しないとでてこないよ 警告がでるが無視してやれ
        "moterZ",        # Zmotor
        "Tamazo",       # 3S1P360RE 
        "cap",           # capacitor こっちで作った
        "ESC",          # ESC of motors
        "Extrude054",    # joint
        "Antena",       # antenna
        "TopBord",       # low voltage circuit panel(FR-4)
        "Suppott",      # spacer1 ｽｯﾎﾟﾄﾄ「」
        "Suppottt",     # spacer2
        "Extrude032",    # middle aluminium stick
        "Extrude074",    # long aluminium stick トレースしてこっちで作た
        "Extrude073",    # long stainless stick トレースしてこっちで作た
        "Extrude028",     # short aluminium stick
        "MP3-25",        # solar panel system MP3-25
        "wall"        # plastic wall
        ]
# [g]
objMass= [
        3.0,          # mount for Xmotor
        2.6,          # mount for Ymotor
        10,          # high voltage circuit panel(FR-4)
        18.3,          # wheel of Xmotor
        18.3,          # wheel of Ymotor
        18.3,          # wheel of Zmotor
        1.2,          # fixing nut for wheel X
        1.2,          # fixing nut for wheel Y
        1.2,          # fixing nut for wheel Z
        26.0,         # Xmotor
        26.0,         # Ymotor
        26.0,         # Zmotor
        28.5,          # 3S1P360RE 
        6.25,          # capacitor
        27.0,          # ESC of motors
        1,          # joint
        2,          # antenna
        14.5,          # low voltage circuit panel(FR-4)
        0.5,          # spacer1
        0.5,          # spacer2
        1.37,          # middle aluminium stick  almi pipe : 16/840 [g/mm]
        1.58,          # long aluminium stick
        2.0,           # long stainless stick ----------------- estimated ---------
        0.57,           # short aluminium stick
        5,        # solar panel system MP3-25
        12        # plastic wall
        ]
center = [0,0,60] # mm
# kg * mm^2
moi = [
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0
        ]
mass = 0.0
doc = App.ActiveDocument
for i in range(len(objName)):
    print(i)

    mass = mass + objMass[i]
    # g → kg
    # 除算は自動的に倍精度にconevrtさるらしい
    objMass[i] = objMass[i] / 1000.0

    obj = doc.getObjectsByLabel(objName[i])[0]
    # double
    density = objMass[i] / obj.Shape.Volume
    for j in range(len(obj.Shape.Solids)):
        # kg*mm*mm ( @ density=1 kg/mm/mm/mm)
        rawI = obj.Shape.Solids[j].MatrixOfInertia
        # mm
        cog = obj.Shape.Solids[j].CenterOfGravity
        d = [center[0] - cog.x, center[1] - cog.y, center[2] - cog.z]

        Ixx =  objMass[i] * (d[1]**2 + d[2]**2)
        Iyy =  objMass[i] * (d[0]**2 + d[2]**2)
        Izz =  objMass[i] * (d[0]**2 + d[1]**2)
        Ixy = -objMass[i] * d[0] * d[1]
        Ixz = -objMass[i] * d[0] * d[2]
        Iyz = -objMass[i] * d[1] * d[2]
        #if i == 3:
            

        moi[0] = moi[0] + density*rawI.A11 + Ixx
        moi[1] = moi[1] + density*rawI.A12 + Ixy
        moi[2] = moi[2] + density*rawI.A13 + Ixz
        moi[3] = moi[3] + density*rawI.A21 + Ixy
        moi[4] = moi[4] + density*rawI.A22 + Iyy
        moi[5] = moi[5] + density*rawI.A23 + Iyz
        moi[6] = moi[6] + density*rawI.A31 + Ixz
        moi[7] = moi[7] + density*rawI.A32 + Iyz
        moi[8] = moi[8] + density*rawI.A33 + Izz
        
for i in range(len(moi)):
    # kg → g
    moi[i] = moi[i]/1000

print(mass)
# print string for pasting into godot_tr/Main.cs
print()
print("    double Ixx = ",moi[0],";")
print("    double Iyy = ",moi[4],";")
print("    double Izz = ",moi[8],";")
print("    double Ixy = ",moi[1],";")
print("    double Ixz = ",moi[2],";")
print("    double Iyz = ",moi[5],";")
print()

    #scale_factor = FreeCAD.Vector(50, 50, 50)
    #center_point = FreeCAD.Vector(0, 0, 0)
    #Draft.scale(obj, scale_factor, center_point, copy=True)

# 処理のシステム的には同密度でオブジェクトを統合すべきなんだけど，
# 設計変更をkonnnyaku2924の側でやる場合再統合の必要が出てきて面倒なので，
# 向うのFCStdをそのまま使えるようにしてある 20260220
