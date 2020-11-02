Author Guillaume WALCK gwalck@techfak.uni-bielefeld.de

work done during the HANDLE project in 2011 (inertia tensor moved to joint axis)
reworked 2015 (inertia tensor at COM)

Overview
========

Arm inertia estimated from bounding box or bounding cylinders.
mass estimated from volume of bounding boxes times density of material
computation done with octave


Octave functions
================

```octave
function [ix,iy,iz] = inertiabox(sx,sy,sz,mass)

ix = 1/12.0*mass*(sy*sy+sz*sz);
iy = 1/12.0*mass*(sx*sx+sz*sz);
iz = 1/12.0*mass*(sy*sy+sx*sx);
end
```

```octave
function [ix,iy,iz] = inertiacyl(radius,length,mass)
% along z

iz = mass* radius*radius/2;
ix = 1/12.0* mass * (3*radius*radius+length*length);
iy = ix;
end
```

Inertia Tensors
===============

```octave
base aluminum m=400g
inertiabox(0.145,0.145,0.012,0.4)
Ix =    7.0563e-04
Iy =    7.0563e-04
Iz =  0.0014017
```


```octave
hand_support_plate_motor aluminum  m=750g
inertiacyl(0.0675,0.006,0.75)
Ix =    8.5655e-04
Iy =    8.5655e-04
Iz =  0.0017086
```

```octave
hand_support_plate_muscle aluminum  m=750g
inertiacyl(0.07,0.006,0.75)
Ix =    9.2100e-04
Iy =    9.2100e-04
Iz =  0.0018375
```

```octave
trunk various material m=10kg
inertiacyl(0.17/2,0.580,10.0)
Ix =  0.29840
Iy =  0.29840
Iz =  0.036125
```

```octave
upperarm various material m=2kg
inertiacyl(0.15/2,0.500,2.0)
Ix =  0.044479
Iy =  0.044479
Iz =  0.0056250
```

```octave
lower_arm various material m=1.7kg
inertiabox(0.080,0.078,0.07,1.7)
Ix =  0.0015561
Iy =  0.0016008
Iz =  0.0017686
```
