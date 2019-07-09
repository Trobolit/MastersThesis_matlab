Q = quaternion([pi/2,pi/4,0],'euler','ZYX','frame');

v1 = [0,1,0];

v2 = rotatepoint(Q,v1)