import numpy as np

def quaternProd(a, b):
    ab = np.zeros_like(a)
    ab[:,0] = a[:,0]*b[:,0]-a[:,1]*b[:,1]-a[:,2]*b[:,2]-a[:,3]*b[:,3]
    ab[:,1] = a[:,0]*b[:,1]+a[:,1]*b[:,0]+a[:,2]*b[:,3]-a[:,3]*b[:,2]
    ab[:,2] = a[:,0]*b[:,2]-a[:,1]*b[:,3]+a[:,2]*b[:,0]+a[:,3]*b[:,1]
    ab[:,3] = a[:,0]*b[:,3]+a[:,1]*b[:,2]-a[:,2]*b[:,1]+a[:,3]*b[:,0]
    return ab

def quaternConj(q):
    qConj = np.column_stack((q[:,0], -q[:,1], -q[:,2], -q[:,3]))
    return qConj

def quaternRotate(v, q):
    print("v.shape:{}".format(v.shape))
    shape = v.shape
    row = shape[0]
    col = shape[1]
    q1 = quaternProd(q, np.column_stack((np.zeros((row,1)),v)))
    q2 = quaternConj(q)
    v0XYZ = quaternProd(q1,q2)
    v = v0XYZ[:,1:]
    print("v0XYZ:{}".format(v0XYZ))
    print("v:{}".format(v))

    return v

