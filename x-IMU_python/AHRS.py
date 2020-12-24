import numpy as np

class AHRS:
    def __init__(self,
                 SamplePeriod,
                 Kp,
                 Ki,
                 KpInit):
        # public
        self.SamplePeriod = SamplePeriod
        self.Quaternion = np.array([1, 0, 0, 0])
        self.Kp = Kp
        self.Ki = Ki
        self.KpInit = KpInit
        self.InitPeriod = 5

        # private
        self.q = np.array([1, 0, 0, 0])
        self.IntError = np.transpose( np.array([0, 0, 0]) )
        self.KpRamped = -1
    
    def quaternProd(self, a, b):

        #print("a.shape:{}".format(a.shape))
        #print("a:{}".format(a))
        #print("b:{}".format(b))

        ab = np.zeros_like(a)
        ab[0] = a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3]
        ab[1] = a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2]
        ab[2] = a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1]
        ab[3] = a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0]

        #print("ab:{}".format(ab))

        return ab


    def quaternConj(self, q):
        qConj = [q[0], -q[1], -q[2], -q[3]]
        return qConj

    def Reset(self):
        self.KpRamped = self.KpInit
        self.IntError = np.transpose( np.array([0, 0, 0]) )
        self.q = np.array([1, 0, 0, 0])

    def UpdateIMU(self, Gyroscope, Accelerometer):
        # Normalise accelerometer measurement
        if np.linalg.norm(Accelerometer) == 0:
            print("Accelerometer magnitude is zero.  Algorithm update aborted.")
            return False
        else:
            Accelerometer = Accelerometer / np.linalg.norm(Accelerometer)
        # Compute error between estimated and measured direction of gravity
        v = [
             2*(self.q[1]*self.q[3] - self.q[0]*self.q[2]),
             2*(self.q[0]*self.q[1] + self.q[2]*self.q[3]),
             self.q[0]**2 - self.q[1]**2 - self.q[2]**2 + self.q[3]**2
            ]
        error = np.cross(v, np.transpose(Accelerometer))
        #print("v:{}, Accelerometer:{}, error:{}".format(v, Accelerometer, error))
        self.IntError = self.IntError + error

        # Apply feedback terms
        Ref = Gyroscope - np.transpose(self.Kp*error + self.Ki*self.IntError)

        # Compute rate of change of quaternion
        pDot = 0.5 * self.quaternProd(self.q, [0, Ref[0], Ref[1], Ref[2]])
        self.q = self.q + pDot * self.SamplePeriod
        self.q = self.q / np.linalg.norm(self.q)

        # Store conjugate
        self.Quaternion = self.quaternConj(self.q)
        return True












