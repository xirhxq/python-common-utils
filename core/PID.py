# core/PID.py

class PID:

    def __init__(self, kp, ki, kd, timeFunc, intMax=None, intMin=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.timeFunc = timeFunc

        self.__intMax = intMax
        self.__intMin = intMin      
        self.__errIntegral = 0.0
        self.__errPrevious = 0.0
        self.__timePrevious = None

    def compute(self, errProportion, errPhysicalDiff=None):
        currentTime = self.timeFunc()
        dt = 0.0 if self.__timePrevious is None else currentTime - self.__timePrevious
        
        self.__errIntegral += errProportion * dt
        if self.__intMax is not None and self.__errIntegral > self.__intMax:
            self.__errIntegral = self.__intMax
        elif self.__intMin is not None and self.__errIntegral < self.__intMin:
            self.__errIntegral = self.__intMin

        errDiff = (errProportion - self.__errPrevious) / dt if dt != 0.0 else 0.0
        output = self.kp * errProportion + self.ki * self.__errIntegral + self.kd * errDiff
        self.__errPrevious = errProportion
        self.__timePrevious = currentTime

        return output

    def clearIntResult(self):
        self.__errIntegral = 0