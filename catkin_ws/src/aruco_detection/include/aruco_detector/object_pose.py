class ObjectPose:

    def __init__(self, x: float = 0, y: float = 0, z: float = 0,
                  roll: float = 0, pitch: float = 0, yaw: float = 0) -> None:
        assert isinstance(x, (int, float))
        assert isinstance(y, (int, float))
        assert isinstance(z, (int, float))
        assert isinstance(roll, (int, float))
        assert isinstance(pitch, (int, float))
        assert isinstance(yaw, (int, float))
        self._X = x
        self._Y = y
        self._Z = z
        self._Roll = roll
        self._Pitch = pitch
        self._Yaw = yaw


    def getXYZ(self):
        return self._X, self._Y, self._Z
    
    
    def getRPY(self):
        return self._Roll, self._Pitch, self._Yaw
    
    
    def setXYZ(self, x: float, y: float, z: float):
        assert isinstance(x, (int, float))
        assert isinstance(y, (int, float))
        assert isinstance(z, (int, float))
        self._X = x
        self._Y = y
        self._Z = z


    def setRPY(self, roll: float, pitch: float, yaw: float):
        assert isinstance(roll, (int, float))
        assert isinstance(pitch, (int, float))
        assert isinstance(yaw, (int, float))
        self._Roll = roll
        self._Pitch = pitch
        self._Yaw = yaw


    @property
    def X(self):
        return self._X
    
    
    @X.setter
    def X(self, x):
        assert isinstance(x, (int, float))
        self._X = x


    @property
    def Y(self):
        return self._Y
    
    
    @Y.setter
    def Y(self, y):
        assert isinstance(y, (int, float))
        self._Y = y


    @property
    def Z(self):
        return self._Z
    
    
    @Z.setter
    def Z(self, z):
        assert isinstance(z, (int, float))
        self._Z = z


    @property
    def Roll(self):
        return self._Roll
    
    @Roll.setter
    def Roll(self, roll: float):
        assert isinstance(roll, (int, float))
        self._Roll = roll

    
    @property
    def Pitch(self):
        return self._Pitch
    
    
    @Pitch.setter
    def Pitch(self, pitch: float):
        assert isinstance(pitch, (int, float))
        self._Pitch = pitch


    @property
    def Yaw(self):
        return self._Yaw
    
    
    @Yaw.setter
    def Yaw(self, yaw: float):
        assert isinstance(yaw, (int, float))
        self._Yaw = yaw
        