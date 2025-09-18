import matplotlib.pyplot as plt
import numpy as np


class inert:
    """docstring for i"""

    def __init__(self, K, T, dt=0.001):
        self.K = K
        self.T = T
        self.y = 0
        self.dt = dt

    def step(self, x):
        self.y = self.y + (x * self.K - self.y) * self.dt / self.T
        return self.y


class intgr360:
    """docstring for i"""

    def __init__(self, K, T, dt=0.001):
        self.K = K
        self.T = T
        self.y = 0
        self.dt = dt

    def step(self, x):
        self.y = self.y + x * self.K * self.dt / self.T
        if self.y >= 360.0:
            self.y -= 360.0
        if self.y < 0.0:
            self.y += 360.0
        return self.y


class intgr:
    """docstring for i"""

    def __init__(self, K, T, dt=0.001):
        self.K = K
        self.T = T
        self.y = 0
        self.dt = dt

    def step(self, x):
        self.y = self.y + x * self.K * self.dt / self.T
        return self.y


class Uabc:
    """docstring for ClassName"""

    def __init__(self, dt=0.001):
        self.Ua = 0.0
        self.Ub = 0.0
        self.Uc = 0.0
        self.tms = 0
        self.phz = 0
        self.dt = dt

    def step(self, fhz, Uv=220.4):
        self.tms += 1000 * self.dt
        self.phz += 360.0 * fhz * self.dt
        if self.phz >= 360.0:
            self.phz = self.phz - 360.0
        if self.phz < 60:
            self.Ua = 1.0 * Uv
            self.Ub = -1.0 * Uv
            self.Uc = 0.0
        elif self.phz < 120:
            self.Ua = 1.0 * Uv
            self.Ub = 0
            self.Uc = -1.0 * Uv
        elif self.phz < 180:
            self.Ua = 0
            self.Ub = 1.0 * Uv
            self.Uc = -1.0 * Uv
        elif self.phz < 240:
            self.Ua = -1.0 * Uv
            self.Ub = 1.0 * Uv
            self.Uc = 0
        elif self.phz < 300:
            self.Ua = -1.0 * Uv
            self.Ub = 0
            self.Uc = 1.0 * Uv
        else:
            self.Ua = 0
            self.Ub = -1.0 * Uv
            self.Uc = 1.0 * Uv


class Eabc:
    """docstring for Eabc"""

    def __init__(self):
        self.Ea = 0
        self.Eb = 0
        self.Ec = 0
        self.phz = 0

    def step(self, phz):
        self.phz = phz
        if self.phz >= 360.0:
            self.phz = self.phz - 360.0
        if self.phz < 60:
            self.Ea = 1.0
            self.Eb = -1.0
            self.Ec = 1.0 - (self.phz - 0) / 30
        elif self.phz < 120:
            self.Ea = 1.0
            self.Eb = -1.0 + (self.phz - 60) / 30
            self.Ec = -1.0
        elif self.phz < 180:
            self.Ea = 1.0 - (self.phz - 120) / 30
            self.Eb = 1.0
            self.Ec = -1.0
        elif self.phz < 240:
            self.Ea = -1.0
            self.Eb = 1.0
            self.Ec = -1.0 + (self.phz - 180) / 30
        elif self.phz < 300:
            self.Ea = -1.0
            self.Eb = 1.0 - (self.phz - 240) / 30
            self.Ec = 1.0
        else:
            self.Ea = -1.0 + (self.phz - 300) / 30
            self.Eb = -1.0
            self.Ec = 1.0


class PhCurr:
    """docstring for ClassName"""

    def __init__(self, Kfp):
        self.Kfp = Kfp
        self.y = 0

    def step(self, u1, u2, u3, e1, e2, e3, wr):
        self.y = 2 * u1 - u2 - u3 + self.Kfp * wr * (e2 + e3 - e1 * 2)
        return self.y


class BLDCmotor:
    """docstring for ClassName"""

    def __init__(self, Rs=0.2491, Ls=0.005, Zp=8, Wf=0.0001927, dt=0.001):
        self.Kfp = Zp * Wf
        self.Zp = Zp
        self.T = Ls / Rs
        self.K = 1 / (3 * Rs)
        self.inertA = inert(self.K, self.T, dt)
        self.inertB = inert(self.K, self.T, dt)
        self.inertC = inert(self.K, self.T, dt)
        self.intgrQe = intgr360(180, 3.14159265359, dt)  # чтобы измерялось в град / сек
        self.Eabc1 = Eabc()

    def PhCurr(self, u1, u2, u3, e1, e2, e3, wr):
        return 2 * u1 - u2 - u3 + self.Kfp * wr * (e2 + e3 - e1 * 2)

    def step(self, Ua, Ub, Uc, wr):
        self.Eabc1.step(self.intgrQe.y)
        self.inertA.step(
            self.PhCurr(Ua, Ub, Uc, self.Eabc1.Ea, self.Eabc1.Eb, self.Eabc1.Ec, wr)
        )
        self.inertB.step(
            self.PhCurr(Ub, Uc, Ua, self.Eabc1.Eb, self.Eabc1.Ec, self.Eabc1.Ea, wr)
        )
        self.inertC.step(
            self.PhCurr(Uc, Ua, Ub, self.Eabc1.Ec, self.Eabc1.Ea, self.Eabc1.Eb, wr)
        )
        self.Me = self.Kfp * (
            self.Eabc1.Ea * self.inertA.y
            + self.Eabc1.Eb * self.inertB.y
            + self.Eabc1.Ec * self.inertC.y
        )
        self.intgrQe.step(wr * self.Zp)
        return self.Me


sfHz = 100  # fHz in 0 to 20
Tfl = 0.2  # полное время моделирования
dt = 0.00001  # время дискретизации
smooz = 1
"""
Zp = 6			# количество пар полюсов на роторе
Kfp = 0.1 	# Kfp = Zp * Wf		example Wf=0.4 Zp=6
k = 13				# W(s) 	k					k=1/(3*Rf)
T = 0.01		#			(1+Ts) 			T=Lf/Rf
"""
motor = BLDCmotor(0.2491, 0.005, 8, 0.0001927, dt)
wr = 0
Ugen = Uabc(dt)
Kj = 50000
Tj = 0.0005
InertWr = inert(Kj, Tj, dt)
altms = int(Tfl / dt)
InerMe = inert(1, 0.01, dt)
InerDe = inert(1, 0.01, dt)
s1 = []
s2 = []
s3 = []
motor.intgrQe.y = 100.12  # угол, от которого стартуем

for x in range(0, altms):
    if smooz < 1:
        fHz = sfHz
    else:
        if x < 10000:
            fHz = sfHz * x / 10000
        else:
            fHz = sfHz
        if fHz < 5:
            fHz = 5
    Ugen.step(fHz, 7.4)
    # Ugen.Ua = 0; Ugen.Ub = 10; Ugen.Uc = -10;
    motor.step(Ugen.Ua, Ugen.Ub, Ugen.Uc, wr)
    InertWr.step(motor.Me)
    InerMe.step(motor.Me)
    InerDe.step((motor.Me - InerMe.y) * (motor.Me - InerMe.y))
    wr = InertWr.y
    print(Ugen.tms, "\t", Ugen.Ua, "\t", Ugen.Ub, "\t", Ugen.Uc, "\t", motor.intgrQe.y)
    # blue
    # s1.append(0.001*motor.Zp*wr/3.14159265359/2) #wr/3.14159265359/2*Zp         #Ugen.Ua
    s1.append(motor.Me - InerMe.y)
    # yelld
    # s2.append(motor.Me*100)   # Me*8
    s2.append(motor.Me)
    # green
    # s3.append(motor.intgrQe.y/360)
    s3.append(InerDe.y * 500)

t = np.arange(0, altms, 1)
fig, axs = plt.subplots(1, 1, figsize=(160 / 25.4, 80 / 25.4))
plt.rc("font", family="Times New Roman", size=12)
plt.plot(t, s3, label="Дисперция")
plt.plot(t, s2, label="Электромагнитный момент")
plt.plot(t, s1, label="Шум, поступающий на двигатель")

plt.title("Моделирование БДПТ при быстром пуске", family="Times New Roman", fontsize=12)
plt.legend()
plt.xlabel("Время(mc)")
plt.ylabel("Амплитуда")
plt.grid(True)
plt.show()
