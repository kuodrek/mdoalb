class Aviao_geometria():
    def __init__(self, xcg, xtdp, xmotor, lt, afuse, hfuse, lfuse) -> None:
        self.xcg = xcg
        self.xtdp = xtdp
        self.xmotor = xmotor
        self.lt = lt
        self.afuse = afuse
        self.hfuse = hfuse
        self.lfuse = lfuse

class Aviao_desempenho():
    def __init__(self, MTOW, V_min, V_max, V_manobra, V_cruise, V_stall) -> None:
        self.mtow = MTOW
        self.v_min = V_min
        self.v_max = V_max
        self.v_man = V_manobra
        self.v_cruise = V_cruise
        self.v_stall = V_stall

class Cond_voo():
    def __init__(self, rho, g, mi) -> None:
        self.rho = rho
        self.g = g
        self.mi = mi