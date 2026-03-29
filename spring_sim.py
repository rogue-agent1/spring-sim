#!/usr/bin/env python3
"""Spring-mass system simulator."""
import math

class Spring:
    def __init__(self, k=1.0, rest_length=1.0, damping=0.1):
        self.k = k
        self.rest_length = rest_length
        self.damping = damping

class Mass:
    def __init__(self, x=0, v=0, m=1.0, fixed=False):
        self.x, self.v, self.m, self.fixed = x, v, m, fixed

class SpringSystem:
    def __init__(self):
        self.masses = []
        self.springs = []

    def add_mass(self, **kw):
        m = Mass(**kw)
        self.masses.append(m)
        return m

    def connect(self, i, j, **kw):
        s = Spring(**kw)
        self.springs.append((i, j, s))
        return s

    def step(self, dt):
        forces = [0.0] * len(self.masses)
        for i, j, s in self.springs:
            dx = self.masses[j].x - self.masses[i].x
            dist = abs(dx)
            direction = 1 if dx >= 0 else -1
            stretch = dist - s.rest_length
            f = s.k * stretch * direction
            dv = self.masses[j].v - self.masses[i].v
            f += s.damping * dv * direction
            forces[i] += f
            forces[j] -= f
        for idx, m in enumerate(self.masses):
            if not m.fixed:
                a = forces[idx] / m.m
                m.v += a * dt
                m.x += m.v * dt

    def energy(self):
        ke = sum(0.5 * m.m * m.v**2 for m in self.masses)
        pe = 0
        for i, j, s in self.springs:
            dx = abs(self.masses[j].x - self.masses[i].x)
            pe += 0.5 * s.k * (dx - s.rest_length)**2
        return ke + pe

if __name__ == "__main__":
    sys = SpringSystem()
    sys.add_mass(x=0, fixed=True)
    sys.add_mass(x=2, m=1)
    sys.connect(0, 1, k=10, rest_length=1, damping=0.5)
    for _ in range(100):
        sys.step(0.01)
    print(f"Position: {sys.masses[1].x:.3f}, Velocity: {sys.masses[1].v:.3f}")

def test():
    s = SpringSystem()
    s.add_mass(x=0, fixed=True)
    s.add_mass(x=2, m=1)
    s.connect(0, 1, k=10, rest_length=1, damping=0)
    e0 = s.energy()
    for _ in range(1000):
        s.step(0.001)
    e1 = s.energy()
    # Energy should be roughly conserved (small dt)
    assert abs(e1 - e0) / e0 < 0.05
    # With damping, energy decreases
    s2 = SpringSystem()
    s2.add_mass(x=0, fixed=True)
    s2.add_mass(x=3, m=1)
    s2.connect(0, 1, k=10, rest_length=1, damping=2)
    e_start = s2.energy()
    for _ in range(5000):
        s2.step(0.001)
    assert s2.energy() < e_start * 0.5
    # Should settle near rest length
    assert abs(s2.masses[1].x - 1) < 0.5
    print("  spring_sim: ALL TESTS PASSED")
