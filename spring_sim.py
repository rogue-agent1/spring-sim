#!/usr/bin/env python3
"""spring_sim - Spring-mass-damper system simulator."""
import sys, math

class Spring:
    def __init__(self, k, rest_length, damping=0):
        self.k = k
        self.rest_length = rest_length
        self.damping = damping

class Particle:
    def __init__(self, x, y, mass=1, fixed=False):
        self.x = x; self.y = y
        self.vx = 0; self.vy = 0
        self.fx = 0; self.fy = 0
        self.mass = mass
        self.fixed = fixed

class SpringSystem:
    def __init__(self):
        self.particles = []
        self.springs = []
    def add_particle(self, x, y, mass=1, fixed=False):
        p = Particle(x, y, mass, fixed)
        self.particles.append(p)
        return len(self.particles) - 1
    def add_spring(self, i, j, k=10, rest_length=None, damping=0.1):
        if rest_length is None:
            dx = self.particles[j].x - self.particles[i].x
            dy = self.particles[j].y - self.particles[i].y
            rest_length = math.sqrt(dx*dx + dy*dy)
        self.springs.append((i, j, Spring(k, rest_length, damping)))
    def step(self, dt, gravity=0):
        for p in self.particles:
            p.fx = 0
            p.fy = gravity * p.mass
        for i, j, spring in self.springs:
            a, b = self.particles[i], self.particles[j]
            dx = b.x - a.x
            dy = b.y - a.y
            dist = math.sqrt(dx*dx + dy*dy)
            if dist < 1e-10: continue
            force = spring.k * (dist - spring.rest_length)
            dvx = b.vx - a.vx
            dvy = b.vy - a.vy
            damp = spring.damping * (dvx*dx + dvy*dy) / dist
            fx = (force + damp) * dx / dist
            fy = (force + damp) * dy / dist
            a.fx += fx; a.fy += fy
            b.fx -= fx; b.fy -= fy
        for p in self.particles:
            if p.fixed: continue
            p.vx += p.fx / p.mass * dt
            p.vy += p.fy / p.mass * dt
            p.x += p.vx * dt
            p.y += p.vy * dt
    def energy(self):
        ke = sum(0.5 * p.mass * (p.vx**2 + p.vy**2) for p in self.particles)
        pe = 0
        for i, j, spring in self.springs:
            a, b = self.particles[i], self.particles[j]
            dx = b.x - a.x; dy = b.y - a.y
            dist = math.sqrt(dx*dx + dy*dy)
            pe += 0.5 * spring.k * (dist - spring.rest_length)**2
        return ke + pe

def test():
    sys = SpringSystem()
    sys.add_particle(0, 0, fixed=True)
    sys.add_particle(2, 0)
    sys.add_spring(0, 1, k=20, rest_length=1, damping=0.5)
    e0 = sys.energy()
    for _ in range(1000):
        sys.step(0.01)
    # with damping, energy should decrease
    e1 = sys.energy()
    assert e1 < e0
    # particle should be near rest length
    dist = math.sqrt(sys.particles[1].x**2 + sys.particles[1].y**2)
    assert abs(dist - 1.0) < 0.5
    print("OK: spring_sim")

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "test":
        test()
    else:
        print("Usage: spring_sim.py test")
