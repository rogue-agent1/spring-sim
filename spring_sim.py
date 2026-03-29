#!/usr/bin/env python3
"""Spring-mass-damper system with Verlet integration."""
import sys, math

class Spring:
    def __init__(self, n_masses=5, k=10, damping=0.1, rest_length=1.0):
        self.k, self.damp, self.rest = k, damping, rest_length
        self.x = [i*rest_length for i in range(n_masses)]
        self.v = [0.0]*n_masses; self.m = [1.0]*n_masses
        self.m[0] = float('inf')  # fixed anchor
    def step(self, dt=0.01):
        forces = [0.0]*len(self.x)
        for i in range(len(self.x)-1):
            dx = self.x[i+1]-self.x[i]; stretch = dx - self.rest
            f = self.k * stretch
            forces[i] += f; forces[i+1] -= f
        for i in range(len(self.x)):
            forces[i] -= self.damp * self.v[i]
        for i in range(1, len(self.x)):
            self.v[i] += forces[i]/self.m[i]*dt
            self.x[i] += self.v[i]*dt

def main():
    s = Spring(8, k=20, damping=0.5)
    s.x[-1] += 2.0  # stretch last mass
    for step in range(200):
        s.step(0.02)
        if step % 20 == 0:
            positions = " ".join(f"{x:.2f}" for x in s.x)
            energy = sum(0.5*m*v**2 for m,v in zip(s.m[1:],s.v[1:]))
            print(f"t={step*0.02:.1f}s KE={energy:.3f} pos=[{positions}]")

if __name__ == "__main__": main()
