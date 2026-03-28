#!/usr/bin/env python3
"""spring_sim - Spring-mass system simulator."""
import argparse, math, sys, time

class Mass:
    def __init__(self, x, y, m=1, fixed=False):
        self.x=x; self.y=y; self.vx=0; self.vy=0; self.m=m; self.fixed=fixed

class Spring:
    def __init__(self, a, b, k=10, rest_len=None, damping=0.1):
        self.a=a; self.b=b; self.k=k; self.damping=damping
        self.rest = rest_len or math.sqrt((a.x-b.x)**2+(a.y-b.y)**2)

def simulate(masses, springs, dt=0.01, gravity=9.8):
    for m in masses:
        if m.fixed: continue
        m.vy += gravity * dt
    for s in springs:
        dx = s.b.x-s.a.x; dy = s.b.y-s.a.y
        dist = math.sqrt(dx*dx+dy*dy) or 0.001
        force = s.k * (dist - s.rest)
        fx = force * dx/dist; fy = force * dy/dist
        # Damping
        dvx = s.b.vx-s.a.vx; dvy = s.b.vy-s.a.vy
        fx += s.damping * dvx; fy += s.damping * dvy
        if not s.a.fixed: s.a.vx += fx/s.a.m*dt; s.a.vy += fy/s.a.m*dt
        if not s.b.fixed: s.b.vx -= fx/s.b.m*dt; s.b.vy -= fy/s.b.m*dt
    for m in masses:
        if m.fixed: continue
        m.x += m.vx*dt; m.y += m.vy*dt

def render(masses, springs, w=60, h=30, scale=2):
    grid = [["."]*w for _ in range(h)]
    for s in springs:
        x1,y1 = int(s.a.x*scale+w//2), int(s.a.y*scale)
        x2,y2 = int(s.b.x*scale+w//2), int(s.b.y*scale)
        steps = max(abs(x2-x1), abs(y2-y1), 1)
        for t in range(steps+1):
            px = x1 + (x2-x1)*t//steps; py = y1 + (y2-y1)*t//steps
            if 0<=px<w and 0<=py<h: grid[py][px] = "-"
    for i, m in enumerate(masses):
        px, py = int(m.x*scale+w//2), int(m.y*scale)
        ch = "●" if m.fixed else "○"
        if 0<=px<w and 0<=py<h: grid[py][px] = ch
    return "\n".join("".join(row) for row in grid)

def main():
    p = argparse.ArgumentParser(description="Spring-mass simulator")
    p.add_argument("--demo", choices=["pendulum","chain","cloth"], default="chain")
    p.add_argument("-s","--steps", type=int, default=300)
    a = p.parse_args()
    if a.demo == "pendulum":
        m0 = Mass(0, 0, fixed=True); m1 = Mass(5, 5, m=2)
        masses = [m0, m1]; springs = [Spring(m0, m1, k=20, rest_len=5)]
    elif a.demo == "chain":
        masses = [Mass(0, 0, fixed=True)]
        for i in range(1, 8): masses.append(Mass(0, i*2))
        springs = [Spring(masses[i], masses[i+1], k=50) for i in range(len(masses)-1)]
        masses[-1].vx = 10
    else:  # cloth
        n = 5; masses = []; springs = []
        for y in range(n):
            for x in range(n):
                m = Mass(x*3-6, y*3, fixed=(y==0))
                masses.append(m)
        for y in range(n):
            for x in range(n):
                idx = y*n+x
                if x < n-1: springs.append(Spring(masses[idx], masses[idx+1], k=30))
                if y < n-1: springs.append(Spring(masses[idx], masses[idx+n], k=30))
    for s in range(a.steps):
        for _ in range(5): simulate(masses, springs, dt=0.005)
        if s % 3 == 0:
            sys.stdout.write(f"\033[2J\033[H{a.demo} step {s}\n")
            print(render(masses, springs))
            time.sleep(0.03)

if __name__ == "__main__": main()
