#!/usr/bin/env python3
"""Spring Simulation - Mass-spring system with damping and multiple bodies."""
import sys, math

class Body:
    def __init__(self, x, y, mass=1.0, fixed=False):
        self.x=x;self.y=y;self.vx=0;self.vy=0;self.mass=mass;self.fixed=fixed

class Spring:
    def __init__(self, a, b, k=50.0, rest_length=None, damping=0.5):
        self.a=a;self.b=b;self.k=k;self.damping=damping
        self.rest = rest_length or math.sqrt((a.x-b.x)**2+(a.y-b.y)**2)

def simulate(bodies, springs, steps=200, dt=0.01, gravity=9.8):
    history = []
    for _ in range(steps):
        forces = {id(b): [0, gravity*b.mass] for b in bodies}
        for s in springs:
            dx=s.b.x-s.a.x;dy=s.b.y-s.a.y
            dist=math.sqrt(dx*dx+dy*dy) or 0.001
            f=(dist-s.rest)*s.k;fx=f*dx/dist;fy=f*dy/dist
            dvx=s.b.vx-s.a.vx;dvy=s.b.vy-s.a.vy
            dampf=s.damping*(dvx*dx/dist+dvy*dy/dist)
            forces[id(s.a)][0]+=fx+dampf*dx/dist;forces[id(s.a)][1]+=fy+dampf*dy/dist
            forces[id(s.b)][0]-=fx+dampf*dx/dist;forces[id(s.b)][1]-=fy+dampf*dy/dist
        for b in bodies:
            if b.fixed: continue
            b.vx+=forces[id(b)][0]/b.mass*dt;b.vy+=forces[id(b)][1]/b.mass*dt
            b.x+=b.vx*dt;b.y+=b.vy*dt
        history.append([(b.x,b.y) for b in bodies])
    return history

def main():
    bodies = [Body(0,0,fixed=True), Body(2,0), Body(4,0), Body(6,0)]
    springs = [Spring(bodies[i],bodies[i+1],k=100,damping=2) for i in range(3)]
    bodies[3].vy = 5
    history = simulate(bodies, springs, steps=300)
    print("=== Spring Simulation ===\n")
    for i in [0,50,100,150,200,250,299]:
        if i < len(history):
            pos = history[i]
            print(f"  t={i*0.01:.2f}s: " + " ".join(f"({x:.1f},{y:.1f})" for x,y in pos))

if __name__ == "__main__":
    main()
