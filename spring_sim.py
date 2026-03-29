#!/usr/bin/env python3
"""spring_sim - Damped spring simulation."""
import sys,argparse,json,math
def simulate(mass=1.0,k=10.0,damping=0.1,x0=1.0,v0=0.0,dt=0.01,steps=1000):
    x,v=x0,v0;history=[]
    for i in range(steps):
        a=(-k*x-damping*v)/mass
        v+=a*dt;x+=v*dt
        if i%(steps//50)==0:
            history.append({"t":round(i*dt,4),"x":round(x,6),"v":round(v,6),"energy":round(0.5*mass*v**2+0.5*k*x**2,6)})
    omega=math.sqrt(k/mass);period=2*math.pi/omega
    return history,period
def main():
    p=argparse.ArgumentParser(description="Spring simulator")
    p.add_argument("--mass",type=float,default=1.0);p.add_argument("--k",type=float,default=10.0)
    p.add_argument("--damping",type=float,default=0.1);p.add_argument("--x0",type=float,default=1.0)
    p.add_argument("--steps",type=int,default=2000)
    args=p.parse_args()
    history,period=simulate(args.mass,args.k,args.damping,args.x0,steps=args.steps)
    print(json.dumps({"mass":args.mass,"k":args.k,"damping":args.damping,"natural_period":round(period,4),"natural_freq":round(1/period,4),"history":history},indent=2))
if __name__=="__main__":main()
