"""Three-dimensional closest-approach prediction and ordered altitude deconfliction."""
import math


def closest_approach(pa, va, pb, vb, horizon=6.0):
    r=[b-a for a,b in zip(pa,pb)]
    v=[b-a for a,b in zip(va,vb)]
    vv=sum(x*x for x in v)
    t=max(0.0,min(horizon,-sum(x*y for x,y in zip(r,v))/vv)) if vv>1e-9 else 0.0
    return math.sqrt(sum(x*x for x in r)), math.sqrt(sum((x+t*y)**2 for x,y in zip(r,v))), t


def avoidance_targets(base, conflicts, cruise=18.0, minimum_gap=12.0):
    """Higher numbered aircraft yields upward; lower one yields downward.

    Project the entire stack onto ordered, separated altitude targets so a
    response to one neighbour cannot command a collision with another.
    """
    offsets=[0.0]*len(base); speed=[cruise]*len(base)
    for i,j in sorted(conflicts):
        offsets[i]=min(offsets[i],-6.0);offsets[j]=max(offsets[j],6.0)
        speed[i]=min(speed[i],cruise-1.0);speed[j]=max(speed[j],cruise+1.0)
    height=[max(30.0,b+d) for b,d in zip(base,offsets)]
    for i in range(1,len(height)):
        height[i]=max(height[i],height[i-1]+minimum_gap)
    return height,speed
