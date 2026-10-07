"""Short observation-space legs; no scene geometry or target ground truth."""
import heapq
import math


def observed_route(start, goal, points, segment_clear, max_leg=12.0):
    nodes=list(dict.fromkeys([tuple(start), tuple(goal)]+[tuple(p) for p in points]))
    if len(nodes)<2:
        return [tuple(goal)]
    frontier=[(math.dist(start,goal),0.0,0)]
    costs={0:0.0}
    parents={}
    tested={}
    while frontier:
        _,cost,i=heapq.heappop(frontier)
        if cost>costs[i]+1e-8:
            continue
        if i==1:
            route=[]
            while i:
                route.append(nodes[i]);i=parents[i]
            return list(reversed(route))
        for j,p in enumerate(nodes):
            distance=math.dist(nodes[i],p)
            if j==i or distance>max_leg or cost+distance>=costs.get(j,float('inf')):
                continue
            key=tuple(sorted((i,j)))
            if key not in tested:
                tested[key]=segment_clear(nodes[i],p)
            if not tested[key]:
                continue
            costs[j]=cost+distance
            parents[j]=i
            heapq.heappush(frontier,(costs[j]+math.dist(p,goal),costs[j],j))
    return []
