#!/usr/bin/env python3
"""Read-only capture of obstacle cloud clusters in Unity world coordinates."""
import json
import math
import time
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry

rospy.init_node('obstacle_alignment_probe', anonymous=True)
topics = ['/registered_scan', '/terrain_map_ext', '/added_obstacles', '/FAR_obs_debug']
received = {}
def capture(msg, topic):
    received[topic] = msg
subs = [rospy.Subscriber(t, PointCloud2, capture, callback_args=t, queue_size=1) for t in topics]
poses = []
subs.append(rospy.Subscriber('/state_estimation', Odometry, lambda m: poses.append(m), queue_size=1))
deadline = time.monotonic() + 12
while len(received) < len(topics) and time.monotonic() < deadline:
    time.sleep(.1)
for topic, msg in list(received.items()):
    pts = list(pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))
    if topic == '/FAR_obs_debug' and poses:
        p = poses[-1].pose.pose.position
        nearby = sorted((math.hypot(x-p.x,y-p.y), x,y,z) for x,y,z in pts
                        if abs(z-p.z) < 2.0)[:5]
        print(json.dumps(dict(robot_ros=[p.x,p.y,p.z], nearest_obstacles=nearby)), flush=True)
    pts = list({(round(-y, 1), round(x, 1)) for x,y,z in pts if .2 < z < 2.8})
    groups = []
    if pts:
        buckets = {}
        for i, p in enumerate(pts):
            buckets.setdefault((math.floor(p[0]/.65), math.floor(p[1]/.65)), []).append(i)
        unseen = set(range(len(pts)))
        while unseen:
            stack = [unseen.pop()]
            group = []
            while stack:
                i = stack.pop()
                group.append(pts[i])
                x,y = pts[i]
                bx,by = math.floor(x/.65), math.floor(y/.65)
                neighbors = {j for dx in (-1,0,1) for dy in (-1,0,1)
                             for j in buckets.get((bx+dx,by+dy), [])
                             if j in unseen and (pts[j][0]-x)**2+(pts[j][1]-y)**2 <= .65**2}
                unseen.difference_update(neighbors)
                stack.extend(neighbors)
            if len(group) >= 8:
                groups.append(dict(n=len(group), center=[round(sum(p[k] for p in group)/len(group),2) for k in (0,1)], bounds=[[min(p[k] for p in group),max(p[k] for p in group)] for k in (0,1)]))
    print(json.dumps(dict(topic=topic, frame=msg.header.frame_id, clusters=groups)), flush=True)
