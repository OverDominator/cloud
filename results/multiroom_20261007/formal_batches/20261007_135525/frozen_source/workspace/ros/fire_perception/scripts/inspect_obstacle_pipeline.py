"""Read-only eight-second cloud snapshot; no navigation commands are sent."""
import json
import math
import time
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from visualization_msgs.msg import MarkerArray


def main():
    rospy.init_node('inspect_obstacle_pipeline', anonymous=True)
    latest = {}
    topics = ['/registered_scan', '/terrain_map', '/terrain_map_ext',
              '/terrain_map_ext_with_fire', '/FAR_obs_debug', '/FAR_free_debug']
    def save(msg, topic):
        latest[topic] = msg
    subs = [rospy.Subscriber(t, PointCloud2, save, callback_args=t, queue_size=1)
            for t in topics]
    subs.append(rospy.Subscriber('/state_estimation', Odometry, save,
                                callback_args='odom', queue_size=1))
    marker_topics = ['/viz_contour_topic', '/viz_poly_topic', '/viz_node_topic_array']
    subs.extend(rospy.Subscriber(t, MarkerArray, save, callback_args=t, queue_size=1)
                for t in marker_topics)
    time.sleep(8)
    odom = latest.get('odom')
    if odom is None:
        print('No fresh odometry received')
        return
    p, q = odom.pose.pose.position, odom.pose.pose.orientation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    print('pose', p.x, p.y, p.z, 'yaw', yaw)
    for t in marker_topics:
        msg = latest.get(t)
        if msg is None:
            print(t, 'NO MESSAGE')
            continue
        for m in msg.markers:
            print(json.dumps(dict(topic=t, ns=m.ns, action=m.action, type=m.type,
                frame=m.header.frame_id, pose=[m.pose.position.x,m.pose.position.y,m.pose.position.z],
                points=[[round(v.x,2),round(v.y,2),round(v.z,2)] for v in m.points])))
    for t in topics:
        msg = latest.get(t)
        if msg is None:
            print(t, 'NO MESSAGE')
            continue
        fields = [f.name for f in msg.fields]
        data = list(point_cloud2.read_points(msg, field_names=['x','y','z'] +
                    (['intensity'] if 'intensity' in fields else []), skip_nans=True))
        if t == '/FAR_obs_debug':
            import cv2
            import numpy as np
            img = np.zeros((401,401), np.uint8)
            for v in data:
                r, c = 200+round((v[0]-p.x)/.15), 200+round((v[1]-p.y)/.15)
                if 1 <= r < 400 and 1 <= c < 400:
                    img[r-1:r+2,c-1:c+2] = 255
            img = cv2.resize(img, (1203,1203), interpolation=cv2.INTER_LINEAR)
            img = cv2.boxFilter(img, -1, (20,20), normalize=False)
            contours, hierarchy = cv2.findContours(img,cv2.RETR_TREE,cv2.CHAIN_APPROX_TC89_L1)
            for contour in contours:
                approx = cv2.approxPolyDP(contour,4.5,True)
                print('REPLAY_APPROX', [[round(float(a[0][1])/20-30.05+p.x,2),
                    round(float(a[0][0])/20-30.05+p.y,2)] for a in approx])
        front = []
        if msg.header.frame_id.lstrip('/') == odom.header.frame_id.lstrip('/'):
            for v in data:
                dx, dy = v[0]-p.x, v[1]-p.y
                x = dx*math.cos(yaw)+dy*math.sin(yaw)
                y = -dx*math.sin(yaw)+dy*math.cos(yaw)
                if 0 < x < 8 and abs(y) < 3:
                    front.append(v)
        print(json.dumps(dict(topic=t, frame=msg.header.frame_id,
            age=(rospy.Time.now()-msg.header.stamp).to_sec(), total=len(data),
            front_count=len(front),
            front_z=[min(v[2] for v in front),max(v[2] for v in front)] if front else [],
            front_obstacle_intensity=sum(v[3]>0.2 for v in front if len(v)>3))))


if __name__ == '__main__':
    main()
