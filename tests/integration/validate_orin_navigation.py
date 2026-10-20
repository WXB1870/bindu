"""Synthetic ROS sensor/TF preflight. No base driver or motion publication."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid


def peer(namespace, case):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
    rclpy.init(args=['--ros-args','-r','/tf:='+namespace+'/tf','-r','/tf_static:='+namespace+'/tf_static'])
    node = rclpy.create_node('sensor_fixture',namespace=namespace)
    scan_pub = node.create_publisher(LaserScan,namespace+'/scan',qos_profile_sensor_data)
    odom_pub = node.create_publisher(Odometry,namespace+'/odom',qos_profile_sensor_data)
    tf = TransformBroadcaster(node); static = StaticTransformBroadcaster(node)
    fixed = TransformStamped(); fixed.header.frame_id='base_link'; fixed.child_frame_id='laser'; fixed.transform.rotation.w=1.
    static.sendTransform(fixed)
    duplicate = rclpy.create_node('duplicate',namespace=namespace)
    duplicate_tf = TransformBroadcaster(duplicate) if case=='duplicate_tf' else None
    started = time.monotonic(); frozen = node.get_clock().now().to_msg()
    def publish():
        elapsed = time.monotonic()-started
        stamp = node.get_clock().now().to_msg()
        if case=='frozen': stamp=frozen
        if case=='old_stamp': stamp.sec-=5
        scan = LaserScan(); scan.header.stamp=stamp; scan.header.frame_id='wrong' if case=='wrong_frame' else 'laser'
        scan.angle_min=-3.14159; scan.angle_increment=6.28318/36; scan.angle_max=scan.angle_min+35*scan.angle_increment
        scan.range_min=.1; scan.range_max=10.; scan.ranges=[2.]*36
        if case!='scan_loss' or elapsed<.5: scan_pub.publish(scan)
        odom = Odometry(); odom.header.stamp=stamp; odom.header.frame_id='odom'; odom.child_frame_id='base_link'
        odom.pose.pose.orientation.w=1.
        if case!='odom_loss' or elapsed<.5: odom_pub.publish(odom)
        transform = TransformStamped(); transform.header=odom.header; transform.child_frame_id='base_link'; transform.transform.rotation.w=1.
        tf.sendTransform(transform)
        if duplicate_tf: duplicate_tf.sendTransform(transform)
    node.create_timer(.025,publish)
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        duplicate.destroy_node();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--peer',action='store_true');parser.add_argument('--namespace');parser.add_argument('--case')
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    if args.peer: peer(args.namespace,args.case);return
    from bindu_runtime.navigation_preflight import run
    from tests.unit.test_orin_navigation import configured
    args.output.mkdir(parents=True,exist_ok=False); results=[]
    cases={'fresh':None,'scan_loss':'scan: STREAM_STALE','odom_loss':'odom: STREAM_STALE',
           'old_stamp':'SOURCE_TIME_INVALID','frozen':'NO_ADVANCING_DATA',
           'wrong_frame':'INVALID_DATA_OR_FRAME','duplicate_tf':'TF_DUPLICATE_AUTHORITY'}
    for case, expected in cases.items():
        ns='/orin_readonly_'+uuid.uuid4().hex[:8];site=configured()
        site.update(namespace=ns,scan_topic=ns+'/scan',odom_topic=ns+'/odom',tf_topic=ns+'/tf',tf_static_topic=ns+'/tf_static')
        site['checks'].update(sample_seconds=1.5,max_age=.3)
        site['tf_authorities']={key:ns+'/sensor_fixture' for key in ('odom_to_base','base_to_scan')}
        log=(args.output/(case+'.log')).open('w')
        process=subprocess.Popen([sys.executable,'-m',__spec__.name,'--peer','--namespace',ns,'--case',case],
                                 stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            # Let the fixture create DDS endpoints, without relying on the preflight itself.
            time.sleep(.5)
            errors=run(site,'sensors')
            assert not errors if expected is None else any(expected in e for e in errors),(case,errors)
            results.append(dict(case=case,passed=True,errors=errors))
        finally:
            import signal
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=3)
            log.close()
    (args.output/'results.json').write_text(json.dumps(results,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(results,ensure_ascii=False))


if __name__=='__main__':main()
