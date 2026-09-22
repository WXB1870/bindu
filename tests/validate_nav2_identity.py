#!/usr/bin/env python3
"""Exercise native RMW publisher identity and immutable per-goal binding."""
import argparse
import json
from pathlib import Path
import subprocess
import uuid
import time
import rclpy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
from bindu_interfaces.msg import NavigationVelocity
from validate_ros import until, terminate


def main():
    p=argparse.ArgumentParser();p.add_argument('--binary',required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False);rclpy.init();results=[]
    try:
        for case in ('publisher_changed','replayed_stamp','bind_once'):
            ns='/identity_'+uuid.uuid4().hex[:8];node=rclpy.create_node('test',namespace=ns);seen=[];fault=[]
            pub=node.create_publisher(TwistStamped,'cmd_vel',10)
            node.create_subscription(NavigationVelocity,'identified_velocity',seen.append,10)
            node.create_subscription(String,'identity_fault',lambda m:fault.append(m.data),10)
            until(node,lambda:node.get_publishers_info_by_topic(ns+'/cmd_vel'))
            gid=bytes(node.get_publishers_info_by_topic(ns+'/cmd_vel')[0].endpoint_gid).hex()
            start=node.get_clock().now().nanoseconds/1e9
            with (a.output/(case+'.log')).open('w') as log:
                goal_id=uuid.uuid4().hex
                process=subprocess.Popen([a.binary,'' if case=='bind_once' else goal_id,ns,gid,str(start),'--ros-args','-r','__ns:='+ns],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                try:
                    until(node,lambda:pub.get_subscription_count()>0)
                    if case=='bind_once':
                        from rcl_interfaces.srv import SetParameters
                        from rcl_interfaces.msg import Parameter, ParameterValue
                        from validate_ros import wait
                        client=node.create_client(SetParameters,ns+'/velocity_identity/set_parameters')
                        assert client.wait_for_service(timeout_sec=5)
                        def bind(value):return wait(node,client.call_async(SetParameters.Request(parameters=[
                            Parameter(name='goal_id',value=ParameterValue(type=4,string_value=value))]))).results[0]
                        assert bind(goal_id).successful
                        refused=bind(uuid.uuid4().hex)
                        assert not refused.successful and refused.reason=='GOAL_BINDING_IMMUTABLE'
                    m=TwistStamped();m.header.stamp=node.get_clock().now().to_msg();m.header.frame_id='base_link';m.twist.linear.x=.1
                    pub.publish(m);until(node,lambda:seen)
                    assert seen[0].goal_id==goal_id and seen[0].source_id==ns and seen[0].command.header.stamp==m.header.stamp
                    if case=='bind_once':
                        results.append({'case':case,'passed':True,'goal_id':goal_id,'rebinding_rejected':True})
                        continue
                    if case=='publisher_changed':
                        foreign=node.create_publisher(TwistStamped,'cmd_vel',10)
                        until(node,lambda:foreign.get_subscription_count()>0)
                        m.header.stamp=node.get_clock().now().to_msg();m.twist.linear.x=.39;foreign.publish(m)
                        expected='NAV2_PUBLISHER_CHANGED'
                    else:pub.publish(m);expected='NAV2_COMMAND_STALE'
                    until(node,lambda:fault)
                    assert fault[-1]==expected and len(seen)==1,(fault,len(seen))
                    results.append({'case':case,'passed':True,'code':expected,'accepted':len(seen)})
                finally:terminate(process);node.destroy_node()
    finally:rclpy.shutdown();(a.output/'results.json').write_text(json.dumps(results,indent=2))
    print(json.dumps(results))
if __name__=='__main__':main()
