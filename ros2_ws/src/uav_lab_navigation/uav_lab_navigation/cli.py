"""Operator interface for bounded three-dimensional navigation."""
import argparse
import json
import math
import time
from pathlib import Path
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from uav_lab_interfaces.srv import PlanPath
from uav_lab_interfaces.action import Navigate
from rclpy.action import ActionClient
from uav_lab_tools.cli import Operator
from .fixture import write_fixture
from .benchmark import benchmark


def pose(values):
    msg=PoseStamped();msg.header.frame_id='odom'
    msg.pose.position.x,msg.pose.position.y,msg.pose.position.z=map(float,values[:3])
    yaw=values[3] if len(values)>3 else 0.
    msg.pose.orientation.z=math.sin(yaw/2);msg.pose.orientation.w=math.cos(yaw/2)
    return msg


class Navigator:
    def __init__(self,node,timeout):
        self.node,self.timeout=node,timeout;self.latest=None;self.received=0.
        self.operator=Operator(node,timeout)
        node.create_subscription(DiagnosticArray,'/uav001/navigation/diagnostics',self.on_status,10)

    def on_status(self,msg):
        for status in msg.status:
            if status.name=='uav001/navigation':
                self.latest={v.key:v.value for v in status.values};self.received=time.monotonic()

    def status(self):
        deadline=time.monotonic()+self.timeout
        while (self.latest is None or time.monotonic()-self.received>1) and time.monotonic()<deadline:self.operator.spin()
        if self.latest is None or time.monotonic()-self.received>1:raise RuntimeError('navigation diagnostic stream unavailable')
        return self.latest

    def plan(self,target):
        client=self.node.create_client(PlanPath,'/uav001/navigation/plan')
        try:
            if not client.wait_for_service(timeout_sec=5):raise RuntimeError('planning service unavailable')
            request=PlanPath.Request();request.goal=pose(target)
            response=self.operator.wait(client.call_async(request))
            return {'success':response.success,'reason':response.reason,'map_version':response.map_version,
                'points':[[p.pose.position.x,p.pose.position.y,p.pose.position.z] for p in response.path.poses]}
        finally:self.node.destroy_client(client)

    def goto(self,target):
        action=ActionClient(self.node,Navigate,'/uav001/navigation/navigate');handle=None;submitted=None
        try:
            if not action.wait_for_server(timeout_sec=5):raise RuntimeError('navigation action unavailable')
            submitted=action.send_goal_async(Navigate.Goal(goal=pose(target)))
            handle=self.operator.wait(submitted)
            if not handle.accepted:raise RuntimeError('navigation request rejected')
            result=self.operator.wait(handle.get_result_async()).result
            return {'success':result.success,'reason':result.reason}
        except (RuntimeError,KeyboardInterrupt):
            try:
                if handle is None and submitted is not None:handle=self.operator.wait(submitted,5.)
                if handle and handle.accepted:
                    self.operator.wait(handle.cancel_goal_async(),3.)
                    self.operator.wait(handle.get_result_async(),5.)
            except (RuntimeError,KeyboardInterrupt):
                # Uncertain ownership must invalidate pending acceptance too.
                try:self.operator.service('navigation/abort')
                except RuntimeError:pass
                try:self.operator.service('hold')
                except RuntimeError:pass
            raise
        finally:action.destroy()

    def destroy(self):self.operator.action.destroy()

    def demo(self,runs):
        results=[]
        for index in range(runs):
            status=self.operator.status()
            if status.get('armed')!='False' or status.get('landed')!='True':raise RuntimeError('demo requires landed disarmed vehicle')
            self.operator.service('arm')
            try:
                self.operator.flight('TAKEOFF',height=2.)
                blocked=self.plan([1.5,0,2])
                if blocked['success']:raise RuntimeError('obstacle-interior goal unexpectedly accepted')
                forward=self.goto([2.5,3.6,2])
                if not forward['success']:raise RuntimeError('navigation demo failed: '+str(forward))
                backward=self.goto([0,0,2])
                if not backward['success']:raise RuntimeError('navigation return failed: '+str(backward))
                self.operator.flight('LAND')
            except (RuntimeError,KeyboardInterrupt) as error:
                try:self.operator.flight('LAND')
                except RuntimeError as landing_error:raise RuntimeError(str(error)+'; landing request failed: '+str(landing_error)) from error
                raise
            final=self.operator.status()
            deadline=time.monotonic()+5
            while (final.get('armed')!='False' or final.get('landed')!='True') and time.monotonic()<deadline:
                self.operator.spin(.1);final=self.operator.status()
            if final.get('armed')!='False' or final.get('landed')!='True':raise RuntimeError('landing/disarm not confirmed')
            results.append({'run':index+1,'blocked':blocked,'forward':forward,'return':backward,'landed_disarmed':True})
        return {'success':True,'runs':results}


def main(args=None):
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--timeout',type=float,default=90.)
    sub=p.add_subparsers(dest='command',required=True);sub.add_parser('status')
    for command in ('plan','goto'):
        child=sub.add_parser(command);child.add_argument('target',nargs=3,type=float)
    demo=sub.add_parser('demo');demo.add_argument('--runs',type=int,default=3)
    f=sub.add_parser('fixture');f.add_argument('output',type=Path)
    b=sub.add_parser('benchmark');b.add_argument('input',type=Path);b.add_argument('output',type=Path)
    options=p.parse_args(args);node=None;nav=None
    try:
        if not math.isfinite(options.timeout) or options.timeout<=0:raise ValueError('positive finite timeout required')
        if options.command=='fixture':result=write_fixture(options.output)
        elif options.command=='benchmark':result=benchmark(options.input,options.output)
        else:
            rclpy.init();node=rclpy.create_node('navigation_operator');nav=Navigator(node,options.timeout)
            if options.command=='status':result=nav.status()
            elif options.command=='plan':result=nav.plan(options.target)
            elif options.command=='goto':result=nav.goto(options.target)
            else:
                if options.runs<=0:raise ValueError('positive runs required')
                result=nav.demo(options.runs)
        print(json.dumps(result,allow_nan=False));return 0 if result.get('success',result.get('ready',result.get('passed',True))) not in (False,'False') else 1
    except (ValueError,RuntimeError,KeyboardInterrupt,OSError) as error:
        print(json.dumps({'success':False,'reason':str(error)}));return 1
    finally:
        if nav:nav.destroy()
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
