"""Inside recording container: enable or pause the manual motion gate."""
import argparse
import time
import rclpy
from std_srvs.srv import SetBool


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('action',choices=['enable','pause'])
    args=parser.parse_args()
    rclpy.init();node=rclpy.create_node('recording_control')
    def call(service,kind,request,timeout):
        client=node.create_client(kind,service)
        if not client.wait_for_service(timeout_sec=3.):raise RuntimeError(service+' unavailable')
        future=client.call_async(request)
        deadline=time.monotonic()+timeout
        while not future.done() and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
        if not future.done():raise TimeoutError(service+' timed out')
        result=future.result()
        print(service+': '+result.message,flush=True)
        if not result.success:raise RuntimeError(result.message)
    try:
        request=SetBool.Request();request.data=args.action=='enable'
        call('/real/manual_allow_motion',SetBool,request,3.)
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
