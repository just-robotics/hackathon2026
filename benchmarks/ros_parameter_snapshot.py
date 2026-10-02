"""Read parameter services together using one ROS node; no parameter writes."""
import json
import sys
import time


def nested_parameters(names, values):
    """Preserve the nested structure produced by ros2 param dump."""
    if len(names) != len(values):
        raise RuntimeError('Incomplete parameter response')
    result = {}
    for name, value in zip(names, values):
        target = result
        parts = name.split('.')
        for part in parts[:-1]:
            target = target.setdefault(part, {})
        target[parts[-1]] = value
    return result


def main():
    import rclpy
    from rcl_interfaces.srv import ListParameters, GetParameters
    from rclpy.parameter import parameter_value_to_python

    nodes = json.loads(sys.argv[1])
    rclpy.init()
    node = rclpy.create_node('hsl_runtime_parameter_audit')
    try:
        clients = {name: (
            node.create_client(ListParameters, name + '/list_parameters'),
            node.create_client(GetParameters, name + '/get_parameters')) for name in nodes}
        deadline = time.monotonic() + 20

        def spin():
            if time.monotonic() >= deadline:
                raise RuntimeError('Parameter snapshot timed out')
            rclpy.spin_once(node, timeout_sec=0.05)

        while not all(client.service_is_ready()
                      for pair in clients.values() for client in pair):
            spin()

        def collect(futures):
            while not all(future.done() for future in futures.values()):
                spin()
            return {key: future.result() for key, future in futures.items()}

        listed = collect({key: pair[0].call_async(ListParameters.Request(depth=0))
                          for key, pair in clients.items()})
        values = collect({key: clients[key][1].call_async(
            GetParameters.Request(names=reply.result.names)) for key, reply in listed.items()})
        result = {key: nested_parameters(listed[key].result.names,
            [parameter_value_to_python(value) for value in reply.values])
            for key, reply in values.items()}
        print(json.dumps(result))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
