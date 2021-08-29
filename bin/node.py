#!/usr/bin/python3

import rospy
from sim_adapter.dira_adapter import DiRASimAdapter
from sim_adapter.uit_adapter import UITSimAdapter
from sim_adapter.simulator import Simulator

import eventlet

def main():
    rospy.init_node('sim_adapter_node', disable_signals=True)
    rospy.loginfo('sim_adapter_node is initialized')
    simulator = Simulator()

    adapter = rospy.get_param('~adapter', default='dira')

    rospy.loginfo('Choose adapter for %s simulation', adapter)

    if adapter not in ['dira', 'uit']:
        raise ValueError('Adapter must be in [dira, uit]')

    if adapter == 'dira':
        simulator.set_adapter(DiRASimAdapter(simulator))
    if adapter == 'uit':
        simulator.set_adapter(UITSimAdapter(simulator))

    simulator.run_loop()

if __name__ == "__main__":
    main()
