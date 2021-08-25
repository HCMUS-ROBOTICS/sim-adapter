#!/usr/bin/python3

import rospy
from sim_adapter.dira_adapter import DiRASimAdapter
from sim_adapter.simulator import Simulator


def main():
    rospy.init_node('sim_adapter_node')
    rospy.loginfo('sim_adapter_node is initialized')
    simulator = Simulator()

    # TODO: set different adapter based on argv
    simulator.set_adapter(DiRASimAdapter(simulator))
    simulator.run_loop()


if __name__ == "__main__":
    main()
