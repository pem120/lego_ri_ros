from lego_spike_interface.device_interfaces import *
import rclpy
import argparse


def main():
    parser = argparse.ArgumentParser(
        "Serial interface for Lego Mindstorms and Lego Spike Prime hub"
    )
    parser.add_argument(
        "-p",
        "--port",
        metavar="TTY",
        type=str,
        default="/dev/lego",
        dest="port",
        help="Serial port to open (default /dev/lego)",
    )
    parser.add_argument(
        "-b",
        "--baud",
        metavar="INT",
        type=int,
        default=115200,
        dest="baud",
        help="Serial port to open (default /dev/lego)",
    )
    parser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        dest="verbose",
        help="Show all serial I/O from the hub",
    )
    args, unk_args = parser.parse_known_args()

    rclpy.init()
    node = SerialInterfaceNode(port=args.port, baud=args.baud, verbose=args.verbose)
    node.open()
    node.run()
    rclpy.spin(node)
    node.close()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
