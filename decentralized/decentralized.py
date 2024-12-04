import velocity_obstacle.velObs as velOb
import argparse
import nlmpc.nonlinearmpc as nmpc

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-m", "--mode", help = "mode of obstacle avoidance; options: velocity obstacle or nmpc"
    )
    parser.add_argument(
        "-f", "--filename", help="filename, incase you want to save the animation"
    )
    args = parser.parse_args()
    if args.mode == "velocity_obstacle":
        velOb.simulate(args.filename)
    elif args.mode == "nmpc":
        nmpc.simulate(args.filename)
    else:
        print("Please enter correctly")