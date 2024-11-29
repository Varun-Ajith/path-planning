import velocity_obstacle.velObs as velOb
import argparse

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
    else:
        print("Please enter correctly")