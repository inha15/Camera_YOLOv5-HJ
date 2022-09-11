import argparse

def parse_opt():
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', nargs='+', type=str, default='yolov5s.pt', help='model path(s)')
    opt = parser.parse_args()

    return opt

def prints(weights=None):
    print(weights)
    print(type(weights))
    print(type(weights[0]))


opt = parse_opt()
prints(**vars(opt))
