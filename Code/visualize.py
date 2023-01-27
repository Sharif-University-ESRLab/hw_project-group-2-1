import matplotlib.pyplot as plt
import os

files = os.listdir('out/')
def get_parameter_name(s):
    if s == 'a':
        return 'Acceleration'
    elif s == 'v':
        return 'Velocity'
    elif s == 'x':
        return 'Location'

def get_method_name(s: str):
    if s == 'SW':
        return 'Sliding Window'
    elif s == 'MA':
        return 'Moving Average'
    else:
        return 'Simple'

def get_is_adjusted(s: str):
    if s == 'adjust':
        return True
    else:
        return False

for file_name in files:
    output = file_name.split('.')[0]
    temp = output.split('_')
    parameter_name = get_parameter_name(temp[1])
    is_adjusted = get_is_adjusted(temp[2])
    algorithm_name = temp[3]
    method_name = get_method_name(temp[4])

    x = []
    y = []
    z = []
    NUM_ITERATION = 40
    iteration = list(range(1, NUM_ITERATION + 1))
    with open(f'out/{file_name}') as f:
        lines = f.readlines()

        for line in lines:
            data_point = line[1:-2].strip().split(',')
            x.append(round(float(data_point[0]), 2))
            y.append(round(float(data_point[1]), 2))
            z.append(round(float(data_point[2]), 2))

    figure, axis = plt.subplots(1, 3, sharex=True)
    axis[0].plot(iteration, x)
    axis[0].set_title("x_axis")
    axis[0].axvline(x=10, color='red')
    axis[0].axvline(x=20, color='red')
    axis[0].axvline(x=30, color='red')

    axis[1].plot(iteration, y)
    axis[1].set_title("y_axis")
    axis[1].axvline(x=10, color='red')
    axis[1].axvline(x=20, color='red')
    axis[1].axvline(x=30, color='red')

    axis[2].plot(iteration, z)
    axis[2].set_title("z_axis")
    axis[2].axvline(x=10, color='red')
    axis[2].axvline(x=20, color='red')
    axis[2].axvline(x=30, color='red')
    figure.supxlabel('iteration number')
    figure.supylabel(f'{parameter_name}')
    figure.suptitle(f'{parameter_name}\nalgorithm: {algorithm_name} - {method_name}\na0 adjustement: {is_adjusted}')
    figure.tight_layout()
    figure.savefig(f'final_plots/{output}.png')
