import os
import sys
import argparse
import numpy as np
os_version = os.popen("lsb_release -r | cut -f2").read().replace("\n", "")
if os_version == "18.04":
    import matplotlib
    matplotlib.use('tkagg')
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
import matplotlib.colors as mcolors

sys.path.insert(0, os.path.join(os.path.realpath(os.path.dirname(__file__)), '../data_analysis/'))
from plotting_functions import get_colors, get_linestyle

plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = ['CMU Serif']
plt.rcParams['mathtext.fontset'] = 'cm'

LINE_STYLES = list(filter(lambda x: (x != 'None' and x != ' ' and x != ''), mlines.lineStyles.keys()))

def parse_arguments():
    parser = argparse.ArgumentParser(description='Plot data of path planner for the MT of Friedrich M. Rockenbauer.')
    parser.add_argument('--dest', dest='destination_folder', type=str, default=None,
                        help='output folder where plots shall be stored')
    parser.add_argument('--file_name', dest='file_name', type=str, default=None,
                        help='prefix for all file names')
    parser.add_argument('--n_cols', dest='n_cols', type=int, default=1,
                        help='prefix for all file names')
    parser.add_argument('--font_size', dest='font_size', type=int, default=10,
                        help='Font size for legend, labels and title')
    parser.add_argument('names', nargs="*", type=str, help='')
    args = parser.parse_args()
    return args

# Press the green button in the gutter to run the script.
if __name__ == '__main__':

    arguments = parse_arguments()

    names = arguments.names
    n_cols = arguments.n_cols
    n_rows = np.ceil(len(names)/n_cols)
    file_name = arguments.file_name
    destination_folder = arguments.destination_folder
    
    plt.rcParams.update({'font.size': arguments.font_size})


    if len(names) == 0: 
        exit(1)

    if file_name == "": 
        file_name = None
    
    if file_name is None: 
        file_name = '_'.join(names)

    markers = get_linestyle(len(names))
    colors = get_colors(len(names))

    fig, ax = plt.subplots(figsize=(5*n_cols, 2.7))

    for i, name in enumerate(names): 
        ax.plot(np.linspace(0, 100, 100), [1 if i==99 else 0 for i in range(0,100)], label=name, color=colors[i], linestyle=markers[i])

    # Put a legend below current axis
    ax.legend(loc='upper center', ncol=n_cols)

    if destination_folder is not None:
        if os.path.isdir(destination_folder):
            plt.savefig(os.path.join(destination_folder, file_name + ".png"),
                        format="png", dpi=400)
            plt.savefig(os.path.join(destination_folder, file_name + ".pdf"),
                        format="pdf", dpi=400)
    else: 
        plt.show()