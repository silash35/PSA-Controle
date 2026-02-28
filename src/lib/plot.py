import matplotlib.pyplot as plt
from matplotlib import colors as mcolors

from .read_data import y_labels

colors = list(mcolors.TABLEAU_COLORS.keys())

plt.rcParams.update(
    {
        "figure.figsize": (16, 6),
        "figure.autolayout": True,
        "axes.grid": True,
        # 'figure.dpi': 300,
        "font.size": 16,
    }
)


def legend_outside(ax: plt.Axes):
    ax.legend(loc="center left", bbox_to_anchor=(1, 0.5), frameon=False)


def plot_comparison(cycles, real_y, model_y, model_name):
    ymin = min(real_y[:, i].min() for i in range(len(y_labels)))
    ymax = max(real_y[:, i].max() for i in range(len(y_labels)))

    for i in range(len(y_labels)):
        plt.scatter(
            cycles,
            real_y[:, i],
            label=f"{y_labels[i]} (gPROMS)",
            color=colors[i],
            alpha=0.6,
        )
        plt.plot(
            cycles,
            model_y[:, i],
            label=f"{y_labels[i]} ({model_name})",
            color=colors[i],
        )
    plt.ylim(ymin * 0.9, ymax * 1.1)

    legend_outside(plt.gca())
    plt.ylabel("Valores / %")
    plt.xlabel("Ciclo")

    # plt.show()
    plt.savefig(f"../figures/comparision-{model_name}.png")
    plt.close()
