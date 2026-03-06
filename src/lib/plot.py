import matplotlib.pyplot as plt
from matplotlib import colors as mcolors

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
