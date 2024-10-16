import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np

plt.rcParams.update(
    {
        "text.usetex": True,
        "font.family": "serif",
        "font.serif": [],
        "pgf.rcfonts": False,
        "legend.fontsize": "x-small",
        "axes.labelsize": "x-small",
        "axes.titlesize": "x-small",
        "xtick.labelsize": "xx-small",
        "ytick.labelsize": "xx-small",
    }
)


df = pd.read_csv("robot_comparison.csv")
df['volume'] = (df.width * df.height * df.length) * 1e-3
#df['volume_log'] = np.log10(df.volume.values)
df = df[df.paper_include == True] #[['brand', 'product', 'mode', 'price', 'price_estimate', 'max_vel', 'mass', 'volume', 'width', 'length', 'height', 'robot_type', 'pareto_frontier', 'commercial']]

def generate_table(df):
    cols_headers = {
        'brand': 'Brand/Uni',
        'product_bibtex': 'Product',
        'max_vel': '\\makecell{{Vel \\\\ {{[m/s]}}}}',
        'cost': '\\makecell{{Cost \\\\ {{[USD]}}}}',
        'mass': '\\makecell{{Mass \\\\ {{[kg]}}}}',
        'lbh': '\\makecell{{L x B x H\\\\ {{[mm]}}}}',
        'robot_type': '\\makecell{{Multi \\\\ {{Agent}}}}',
        'commercial': 'Commercial'
    }
    df = df.copy()
    df['cost'] = df.apply(lambda x: ('*' if x.price_estimate else '') + f'{x.price:,}', axis=1)
    df['product_bibtex'] = df.apply(lambda x: f"{x['product']} " + (f"\cite{{{x.bibtex}}}" if isinstance(x.bibtex, str) else "(ours)"), axis=1)
    df = df.sort_values('max_vel')
    df['lbh'] = df.apply(lambda x: f"${int(x.width)} \\times {int(x.length)} \\times {int(x.height)}$", axis=1)
    df_tab = df[list(cols_headers.keys())]
    formatters = {
        'commercial': lambda x: "\checkmark" if x else "",
        'robot_type': lambda x: "\checkmark" if x == 'multi' else "",
        'mass': lambda x: f"{x:.2f}",
        'max_vel': lambda x: f"{x:.2f}"
    }
    df_tab.to_latex("robot_comparison_table.tex", index=False, header=list(cols_headers.values()), column_format="llrrrccc", formatters=formatters)


def generate_figure(df):
    fig, (ax, ax_legend) = plt.subplots(1, 2, figsize=[5.0, 3.2], width_ratios=[8, 1])
    palette = sns.color_palette()
    palette[3] = (0.5, 0.5, 0.5) # Gray for Airborne
    sns.scatterplot(
        ax=ax,
        data=df,
        x='max_vel', y='price',
        hue='mode',
        hue_order=[
            "Ackermann",
            "Differential",
            "Omnidirectional",
            "Airborne"
        ],
        style='robot_type',
        style_order=[
            "single",
            "multi"
        ],
        markers=["o", "P"],
        palette=palette
    )

    pareto_rows = df.loc[df.pareto_frontier_wo_drones.dropna().sort_values().index]
    pareto = pareto_rows[['price', 'max_vel']].values
    ax.plot(pareto[:, 1], pareto[:, 0], color='gray', ls='--')

    handles, labels = ax.get_legend_handles_labels()
    l1 = ax.legend(handles=handles[1:5], labels=labels[1:5], loc="upper left")
    l2 = ax.legend(handles=handles[6:8], labels=labels[6:8], loc="upper right")
    ax.add_artist(l1) # we need this because the 2nd call to legend() erases the first

    texts = []
    legend_idx = 0
    df['product_lower'] = df['product'].str.lower()
    for _, row in df.sort_values('product_lower').iterrows():
        offset = row.max_vel * 0.1
        if isinstance(row.bibtex, str):
            ax_legend.text(0, 1 - legend_idx * 0.025, f"{row['product']} \cite{{{row.bibtex}}}", fontsize=5, ha='left')
            legend_idx += 1
            texts.append(ax.text(row.max_vel + offset, row.price, f"\cite{{{row.bibtex}}}", fontsize=6, ha='left', va='center'))
        else:
            texts.append(ax.text(row.max_vel + offset, row.price, f"{row['product']} (ours)", fontsize=6, ha='left', va='center'))

    ax.text(4, 150, f"Future Work", fontsize=14, ha='left', va='center')

    ax.set_xscale('log')
    ax.set_yscale('log')
    ax.set_xlabel('Maximum Velocity [m/s]')
    ax.set_ylabel('Cost per unit [USD]')
    ax.grid()

    ax_legend.axis("off")
    fig.tight_layout()
    plt.savefig('robot_comparison.pgf', bbox_inches="tight", pad_inches=0)
    plt.show()

generate_table(df[df['mode'] == "Omnidirectional"])
#generate_table(df[(df['mode'] == "Omnidirectional") | (df['robot_type'] == "multi")])
generate_figure(df)

def to_bibtex(df):
    return ",".join(df['bibtex'].dropna().values)

def to_stats(df):
    df_v = df.sort_values('max_vel')[['bibtex', 'max_vel', 'price']].dropna()
    fastest = df_v.iloc[-1]
    slowest = df_v.iloc[0]
    df_p = df.sort_values('price')[['bibtex', 'max_vel', 'price']].dropna()
    cheapest = df_p.iloc[-1]
    expensivest = df_p.iloc[0]
    return f"{slowest.bibtex}, {slowest.max_vel}, {fastest.bibtex}, {fastest.max_vel}, {cheapest.price}, {expensivest.price}, {df.price.mean()}"

multi = df[(df.robot_type == "multi")]
print("multi", to_bibtex(multi))
print(to_stats(multi))
print()

diffdrive_multi = df[(df['mode'] == "Differential") & (df.robot_type == "multi")]
print("diffdrive_multi", to_bibtex(diffdrive_multi))
print(to_stats(diffdrive_multi))
print()

diffdrive_single = df[(df['mode'] == "Differential") & (df.robot_type == "single")]
print("diffdrive_single", to_bibtex(diffdrive_single))
print(to_stats(diffdrive_single))
print()

nondiffdrive_multi = df[(df['mode'] != "Differential") & (df.robot_type == "multi")]
print("nondiffdrive_multi", to_bibtex(nondiffdrive_multi))
print(to_stats(nondiffdrive_multi))
print()

omni = df[(df['mode'] == "Omnidirectional")]
print("omni", to_bibtex(omni))
print(to_stats(omni))
print()

ackermann = df[(df['mode'] == "Ackermann")]
print("ackermann", to_bibtex(ackermann))
print(to_stats(ackermann))
print()

drone = df[(df['mode'] == "Airborne")]
print("airborne", to_bibtex(drone))
print(to_stats(drone))
print()
