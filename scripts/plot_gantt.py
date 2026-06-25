"""
plot_gantt.py
=============

Builds a Gantt chart from task_execution.csv (written by
ExperimentLogger.log_task_execution): one horizontal bar per executed
task, from started_at to finished_at, grouped and colored by agent_id.

Usage
-----
    python plot_gantt.py path/to/task_execution.csv
    python plot_gantt.py path/to/task_execution.csv --run-id my_run_1
    python plot_gantt.py path/to/task_execution.csv --out gantt.png

If the CSV contains multiple run_id values, pass --run-id to pick one
(otherwise the most recent run, by max timestamp, is used automatically
and a warning is printed).
"""

import argparse
import sys

import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.dates as mdates


def load_execution_data(csv_path, run_id=None):
    df = pd.read_csv(csv_path)

    if "started_at" not in df.columns or "finished_at" not in df.columns:
        raise ValueError(
            "task_execution.csv has no 'started_at'/'finished_at' columns. "
            "These are written by Agent.step() via the extra_fields passed "
            "to log_task_execution."
        )

    if run_id is None:
        if df["run_id"].nunique() > 1:
            run_id = df.loc[df["timestamp"].idxmax(), "run_id"]
            print(
                f"[plot_gantt] Multiple run_id values found in {csv_path}; "
                f"defaulting to the most recent one: '{run_id}'. "
                f"Pass --run-id to pick a different run.",
                file=sys.stderr,
            )
        else:
            run_id = df["run_id"].iloc[0]

    df = df[df["run_id"] == run_id].copy()
    if df.empty:
        raise ValueError(f"No rows found for run_id='{run_id}' in {csv_path}.")

    df["started_at"] = pd.to_datetime(df["started_at"], unit="s")
    df["finished_at"] = pd.to_datetime(df["finished_at"], unit="s")
    df["duration_s"] = (df["finished_at"] - df["started_at"]).dt.total_seconds()

    # Order tasks top-to-bottom by start time within each agent, and group
    # agents together (rather than interleaving rows by raw start time).
    df.sort_values(["agent_id", "started_at"], inplace=True)
    return df, run_id


def plot_gantt(df, run_id, out_path=None, title=None):
    agents = list(df["agent_id"].unique())
    # Stable row position per task: grouped by agent, in chronological
    # order within each agent (matches df's sort order already).
    df = df.reset_index(drop=True)
    df["row_pos"] = range(len(df))

    cmap = plt.get_cmap("tab20")
    agent_colors = {agent: cmap(i % 20) for i, agent in enumerate(agents)}

    fig_height = max(3, 0.4 * len(df) + 1.5)
    fig, ax = plt.subplots(figsize=(12, fig_height))

    for _, row in df.iterrows():
        ax.barh(
            row["row_pos"],
            width=row["finished_at"] - row["started_at"],
            left=row["started_at"],
            height=0.6,
            color=agent_colors[row["agent_id"]],
            edgecolor="black",
            linewidth=0.5,
        )
        ax.text(
            row["finished_at"], row["row_pos"],
            f"  {row['task_id']}",
            va="center", ha="left", fontsize=8,
        )

    ax.set_yticks(df["row_pos"])
    ax.set_yticklabels(df["agent_id"])
    ax.invert_yaxis()  # first task at the top
    ax.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M:%S"))
    ax.set_xlabel("Time")
    ax.set_ylabel("Agent")
    ax.set_title(title or f"Task execution Gantt chart -- run '{run_id}'")
    ax.grid(axis="x", linestyle=":", alpha=0.5)

    # Collapse repeated agent labels into one centered label per group, so
    # an agent with several tasks doesn't repeat its name on every row.
    new_labels = [""] * len(df)
    for agent in agents:
        positions = df.index[df["agent_id"] == agent].tolist()
        if positions:
            mid = positions[len(positions) // 2]
            new_labels[mid] = agent
    ax.set_yticklabels(new_labels)

    # Faint separators between agent groups
    boundaries = df.index[df["agent_id"] != df["agent_id"].shift()].tolist()[1:]
    for b in boundaries:
        ax.axhline(b - 0.5, color="grey", linewidth=0.8, alpha=0.4)

    # Legend: one swatch per agent (de-duplicated)
    handles = [
        plt.Rectangle((0, 0), 1, 1, color=agent_colors[a]) for a in agents
    ]
    ax.legend(handles, agents, loc="upper left", bbox_to_anchor=(1.01, 1), title="Agent")

    fig.tight_layout()

    if out_path:
        fig.savefig(out_path, dpi=150, bbox_inches="tight")
        print(f"[plot_gantt] Saved chart to {out_path}")
    else:
        plt.show()

    return fig


def main():
    parser = argparse.ArgumentParser(description="Plot a Gantt chart from task_execution.csv")
    parser.add_argument("csv_path", help="Path to task_execution.csv")
    parser.add_argument("--run-id", default=None, help="Which run_id to plot (default: most recent)")
    parser.add_argument("--out", default=None, help="Output image path (e.g. gantt.png). If omitted, shows interactively.")
    parser.add_argument("--title", default=None, help="Custom chart title")
    args = parser.parse_args()

    df, run_id = load_execution_data(args.csv_path, run_id=args.run_id)
    plot_gantt(df, run_id, out_path=args.out, title=args.title)


if __name__ == "__main__":
    main()
