"""
plot_gantt.py
=============

Builds a Gantt chart from task_execution.csv (written by
ExperimentLogger.log_task_execution): one horizontal bar per executed
task, plotted as elapsed minutes since the experiment started (t=0 at
the earliest started_at in the run), grouped and colored by agent_id.

Leaf tasks (is_leaf=True -- typically the end-goal/final node with no
successors) have no real duration of their own: they just mark the
instant every subgoal converged. These are drawn as milestone markers
(diamonds at finished_at, with a dashed vertical line through the chart)
rather than as duration bars, following standard Gantt-chart convention
for zero-duration milestones -- a full-width bar would misleadingly
imply the goal node was "busy" the whole time, and removing it entirely
would hide the convergence event the chart is meant to show.

Usage
-----
    python plot_gantt.py path/to/task_execution.csv
    python plot_gantt.py path/to/task_execution.csv --run-id my_run_1
    python plot_gantt.py path/to/task_execution.csv --out gantt.png

If the CSV contains multiple run_id values, pass --run-id to pick one
(otherwise the most recent run, by max timestamp, is used automatically
and a warning is printed).

Requires started_at / finished_at columns. These are only present on
rows logged after the started_at/finished_at fields were added to
Agent.step()'s call to log_task_execution -- older CSVs without them
will raise a clear error telling you what to do.
"""

import argparse
import sys

import pandas as pd
import matplotlib.pyplot as plt


def load_execution_data(csv_path, run_id=None):
    df = pd.read_csv(csv_path)

    if "started_at" not in df.columns or "finished_at" not in df.columns:
        raise ValueError(
            "task_execution.csv has no 'started_at'/'finished_at' columns. "
            "These are written by Agent.step() via the extra_fields passed "
            "to log_task_execution -- make sure you're using the updated "
            "Agent.py and that this CSV was generated after that change."
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

    # Leaf tasks (is_leaf=True, i.e. zero successors -- typically the
    # end-goal/final node) are kept in the chart but tagged as milestones:
    # plot_gantt() draws them as zero-duration markers instead of bars
    # (see module docstring for why).
    if "is_leaf" in df.columns:
        df["is_milestone"] = df["is_leaf"].astype(str).str.lower().isin(["true", "1"])
    else:
        df["is_milestone"] = False

    started_at = pd.to_datetime(df["started_at"], unit="s")
    finished_at = pd.to_datetime(df["finished_at"], unit="s")
    df["duration_s"] = (finished_at - started_at).dt.total_seconds()

    # Elapsed time since the experiment began (t=0 at the earliest
    # started_at across the whole run), in minutes. This is the only
    # x-axis the chart uses -- wall-clock time-of-day isn't meaningful
    # for comparing runs or for a reader, only how far into the
    # experiment each task happened.
    experiment_start = started_at.min()
    df["started_min"] = (started_at - experiment_start).dt.total_seconds() / 60.0
    df["finished_min"] = (finished_at - experiment_start).dt.total_seconds() / 60.0

    # Order tasks top-to-bottom by start time within each agent, and group
    # agents together (rather than interleaving rows by raw start time).
    df.sort_values(["agent_id", "started_min"], inplace=True)
    return df, run_id


def plot_gantt(df, run_id, title=None):
    agents = list(df["agent_id"].unique())
    # Stable row position per task: grouped by agent, in chronological
    # order within each agent (matches df's sort order already).
    df = df.reset_index(drop=True)
    df["row_pos"] = range(len(df))

    cmap = plt.get_cmap("tab20")
    agent_colors = {agent: cmap(i % 20) for i, agent in enumerate(agents)}

    fig_height = max(3, 0.4 * len(df) + 1.5)
    fig, ax = plt.subplots(figsize=(12, fig_height))

    # Extra horizontal clearance for the milestone label, scaled to the
    # chart's time span so it clears the (larger) diamond marker -- a
    # fixed offset in minutes, not just leading whitespace in the text,
    # since the diamond's render size doesn't translate to a fixed
    # number of characters.
    span = df["finished_min"].max() - df["started_min"].min()
    label_offset = span * 0.04 if span > 0 else 0.5

    for _, row in df.iterrows():
        if row["is_milestone"]:
            ax.plot(
                row["finished_min"], row["row_pos"],
                marker="D", markersize=16,
                color="gold",
                markeredgecolor="black", markeredgewidth=1.8,
                zorder=4,
            )
            ax.axvline(row["finished_min"], color="grey", linestyle="--", linewidth=1.2, alpha=0.7, zorder=1)
            ax.text(
                row["finished_min"] + label_offset, row["row_pos"],
                f"{row['task_id']} (goal)",
                va="center", ha="left", fontsize=9, fontweight="bold",
            )
        else:
            ax.barh(
                row["row_pos"],
                width=row["finished_min"] - row["started_min"],
                left=row["started_min"],
                height=0.6,
                color=agent_colors[row["agent_id"]],
                edgecolor="black",
                linewidth=0.5,
                zorder=2,
            )
            ax.text(
                row["finished_min"], row["row_pos"],
                f"  {row['task_id']}",
                va="center", ha="left", fontsize=8,
            )

    ax.set_yticks(df["row_pos"])
    ax.set_yticklabels(df["agent_id"])
    ax.invert_yaxis()  # first task at the top
    ax.set_xlabel("Elapsed time (minutes since experiment start)")
    ax.set_ylabel("Agent")
    ax.set_title(title or f"Task execution -- run '{run_id}'")
    ax.grid(axis="x", linestyle=":", alpha=0.5)

    # Explicitly pin the x-axis: left edge at exactly 0 (elapsed time
    # starts at the experiment's first task, no reason to pad before it),
    # right edge with a margin proportional to the span so the milestone
    # label/diamond near the end never gets clipped. Autoscaling can
    # otherwise lag behind manually-added artists (axvline/plot/barh
    # added in a loop) and clip the last points -- text labels are often
    # drawn unclipped by default and so still appear, while the
    # bar/marker silently gets cut off.
    right_margin = span * 0.08 if span > 0 else 0.5
    ax.set_xlim(0, df["finished_min"].max() + right_margin)

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

    # Legend: one swatch per agent (de-duplicated), plus a milestone marker
    # entry if the chart has any milestones.
    handles = [
        plt.Rectangle((0, 0), 1, 1, color=agent_colors[a]) for a in agents
    ]
    labels = list(agents)
    if df["is_milestone"].any():
        milestone_handle = plt.Line2D(
            [], [], marker="D", markersize=12, color="gold",
            markeredgecolor="black", markeredgewidth=1.8, linestyle="None",
        )
        handles.append(milestone_handle)
        labels.append("Milestone (goal)")
    ax.legend(handles, labels, loc="upper left", bbox_to_anchor=(1.01, 1), title="Agent")

    fig.tight_layout()

    fig.savefig(run_id + '_gantt.png', dpi=150, bbox_inches="tight")
    fig.savefig(run_id + '_gantt.pdf', dpi=150, bbox_inches="tight")

    return fig


def main():
    parser = argparse.ArgumentParser(description="Plot a Gantt chart from task_execution.csv")
    parser.add_argument("csv_path", help="Path to task_execution.csv")
    parser.add_argument("--run-id", default=None, help="Which run_id to plot (default: most recent)")
    parser.add_argument("--title", default=None, help="Custom chart title")
    args = parser.parse_args()

    df, run_id = load_execution_data(args.csv_path, run_id=args.run_id)
    plot_gantt(df, run_id, title=args.title)


if __name__ == "__main__":
    main()
