#!/usr/bin/env python3
# main.py - Main CLI entry point for tfdiag
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

from typing import Annotated

import cyclopts

from tfdiag import tf_checker
from tfdiag import tf_lister
from tfdiag import tf_topics


app = cyclopts.App(
    name="tfdiag",
    help="TF diagnostics tool for ROS2"
)


@app.command(name="list")
def list_frames(
    dynamic: Annotated[bool, cyclopts.Parameter(name=["-d", "--dynamic"])] = False,
    static: Annotated[bool, cyclopts.Parameter(name=["-s", "--static"])] = False,
    compare: Annotated[bool, cyclopts.Parameter(name=["-c", "--compare"])] = False,
    topics: Annotated[bool, cyclopts.Parameter(name=["-t", "--topics"])] = False,
    all_reports: Annotated[bool, cyclopts.Parameter(name=["-a", "--all"])] = False,
):
    """List TF frames and check transforms.

    Args:
        dynamic: List dynamic TF frames
        static: List static TF frames
        compare: Compare pairs of TF transforms
        topics: List topics publishing TF frames
        all_reports: Run all reports (static, compare, topics)
    """
    # If --all is specified, enable all reports
    if all_reports:
        static = True
        compare = True
        topics = True

    # If no options specified, show help
    if not any([dynamic, static, compare, topics]):
        print("Usage: tfdiag list [OPTIONS]")
        print("\nOptions:")
        print("  --dynamic, -d    List dynamic TF frames")
        print("  --static, -s     List static TF frames")
        print("  --compare, -c    Compare pairs of TF transforms")
        print("  --topics, -t     List topics publishing TF frames")
        print("  --all, -a        Run all reports (static, compare, topics)")
        print("\nMultiple options can be combined.")
        return

    # Execute requested operations
    if dynamic or static:
        tf_lister.list_tf_frames(static)

    if compare:
        print()  # Add spacing between reports
        tf_checker.check_tf_pairs()

    if topics:
        print()  # Add spacing between reports
        tf_topics.monitor_tf_topics()


@app.command(name="help")
def show_help():
    """Display help message."""
    print("tfdiag - TF diagnostics tool for ROS2")
    print("\nCommands:")
    print("  list     List TF frames and check transforms")
    print("  help     Display this help message")
    print("\nFor command-specific help, use: tfdiag COMMAND --help")


def main():
    """Run the CLI."""
    import sys
    # If no arguments provided, show help
    if len(sys.argv) == 1:
        sys.argv.append("--help")
    app()


if __name__ == "__main__":
    main()
