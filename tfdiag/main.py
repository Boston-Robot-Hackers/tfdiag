#!/usr/bin/env python3
# main.py - Main CLI entry point for tfdiag
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

from typing import List

import cyclopts

from tfdiag import tf_checker
from tfdiag import tf_lister


app = cyclopts.App(name="tfdiag", help="TF diagnostics tool for ROS2")


@app.command(name="list")
def list_frames(*options: str):
    """List TF frames and check transforms.

    Args:
        options: One or more of: dynamic, static, pairs
    """
    opts = set(options)
    valid = {"dynamic", "static", "pairs"}

    invalid = opts - valid
    if invalid:
        print(f"Invalid options: {', '.join(invalid)}")
        print(f"Valid options: dynamic, static, pairs")
        return

    if not opts:
        opts = {"dynamic"}

    show_static = "static" in opts
    show_dynamic = "dynamic" in opts or show_static

    if show_dynamic or show_static:
        tf_lister.list_tf_frames(show_static)

    if "pairs" in opts:
        tf_checker.check_tf_pairs()


def main():
    """Run the CLI."""
    app()


if __name__ == "__main__":
    main()
