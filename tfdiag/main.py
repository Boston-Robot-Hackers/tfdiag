#!/usr/bin/env python3
# main.py - Main CLI entry point for tfdiag
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import cyclopts

import tfdiag.tf_lister as tf_lister
import tfdiag.tf_lister_accurate as tf_lister_accurate


app = cyclopts.App(name='tfdiag', help='TF diagnostics tool for ROS2')


@app.command(name='list')
def list_frames():
    """List all TF frames with both detection methods."""
    print('=' * 80)
    print('ORIGINAL IMPLEMENTATION (Topic-based approximation)')
    print('=' * 80)
    tf_lister.list_tf_frames()

    print('\n')
    print('=' * 80)
    print('ACCURATE IMPLEMENTATION (GID-based detection)')
    print('=' * 80)
    tf_lister_accurate.list_tf_frames_accurate()


def main():
    """Run the CLI."""
    app()


if __name__ == '__main__':
    main()
