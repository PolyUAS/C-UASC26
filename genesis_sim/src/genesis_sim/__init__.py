"""
genesis-sim - Python Zenoh Node
"""

import os
import sys
from time import sleep

import genesis as gs

gs.init()

import torch
import zenoh


def main():
    try:
        session = zenoh.open(zenoh.Config().from_env())
    except zenoh.ZError:
        session = zenoh.open(zenoh.Config())

    scene = gs.Scene(show_viewer=True)
    plane = scene.add_entity(gs.morphs.Plane())
    sphere = scene.add_entity(gs.morphs.Sphere())

    scene.build()

    for i in range(1000):
        scene.step()

    session.close()


if __name__ == "__main__":
    main()
