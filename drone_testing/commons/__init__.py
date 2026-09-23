"""Track detection and the track-following law, AS FLOWN BY track_traversal_node.

Copied verbatim from the commons package that node is built on:

    landing_guidance/landing_guidance/track_detector.py  -> track_detector.py
    traversal_utils/traversal_utils/track_control.py     -> track_control.py

Verbatim on purpose. The mission FSM follows the carpet track with exactly
the detector and exactly the law that node was tuned with, so behaviour on
the day matches what was flown there. Do not "improve" these two files here;
take a new copy if they change upstream.
"""
