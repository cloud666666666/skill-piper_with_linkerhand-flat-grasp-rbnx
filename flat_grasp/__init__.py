# SPDX-License-Identifier: Apache-2.0
"""flat_grasp — flat-palm enveloping grasp skill (Piper + LinkerHand O6).

See main.py for the pipeline. Per-package init is intentionally empty:
re-exporting `flat_grasp` here would make `python3 -m flat_grasp.main` import
main.py twice (once while resolving the package, once as __main__), running
the lifecycle twice.
"""
