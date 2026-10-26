"""Compatibility entry point: regenerate the current three-piece enclosure."""
import os, runpy
runpy.run_path(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'build_three_piece.py'), run_name='__main__')
