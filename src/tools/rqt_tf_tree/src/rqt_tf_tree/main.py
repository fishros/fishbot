#!/usr/bin/env python3

import sys

from rqt_gui.main import Main


def main():
    main = Main()
    sys.exit(main.main(sys.argv, standalone='rqt_tf_tree.tf_tree.RosTfTree'))


if __name__ == '__main__':
    main()
