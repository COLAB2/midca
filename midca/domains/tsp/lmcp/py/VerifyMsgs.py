#!/usr/bin/env python3

## ===============================================================================
## Authors: AFRL/RQQA
## Organization: Air Force Research Laboratory, Aerospace Systems Directorate, Power and Control Division
##
## Copyright (c) 2017 Government of the United State of America, as represented by
## the Secretary of the Air Force.  No copyright is claimed in the United States under
## Title 17, U.S. Code.  All Other Rights Reserved.
## ===============================================================================

import argparse
from pathlib import Path
import os
import sys
import xml.dom.minidom

from midca.domains.tsp.lmcp.py.lmcp import LMCPFactory

factory = LMCPFactory.LMCPFactory()

def from_xml_str(xml_str):
    doc = xml.dom.minidom.parseString(xml_str)
    msg = factory.createObjectByName(doc.documentElement.getAttribute('Series'), doc.documentElement.localName)

    if msg:
        msg.unpackFromXMLNode(doc.documentElement, factory)

    doc.unlink()

    return msg

def valid_directory(string):
    dir_ = Path(string)
    if dir_.is_dir():
        return dir_
    else:
        raise argparse.ArgumentTypeError("\'{}\' does not exist".format(string))

def to_bool(string):
    if string.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif string.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')

def main():
    parser = argparse.ArgumentParser(description="Verify generated python serialization/deserialization functions using reference message set")
    parser.add_argument("dir", help="path to root reference message directory", type=valid_directory)
    parser.add_argument("-checksum", help="compute checksum (default", type=to_bool, default=False)
    args = parser.parse_args()

    # Gather all valid messages
    ref_files = []

    for root, _, files in os.walk(str(args.dir)):
        ref_files += [(file, root) for file in files if os.path.isfile(os.path.join(root, file))]

    width = len(str(len(ref_files)))

    # process messages
    current = 1
    for (msg_name, path) in ref_files:
        print(str('{:>0' + str(width) + '}/{:>' + str(width) + '}: checking \'{}\'').format(current, len(ref_files), os.path.join(path, msg_name)))
        current += 1

        ref_bytes = []
        with open(os.path.join(path, msg_name), 'rb') as file:
            ref_bytes = file.read()

        msg = factory.getObject(ref_bytes)
        if msg:
            # check binary roundtrip equivalence
            if LMCPFactory.packMessage(msg, args.checksum) != ref_bytes:
                print('    Error: ref bin -> msg -> bin doesn\'t match', file=sys.stderr)

            # check xml roundtrip equivalence (via reference binary)
            msg = from_xml_str(msg.toXMLStr(""))

            if msg:
                if LMCPFactory.packMessage(msg, args.checksum) != ref_bytes:
                    print('    Error: ref bin -> msg -> xml -> msg -> bin doesn\'t match', file=sys.stderr)
            else:
                print('    Error: unable to construct message object from xml (ref bin -> msg -> xml -> msg)', file=sys.stderr)

        else:
            print('    Error: unable to construct message object \'{}\''.format(msg_name), file=sys.stderr)

    return 0


if __name__ == "__main__":
    sys.exit(main())
