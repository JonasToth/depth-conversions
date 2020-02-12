#!/usr/bin/env python3

import argparse
import configparser
import glob
import logging as log
from os import path
from pathlib import Path
import subprocess
import sys


__l = log.getLogger(__file__)
__cmd_prefix = ['']
__cmds = {
    'filter': 'depth_filter',
}


def arguments():
    parser = argparse.ArgumentParser(
        description='Run the transformation from depth image to feature image')
    parser.add_argument('-c', '--config',
                        help='Path to configuration file that describes the'
                             ' pipeline')
    parser.add_argument('--command-prefix',
                        help='Define a prefix that is prepended to every'
                             ' command. This can be used to run the commands'
                             ' with \'docker run\'',
                        default='')
    return parser.parse_args()


def run_filter(in_pattern, out_pattern, config_args, source_info):
    __l.debug('Filter command: %s' % config_args['filter'])
    __l.debug('Filter arguments: %s' % config_args['arguments'])

    invocation = __cmd_prefix
    invocation.append(__cmds['filter'])
    invocation.extend(['--input', in_pattern])
    invocation.extend(['--output', out_pattern])
    invocation.extend(['--start', source_info['start']])
    invocation.extend(['--end', source_info['end']])
    invocation.append(config_args['filter'])
    invocation.extend(config_args['arguments'].split(' '))
    filtered = [s for s in invocation if len(s) > 0]
    __l.debug('Final invocation: %s' % filtered)

    __l.info('Running filter with the following command:')
    __l.info(' '.join(filtered))

    return_code = subprocess.run(filtered, shell=False, stdin=None,
                                 stdout=sys.stdout, stderr=sys.stderr)
    return_code.check_returncode()


def main():
    args = arguments()

    global __cmd_prefix
    __cmd_prefix = args.command_prefix.split(' ')
    __l.debug('Command prefix: %s' % __cmd_prefix)

    toplevel_cfg = configparser.ConfigParser()
    __l.debug('Reading configuration from file: %s' % args.config)
    toplevel_cfg.read(args.config, encoding='utf-8')

    source_data_cfg = configparser.ConfigParser()
    # The path to the source for the command is relative to the config file
    # itself.
    data_cfg_path = path.join(path.dirname(args.config),
                              toplevel_cfg['data']['source'])
    __l.debug('Source Data configuration expected here: %s' % data_cfg_path)
    source_data_cfg.read(data_cfg_path, encoding='utf-8')

    dataset_pattern = path.join(path.dirname(data_cfg_path),
                                source_data_cfg['data']['pattern'])
    __l.debug('Determined dataset pattern to: %s' % dataset_pattern)

    output_pattern = path.join(path.dirname(args.config),
                               toplevel_cfg['data']['target'])
    __l.debug('Determined output pattern to: %s' % output_pattern)

    # Create directories for the output, if they do not exist.
    dirs_to_output = path.dirname(output_pattern)
    Path(dirs_to_output).mkdir(parents=True, exist_ok=True)

    # Check if any work has to be done, if yes, do it.
    # Otherwise just return.
    start_idx = source_data_cfg.getint('data', 'start')
    end_idx = source_data_cfg.getint('data', 'end')
    # Counting is inclusive in the processing tools.
    n_elements = end_idx - start_idx + 1
    __l.debug('Expect %d elements' % n_elements)

    globbed = glob.glob(path.join(path.dirname(args.config),
                                  toplevel_cfg['data']['test_glob']))
    __l.debug('Glob detects %d elements' % len(globbed))
    if len(globbed) == n_elements:
        __l.info('Detected that the output files already exist.'
                 ' Skipping processing!')
        return
    if len(globbed) > n_elements:
        __l.error('Test expression resulted in more files then the original'
                  ' dataset has. Check you configuration! No processing!')
        return

    if 'filter' in toplevel_cfg:
        run_filter(dataset_pattern, output_pattern, toplevel_cfg['filter'],
                   source_data_cfg['data'])


if __name__ == '__main__':
    log.basicConfig(level=log.DEBUG)
    main()
