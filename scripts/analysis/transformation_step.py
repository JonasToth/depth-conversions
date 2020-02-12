#!/usr/bin/env python3

import argparse
import configparser
import glob
import logging as log
from os.path import abspath, basename, dirname, join
from pathlib import Path
import subprocess
import sys


__l = log.getLogger(__file__)
__cmd_prefix = ['']
__cmds = {
    'filter': 'depth_filter',
    'converter': 'depth2x',
    'extractor': 'feature_extractor',
    'plotter': 'keypoint_plotter',
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
    parser.add_argument('--force',
                        help='Force execution of the transformation, even'
                             ' if the files already exist.',
                        action='store_true')
    return parser.parse_args()


def command_invocer(invocation, config_args, source_info):
    filtered = [s for s in invocation if len(s) > 0]
    __l.debug('Final invocation: %s' % filtered)
    __l.info('Running filter with the following command:')
    __l.info(' '.join(filtered))

    return_code = subprocess.run(filtered, shell=False, stdin=None,
                                 stdout=sys.stdout, stderr=sys.stderr)
    return_code.check_returncode()

    # Because filtering is a way to create a new base-dataset, a configuration
    # file must be written for the new dataset.
    new_cfg = configparser.ConfigParser()
    new_cfg['data'] = source_info
    new_cfg['data']['pattern'] = basename(config_args['target'])
    new_cfg.write(open(join(dirname(config_args['target']),
                            config_args.get('config', 'dataset.config')), 'w'))


def run_filter(config_args, source_info):
    __l.debug('Filter command: %s' % config_args['filter'])
    __l.debug('Filter arguments: %s' % config_args['arguments'])

    invocation = __cmd_prefix
    invocation.append(__cmds['filter'])
    invocation.extend(['--input', source_info['pattern']])
    invocation.extend(['--output', config_args['target']])
    invocation.extend(['--start', source_info['start']])
    invocation.extend(['--end', source_info['end']])
    invocation.append(config_args['filter'])
    invocation.extend(config_args['arguments'].split(' '))

    command_invocer(invocation, config_args, source_info)


def run_converter(config_args, source_info):
    __l.debug('Converter type: %s' % config_args['type'])

    invocation = __cmd_prefix
    invocation.append(__cmds['converter'])
    invocation.extend(['--calibration', source_info['intrinsic']])
    invocation.extend(['--model', source_info['model']])
    invocation.extend(['--type', source_info['type']])
    invocation.extend(['--input', source_info['pattern']])
    invocation.extend(['--start', source_info['start']])
    invocation.extend(['--end', source_info['end']])
    invocation.append(config_args['type'])

    if config_args['type'] == 'bearing':
        add_args = config_args.get('arguments', '').split(' ')
        assert len(add_args) == 1, "Too many argument for bearing angle"
        invocation.extend(add_args)
        invocation.append(config_args['target'])
    else:
        invocation.extend(['--output', config_args['target']])
        invocation.extend(config_args.get('arguments', '').split(' '))

    command_invocer(invocation, config_args, source_info)


def run_extraction(config_args, source_info):
    __l.debug('Detector: %s' % config_args['detector'])
    __l.debug('Descriptor: %s' % config_args['descriptor'])

    invocation = __cmd_prefix
    invocation.append(__cmds['extractor'])
    invocation.extend(['--input', source_info['pattern']])
    invocation.extend(['--output', config_args['target']])
    invocation.extend(['--start', source_info['start']])
    invocation.extend(['--end', source_info['end']])
    invocation.extend(['detector', config_args['detector']])
    add_args = config_args.get('detector_args', '').split(' ')
    invocation.extend(add_args)
    invocation.extend(['descriptor', config_args['descriptor']])
    add_args = config_args.get('descriptor_args', '').split(' ')
    invocation.extend(add_args)

    command_invocer(invocation, config_args, source_info)


def run_plotting(config_args, source_info):
    invocation = __cmd_prefix
    invocation.append(__cmds['plotter'])
    invocation.extend(['--input', source_info['pattern']])
    invocation.extend(['--output', config_args['target']])
    invocation.extend(['--start', source_info['start']])
    invocation.extend(['--end', source_info['end']])
    invocation.extend(['--color', config_args.get('color', 'all')])

    command_invocer(invocation, config_args, source_info)


def create_video(config_args):
    __l.debug('Creating video \'%s\'' % config_args['output'])
    # Adjusted from
    # https://stackoverflow.com/questions/24961127/how-to-create-a-video-from-images-with-ffmpeg
    ffmpeg_call = ['ffmpeg', '-r', config_args['rate'], '-i',
                   config_args['source'], '-c:v', 'libx264',
                   '-y',  # overwrite the output-file automatically
                   '-vf', 'fps=25', '-pix_fmt', 'yuv420p',
                   config_args['output']]
    return_code = subprocess.run(ffmpeg_call, shell=False, stdin=None,
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

    # Substitute paths in the configurations with absolute paths.
    toplevel_cfg['data']['target'] = \
        abspath(join(dirname(args.config), toplevel_cfg['data']['target']))
    toplevel_cfg['data']['test_glob'] = \
        abspath(join(dirname(args.config), toplevel_cfg['data']['test_glob']))
    toplevel_cfg['data']['source'] = \
        abspath(join(dirname(args.config), toplevel_cfg['data']['source']))
    __l.debug('Source Data configuration expected here: %s' %
              toplevel_cfg['data']['source'])

    source_data_cfg = configparser.ConfigParser()
    source_data_cfg.read(toplevel_cfg['data']['source'], encoding='utf-8')

    # Substitute paths in source-configuration as well.
    if 'intrinsic' in source_data_cfg['data']:
        source_data_cfg['data']['intrinsic'] = \
            abspath(join(dirname(toplevel_cfg['data']['source']),
                         source_data_cfg['data']['intrinsic']))
    source_data_cfg['data']['pattern'] = \
        abspath(join(dirname(toplevel_cfg['data']['source']),
                     source_data_cfg['data']['pattern']))

    # Create directories for the output, if they do not exist.
    dirs_to_output = dirname(toplevel_cfg['data']['target'])
    Path(dirs_to_output).mkdir(parents=True, exist_ok=True)

    if not args.force:
        # Check if any work has to be done, if yes, do it.
        # Otherwise just return.
        start_idx = source_data_cfg.getint('data', 'start')
        end_idx = source_data_cfg.getint('data', 'end')
        # Counting is inclusive in the processing tools.
        n_elements = end_idx - start_idx + 1
        __l.debug('Expect %d elements' % n_elements)

        globbed = glob.glob(toplevel_cfg['data']['test_glob'])
        __l.debug('Glob detects %d elements' % len(globbed))
        if len(globbed) == n_elements:
            __l.info('Detected that the output files already exist.'
                     ' Skipping processing!')
            return
        if len(globbed) > n_elements:
            __l.error('Test expression resulted in more files then the'
                      ' original dataset has. Check you configuration!'
                      ' No processing!')
            return

    if 'filter' in toplevel_cfg:
        toplevel_cfg['filter']['target'] = toplevel_cfg['data']['target']
        run_filter(toplevel_cfg['filter'], source_data_cfg['data'])

    elif 'converter' in toplevel_cfg:
        toplevel_cfg['converter']['target'] = toplevel_cfg['data']['target']
        run_converter(toplevel_cfg['converter'], source_data_cfg['data'])

    elif 'extract' in toplevel_cfg:
        toplevel_cfg['extract']['target'] = toplevel_cfg['data']['target']
        run_extraction(toplevel_cfg['extract'], source_data_cfg['data'])

    elif 'plot' in toplevel_cfg:
        toplevel_cfg['plot']['target'] = toplevel_cfg['data']['target']
        run_plotting(toplevel_cfg['plot'], source_data_cfg['data'])

    # Creating a video from the frames helps with visualization.
    if 'video' in toplevel_cfg:
        toplevel_cfg['video']['output'] = \
            abspath(join(dirname(args.config),
                         toplevel_cfg['video']['output']))
        toplevel_cfg['video']['source'] = \
            abspath(join(dirname(args.config),
                         toplevel_cfg['video']['source'])).replace('%', '%%')
        create_video(toplevel_cfg['video'])


if __name__ == '__main__':
    log.basicConfig(format='%(asctime)s, %(levelname)s: %(message)s',
                    level=log.INFO)
    main()
