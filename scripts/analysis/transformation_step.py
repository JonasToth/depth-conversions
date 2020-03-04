#!/usr/bin/env python3

import argparse
import configparser
import glob
import logging as log
from os.path import abspath, basename, dirname, join, exists
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
    'distribution': 'feature_performance',
    'matching': 'feature_performance',
    'recognition': 'feature_performance',
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
    __l.info('Running the following command:')
    __l.info(' '.join(filtered))

    return_code = subprocess.run(filtered, shell=False, stdin=None,
                                 stdout=sys.stdout, stderr=sys.stderr)
    if return_code.returncode < 0:
        __l.error("Command got terminated by signal %d" %
                  -return_code.returncode)
        __l.error("Invocation:\n%s" % '\n'.join(filtered))
    elif return_code.returncode > 0:
        __l.warning("Command terminated with error code!")
        __l.warning("Invocation:\n%s" % '\n'.join(filtered))

    # Because filtering is a way to create a new base-dataset, a configuration
    # file must be written for the new dataset.
    new_cfg = configparser.ConfigParser()
    new_cfg['data'] = source_info
    new_cfg['data']['pattern'] = basename(config_args['target'])
    new_cfg.write(open(join(dirname(config_args['target']),
                            config_args.get('config', 'dataset.config')), 'w'))


def path_adjustment(dictionary, keys, path):
    """
    Iterates :param: keys in the :param: dictionary and replaces each value
    with the 'abspath(join(path, dictionary[key]))'.
    """
    for key in keys:
        if key not in dictionary:
            continue
        dictionary[key] = abspath(join(dirname(path), dictionary[key]))


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
    invocation.append('detector')
    det_args = config_args.get('detector_filter', '').split(' ')
    invocation.extend(det_args)

    invocation.append(config_args['detector'])
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


def run_distribution(config_args, source_info):
    invocation = __cmd_prefix
    invocation.append(__cmds['distribution'])
    invocation.extend(['--input', source_info['pattern']])
    invocation.extend(['--output', config_args['target']])
    invocation.extend(['--start', source_info['start']])
    invocation.extend(['--end', source_info['end']])
    invocation.append('keypoint-distribution')
    invocation.extend(['--image-width', config_args['width']])
    invocation.extend(['--image-height', config_args['height']])
    invocation.extend(['--response-histo', config_args['response']])
    invocation.extend(['--size-histo', config_args['size']])
    invocation.extend(['--kp-distance-histo', config_args['kp_distance']])
    invocation.extend(['--kp-distribution-histo',
                       config_args['kp_distribution']])

    command_invocer(invocation, config_args, source_info)


def run_matching(config_args, source_info):
    invocation = __cmd_prefix
    invocation.append(__cmds['distribution'])
    invocation.extend(['--input', source_info['pattern']])
    invocation.extend(['--output', config_args['target']])
    invocation.extend(['--start', source_info['start']])
    invocation.extend(['--end', source_info['end']])
    invocation.append('matching')
    invocation.extend(['--distance-norm', config_args['norm']])
    invocation.extend(['--match-output', config_args['match_output']])
    invocation.extend(['--original-images', config_args['original_images']])
    invocation.extend(['--matched-distance-histo',
                       config_args['match_distances']])

    command_invocer(invocation, config_args, source_info)


def run_recognition(config_args, source_info):
    invocation = __cmd_prefix
    invocation.append(__cmds['recognition'])
    invocation.extend(['--input', source_info['pattern']])
    invocation.extend(['--output', config_args['target']])
    invocation.extend(['--start', source_info['start']])
    invocation.extend(['--end', source_info['end']])
    invocation.append('recognition-performance')
    invocation.extend(['--depth-image', config_args['depth_images']])
    invocation.extend(['--pose-file', source_info['pose']])
    invocation.extend(['--intrinsic', source_info['intrinsic']])
    if 'mask' in source_info:
        invocation.extend(['--mask', source_info['mask']])
    invocation.extend(['--match-norm', config_args['norm']])
    invocation.extend(['--keypoint-distance-threshold',
                       config_args['distance_threshold']])

    if 'backprojection' in config_args:
        invocation.extend(['--backprojection', config_args['backprojection']])
        invocation.extend(['--orig-images', config_args['original_images']])

    invocation.extend(['--backprojection-selected-histo',
                       config_args['backprojection_selected_histo']])
    invocation.extend(['--relevant-elements-histo',
                       config_args['relevant_elements_histo']])
    invocation.extend(['--true-positive-histo',
                       config_args['true_positive_histo']])
    invocation.extend(['--false-positive-histo',
                       config_args['false_positive_histo']])
    invocation.extend(['--true-positive-distance-histo',
                       config_args['true_positive_distance_histo']])
    invocation.extend(['--false-positive-distance-histo',
                       config_args['false_positive_distance_histo']])

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

    if not exists(args.config):
        __l.error('Configuration path \'%s\' does not exist!' % args.config)
        sys.exit(1)

    toplevel_cfg = configparser.ConfigParser()
    __l.debug('Reading configuration from file: %s' % args.config)
    toplevel_cfg.read(args.config, encoding='utf-8')

    # Substitute paths in the configurations with absolute paths.
    path_adjustment(toplevel_cfg['data'], ['target', 'test_glob', 'source'],
                    args.config)
    __l.debug('Source Data configuration expected here: %s' %
              toplevel_cfg['data']['source'])

    if not exists(toplevel_cfg['data']['source']):
        __l.error('Configuration path \'%s\' does not exist!' %
                  toplevel_cfg['data']['source'])
        sys.exit(1)

    source_data_cfg = configparser.ConfigParser()
    source_data_cfg.read(toplevel_cfg['data']['source'], encoding='utf-8')

    # Substitute paths in source-configuration as well.
    path_adjustment(source_data_cfg['data'],
                    ['pattern', 'pose', 'mask', 'intrinsic'],
                    toplevel_cfg['data']['source'])

    # Create directories for the output, if they do not exist.
    dirs_to_output = dirname(toplevel_cfg['data']['target'])
    Path(dirs_to_output).mkdir(parents=True, exist_ok=True)

    if not args.force:
        n_elements = 0
        provided_count = int(toplevel_cfg['data'].get('expected_elements', -1))
        if provided_count != -1:
            n_elements = provided_count
        else:
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
                     ' Skipping processing of %s!' % args.config)
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

    elif 'distribution' in toplevel_cfg:
        toplevel_cfg['distribution']['target'] = toplevel_cfg['data']['target']
        path_adjustment(toplevel_cfg['distribution'],
                        ['response', 'size', 'kp_distance', 'kp_distribution'],
                        args.config)
        run_distribution(toplevel_cfg['distribution'], source_data_cfg['data'])

    elif 'matching' in toplevel_cfg:
        toplevel_cfg['matching']['target'] = toplevel_cfg['data']['target']
        path_adjustment(toplevel_cfg['matching'],
                        ['match_output', 'original_images', 'match_distances'],
                        args.config)
        run_matching(toplevel_cfg['matching'], source_data_cfg['data'])

    elif 'recognition' in toplevel_cfg:
        toplevel_cfg['recognition']['target'] = toplevel_cfg['data']['target']
        path_adjustment(toplevel_cfg['recognition'],
                        ['depth_images', 'backprojection', 'original_images',
                         'backprojection_selected_histo',
                         'relevant_elements_histo', 'true_positive_histo',
                         'false_positive_histo',
                         'true_positive_distance_histo',
                         'false_positive_distance_histo'],
                        args.config)
        depth_image_cfg = configparser.ConfigParser()
        depth_image_cfg.read(toplevel_cfg['recognition']['depth_images'],
                             encoding='utf-8')
        toplevel_cfg['recognition']['depth_images'] = \
            join(dirname(toplevel_cfg['recognition']['depth_images']),
                 depth_image_cfg['data']['pattern'])
        run_recognition(toplevel_cfg['recognition'], source_data_cfg['data'])

    # Creating a video from the frames helps with visualization.
    if 'video' in toplevel_cfg:
        # Replacing '%' with '%%' to mask interpolation of the ConfigParser.
        # ffmpeg uses '%' as placeholder for indices in filenames
        # (printf-syntax).
        toplevel_cfg['video']['source'] = \
            toplevel_cfg['video']['source'].replace('%', '%%%%')
        path_adjustment(toplevel_cfg['video'], ['output', 'source'],
                        args.config)
        create_video(toplevel_cfg['video'])


if __name__ == '__main__':
    log.basicConfig(format='%(asctime)s, %(levelname)s: %(message)s',
                    level=log.WARN)
    main()
