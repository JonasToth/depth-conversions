#!/usr/bin/env python
import os
import subprocess
import sys
import math
import argparse
import matplotlib.pyplot as plot
import statistics as stat

tmp_file="/tmp/tmp.txt"

def analyze(Y):
  print(
    "  -> [min, max, avg] = [%f, %f, %f]" % 
    (min(Y), max(Y), stat.mean(Y))
  )

###########################################################
# regression: run
###########################################################
def run(target, method, thread, round):

  exe = target + '/' + target
  print(exe, '-m', method, '-t', thread, '-r', round)

  with open(tmp_file, "w") as ofs:
    subprocess.call(
      [exe, '-m', method, '-t', str(thread), '-r', str(round)], 
      stdout=ofs
    )

  X = []
  Y = []
  
  with open(tmp_file, "r") as ifs:
    ifs.readline()
    ifs.readline()
    for line in ifs:
      token = line.split()
      assert len(token) == 2, "output line must have exactly two numbers"
      X.append(int(token[0]))
      Y.append(float(token[1]))

  analyze(Y)

  return X, Y

###########################################################
# main function
###########################################################
def main():

  parser = argparse.ArgumentParser(description='regression')

  parser.add_argument(
    '-b', '--benchmarks',
    nargs='+',
    help='valid benchmarks: wavefront, graph_traversal',
    choices=['wavefront', 
             'graph_traversal', 
             'binary_tree', 
             'linear_chain', 
             'matrix_multiplication',
             'mnist'],
    required=True
  )

  parser.add_argument(
    '-m','--methods', 
    nargs='+', 
    help='valid methods: omp, tbb, tf', 
    default=['tf', 'tbb', 'omp'],
    choices=['tf', 'tbb', 'omp']
  )

  parser.add_argument(
    '-t', '--threads', 
    type=int,
    nargs='+',
    help='list of the number of threads',
    required=True
  )

  parser.add_argument(
    '-r', '--num_rounds',
    type=int,
    help='number of rounds to average',
    default=1
  )
  
  # parse the arguments
  args = parser.parse_args()
  
  print('benchmarks: ', args.benchmarks)
  print('threads:', args.threads)
  print('methods:', args.methods)
  print('num_rounds:', args.num_rounds)

  rows = len(args.benchmarks)
  cols = len(args.threads)

  figc = plot.rcParams["figure.figsize"][0]
  figr = plot.rcParams["figure.figsize"][1]

  plot.rcParams["figure.figsize"] = [figc*cols*0.5, figr*rows*0.5]

  fig, axes = plot.subplots(rows, cols)
  plot_index = 1

  for benchmark in args.benchmarks:
    for thread in args.threads:
      ax = plot.subplot(rows, cols, plot_index)
      for method in args.methods:
        ax = plot.title(benchmark + ' (' + str(thread) + ' threads)')
        X, Y = run(
          benchmark, method, thread, args.num_rounds
        )
        #ax.text(
        #  .5, .9, 
        #  benchmark + ' (' + str(thread) + ' threads)',
        #  horizontalalignment='center',
        #  transform=ax.transAxes
        #)
        plot.plot(X, Y, label=method)
        plot.legend()
        print(X)
        print(Y)
      plot_index = plot_index + 1

  plot.tight_layout()
  plot.savefig('result.png')
  #plot.show()
  plot.close(fig)

# run the main entry
if __name__ == "__main__":
  main()



