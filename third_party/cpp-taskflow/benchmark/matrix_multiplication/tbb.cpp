#include "matrix_multiplication.hpp"
#include <tbb/tbb.h>
#include <tbb/task_scheduler_init.h>

// matrix_multiplication_tbb
void matrix_multiplication_tbb(unsigned num_threads) {

  using namespace tbb;
  using namespace tbb::flow;
 
  tbb::task_scheduler_init init(num_threads);
  tbb::flow::graph G;

  std::vector<std::unique_ptr<continue_node<continue_msg>>> tasks;
  tasks.resize(4*N);

  int i=0;

  auto sync = std::make_unique<continue_node<continue_msg>>(G, [](const continue_msg&){});
  for(; i<N ; i++) {
    tasks[i] = std::make_unique<continue_node<continue_msg>>(G, 
      [=, i=i](const continue_msg&){
        for(int j=0; j<N; ++j) {
          a[i][j] = i + j;
        }
      }
    );
    make_edge(*tasks[i], *sync);
  }

  for(; i<2*N ; i++) {
    tasks[i] = std::make_unique<continue_node<continue_msg>>(G, 
      [=, i=i-N](const continue_msg&){
        for(int j=0; j<N; ++j) {
          b[i][j] = i * j;
        }
      }
    );
    make_edge(*tasks[i], *sync);
  }

  for(; i<3*N ; i++) {
    tasks[i] = std::make_unique<continue_node<continue_msg>>(G, 
      [=, i=i-2*N](const continue_msg&){
        for(int j=0; j<N; ++j) {
          c[i][j] = 0;
        }
      }
    );
    make_edge(*tasks[i], *sync);
  }

  for(; i<4*N ; i++) {
    tasks[i] = std::make_unique<continue_node<continue_msg>>(G, 
      [=, i=i-3*N](const continue_msg&){ 
        for(int j=0; j<N; ++j) {
          for(int k=0; k<N; ++k) {
            c[i][j] += a[i][k] * b[k][j];
          }
        }
      }
    );
    make_edge(*sync, *tasks[i]);
  }

  for(auto j=0; j<3*N; j++) {
    tasks[j]->try_put(continue_msg());
  }
  G.wait_for_all();

  //std::cout << reduce_sum() << std::endl;
}

std::chrono::microseconds measure_time_tbb(unsigned num_threads) {
  auto beg = std::chrono::high_resolution_clock::now();
  matrix_multiplication_tbb(num_threads);
  auto end = std::chrono::high_resolution_clock::now();
  return std::chrono::duration_cast<std::chrono::microseconds>(end - beg);
}
