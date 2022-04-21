#ifndef EXECUTION_TIMER_H
#define EXECUTION_TIMER_H

#include <chrono>

template<class Resolution = std::chrono::milliseconds>
class Chrono {
public:
  using Clock = std::conditional_t<std::chrono::high_resolution_clock::is_steady,
    std::chrono::high_resolution_clock,
    std::chrono::steady_clock>;
private:
  Clock::time_point _start = Clock::now();
  Clock::time_point _step = Clock::now();

public:
  Chrono() = default;
  
  ~Chrono() = default;

  inline void stop() {
    const auto end = Clock::now();
    std::ostringstream strStream;
    strStream << "Stop Elapsed: "
      << std::chrono::duration_cast<Resolution>(end - _start).count()
      << std::endl;
    std::cout << strStream.str() << std::endl;
  }

  std::string step() {
    const auto end = Clock::now();
    std::ostringstream strStream; 
    strStream << std::chrono::duration_cast<Resolution>(end - _step).count();
    _step = Clock::now();
    return strStream.str();
  }

  std::string total() {
    const auto end = Clock::now();
    std::ostringstream strStream;
    strStream << std::chrono::duration_cast<Resolution>(end - _start).count();
    return strStream.str();
  }

  void reset() {
    _start = Clock::now();
    _step = Clock::now();
  }


}; // ExecutionTimer

#endif // EXECUTION_TIMER_H