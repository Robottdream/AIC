// Behavioral checks for time-aligned motion and crossings between samples.
#include <cassert>
#include <iostream>
#include "../src/yzbot/bot_navigation/src/prediction_math.hpp"
using namespace bot_navigation;
int main()
{
  const std::vector<TimedPoint> forward{{0, 0, 0}, {1.26, 0, 1.8}};
  const std::vector<TimedPoint> evade{{0, 0, 0}, {.54, -.90, 1.8}};
  const std::vector<Forecast> crossing{{1, 1.25, 1.3, 0, -.75, .25}};
  assert(scoreForecast(forward, crossing, 0, .325, .08, .35).collision);
  assert(!scoreForecast(evade, crossing, 0, .325, .08, .35).collision);
  assert(!scoreForecast(forward, {{1, 1.25, 1.3, 0, .75, .25}}, 0, .325, .08, .35).collision);
  assert(scoreForecast({{0, 0, 0}, {0, 0, 1}}, {{2, -.6, 0, 1.2, 0, .05}},
    0, .2, .05, .35).collision);  // both endpoints clear, crossing at t=.5
  assert(!scoreForecast({{0, 0, 0}}, {{3, .8, 0, -.75, 0, .15}},
    .3, .325, .08, .35).collision);
  assert(scoreForecast({{0, 0, 0}}, {{3, .8, 0, -.75, 0, .15}},
    .4, .325, .08, .35).collision);  // compensate observation latency
  assert(scoreForecast(forward, {}, 0, .325, .08, .35).cost == 0);
  Forecast turn{4, 0, 1, 0, .5, .1, {{0,1,0}, {0,0,1}, {0,1,2.2}}};
  assert(scoreForecast({{0,0,0}, {0,0,2}}, {turn}, 0, .325, .08, .35).collision);
  turn.samples.clear();
  assert(!scoreForecast({{0,0,0}, {0,0,2}}, {turn}, 0, .325, .08, .35).collision);
  std::cout << "PASS: crossing rejection, evasive path, moving away, between-sample crossing, latency, no tracks\n";
}
