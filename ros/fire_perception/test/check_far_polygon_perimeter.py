"""Compile the actual FAR classifier body with minimal geometry types (WSL)."""
from pathlib import Path
import subprocess
import tempfile

source = Path('/root/catkin_ws/src/far_planner/far_planner-melodic-noetic/src/far_planner/src/contour_graph.cpp').read_text()
start = source.index('bool ContourGraph::IsAPillarPolygon(')
end = source.index('\n}', start) + 2
body = source[start:end]
prefix = '''
#include <cmath>
#include <vector>
#include <algorithm>
#include <cassert>
#include <iostream>
struct Point3D { float x,y; };
using PointStack=std::vector<Point3D>;
struct ContourGraph {
  struct {float kPillarPerimeter=22.4f;} ctgraph_params_;
  bool IsAPillarPolygon(const PointStack&,float&);
};
'''
tests = '''
int main() {
  ContourGraph graph; float perimeter;
  PointStack wall{{0,0},{1.45f,0},{1.45f,19.45f},{0,19.45f}};
  for(int reverse=0; reverse<2; ++reverse) {
    for(int i=0;i<4;++i) {
      assert(!graph.IsAPillarPolygon(wall,perimeter));
      assert(std::abs(perimeter-41.8f)<0.001f);
      std::rotate(wall.begin(),wall.begin()+1,wall.end());
    }
    std::reverse(wall.begin(),wall.end());
  }
  wall.push_back(wall.front());
  assert(!graph.IsAPillarPolygon(wall,perimeter));
  assert(std::abs(perimeter-41.8f)<0.001f);
  assert(graph.IsAPillarPolygon({{0,0},{1,0},{1,1},{0,1}},perimeter));
  assert(std::abs(perimeter-4)<0.001f);
  assert(graph.IsAPillarPolygon({},perimeter));
  std::cout << "PASS: closed perimeter, 8 vertex orderings, repeated endpoint, small pillar, empty contour\\n";
}
'''
with tempfile.TemporaryDirectory(prefix='far_polygon_test_') as tmp:
    cpp = Path(tmp) / 'check.cpp'
    cpp.write_text(prefix + body + tests)
    exe = str(Path(tmp) / 'check')
    subprocess.run(['g++', '-std=c++14', str(cpp), '-o', exe], check=True)
    subprocess.run([exe], check=True)
