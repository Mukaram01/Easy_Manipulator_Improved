#include "bindings.hh"
#include <cassert>
int main() {
  int a,b,c;
  workcell::Bindings<int> m;
  assert(m.Bind(7,70,{}, {&a}));
  assert(m.Complete({{7,70}}, {&a}));
  assert(!m.Complete({{7,71}}, {&a})); // wrong physics identity
  assert(!m.Complete({{7,70}}, {&a,&b})); // unmapped backend node
  assert(!m.Complete({{7,70},{8,80}}, {&a})); // missing node
  assert(!m.Complete({{7,70}}, {})); // vanished node
  assert(!m.Bind(8,80,{&a},{&a,&b,&c})); // ambiguous construction
  assert(!m.Complete({{7,70}}, {&a})); // failure remains latched
  workcell::Bindings<int> duplicate;
  assert(duplicate.Bind(7,70,{}, {&a}));
  assert(!duplicate.Bind(7,70,{&a},{&a,&b}));
  workcell::Bindings<int> missing;
  assert(!missing.Bind(7,70,{&a},{&a}));
}
