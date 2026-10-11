// Read-only construction-boundary witness. No private backend representation.
#pragma once
#include <cstdint>
#include <map>
#include <set>
#include <iterator>
#include <algorithm>
namespace workcell {
template<class Node> struct Bindings {
  struct Binding {std::size_t physicsId; const Node *node;};
  std::map<std::uint64_t, Binding> entries;
  bool valid=true;
  bool Bind(std::uint64_t entity,std::size_t physicsId,
            const std::set<const Node*> &before,const std::set<const Node*> &after) {
    std::set<const Node*> added,removed;
    std::set_difference(after.begin(),after.end(),before.begin(),before.end(),std::inserter(added,added.end()));
    std::set_difference(before.begin(),before.end(),after.begin(),after.end(),std::inserter(removed,removed.end()));
    if(!valid||added.size()!=1||!removed.empty()||entries.count(entity))return valid=false;
    entries.emplace(entity,Binding{physicsId,*added.begin()});return true;
  }
  bool Complete(const std::map<std::uint64_t,std::size_t> &identities,
                const std::set<const Node*> &nodes) const {
    if(!valid||entries.empty()||entries.size()!=identities.size()||entries.size()!=nodes.size())return false;
    std::set<const Node*> seen;
    for(const auto &[id,b]:entries) {
      auto i=identities.find(id);
      if(i==identities.end()||i->second!=b.physicsId||!nodes.count(b.node)||!seen.insert(b.node).second)return false;
    }
    return seen==nodes;
  }
};
}
