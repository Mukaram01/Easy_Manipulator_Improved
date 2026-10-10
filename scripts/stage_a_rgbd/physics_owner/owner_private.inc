// Included inside the matching PhysicsPrivate class; no engine re-instantiation.
public: workcell::OwnerReadback owner;
public: dart::simulation::WorldPtr OwnerWorld() {
  if(this->entityWorldMap.Map().size()!=1){owner.bindings.valid=false;return nullptr;}
  const auto id=this->entityWorldMap.Map().begin()->first;
  auto world=this->entityWorldMap.EntityCast<OwnerWorldFeatures>(id);
  if(!world){owner.bindings.valid=false;return nullptr;}
  return world->GetDartsimWorld();
}
public: std::set<const workcell::Node*> OwnerNodes() {
  std::set<const workcell::Node*> nodes;
  auto world=OwnerWorld();if(!world)return nodes;
  for(std::size_t i=0;i<world->getNumSkeletons();++i) {
    auto skeleton=world->getSkeleton(i);
    for(std::size_t j=0;j<skeleton->getNumShapeNodes();++j) {
      const auto node=skeleton->getShapeNode(j);
      if(node->has<dart::dynamics::CollisionAspect>())nodes.insert(node);
    }
  }
  return nodes;
}
public: void OwnerRecord(const UpdateInfo &info,const EntityComponentManager &ecm) {
  if(owner.output.empty()||!owner.recordSteps.count(info.iterations))return;
  Json::Value r;
  r["schema"]="workcell_instrumented_owner/v1";
  r["session"]=owner.session;r["step"]=Json::UInt64(info.iterations);
  r["stamp_ns"]=Json::Int64(std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count());
  r["dt_ns"]=Json::Int64(std::chrono::duration_cast<std::chrono::nanoseconds>(info.dt).count());
  r["backend_path"]=owner.backendPath;r["backend_class"]=owner.backendClass;
  r["authority"]="instrumented_runtime_only_not_stock_binary_or_planning_geometry";
  r["decision"]="BLOCKED_BACKEND_NUMERICS_AND_EPD_ASSOCIATION";
  auto world=OwnerWorld();const auto nodes=OwnerNodes();
  std::map<std::uint64_t,std::size_t> ids;
  for(const auto &[entity,shape]:entityCollisionMap.Map())ids.emplace(entity,shape->EntityID());
  std::set<std::uint64_t> ecmIds;
  ecm.Each<components::Collision>([&](const Entity &id,const auto*){ecmIds.insert(id);return true;});
  bool complete=world && owner.bindings.Complete(ids,nodes) && !owner.session.empty() && ecmIds.size()==ids.size();
  for(auto id:ecmIds)if(!ids.count(id))complete=false;
  r["complete"]=complete;r["shapes"]=Json::Value(Json::arrayValue);
  if(world){r["dart_frames"]=world->getSimFrames();r["dart_time_s"]=world->getTime();}
  if(complete)for(const auto &[entity,binding]:owner.bindings.entries) {
    const auto node=binding.node;Json::Value shape;
    shape["collision_id"]=Json::UInt64(entity);shape["physics_shape_id"]=Json::UInt64(binding.physicsId);
    shape["collision_name"]=scopedName(entity,ecm,"::",false);
    shape["shape_node_identity"]=workcell::Pointer(node);shape["shape_node_name"]=node->getName();
    shape["skeleton_name"]=node->getBodyNodePtr()->getSkeleton()->getName();
    shape["body_node_name"]=node->getBodyNodePtr()->getName();
    shape["mobile"]=node->getBodyNodePtr()->getSkeleton()->isMobile();
    shape["shape_type"]=node->getShape()->getType();
    if(const auto box=dynamic_cast<const dart::dynamics::BoxShape*>(node->getShape().get())) {
      Json::Value dims(Json::arrayValue);for(int i=0;i<3;++i)dims.append(box->getSize()[i]);shape["dimensions_m"]=dims;
    }
    shape["local_transform"]=workcell::Matrix(node->getRelativeTransform());
    shape["world_transform"]=workcell::Matrix(node->getWorldTransform());
    shape["body_world_transform"]=workcell::Matrix(node->getBodyNodePtr()->getWorldTransform());
    shape["ecm_world_transform"]=workcell::Matrix(math::eigen3::convert(worldPose(entity,ecm)));
    r["shapes"].append(shape);
  }
  r["contacts"]=Json::Value(Json::arrayValue);
  if(complete)for(const auto &c:world->getLastCollisionResult().getContacts()) {
    Json::Value contact;
    contact["node1"]=workcell::Pointer(c.collisionObject1->getShapeFrame()->asShapeNode());
    contact["node2"]=workcell::Pointer(c.collisionObject2->getShapeFrame()->asShapeNode());
    Json::Value point(Json::arrayValue),normal(Json::arrayValue);
    for(int i=0;i<3;++i){point.append(c.point[i]);normal.append(c.normal[i]);}
    contact["position_m"]=point;contact["normal"]=normal;
    for(const auto &[id,b]:owner.bindings.entries) {
      if(b.node==c.collisionObject1->getShapeFrame()->asShapeNode())contact["collision1"]=Json::UInt64(id);
      if(b.node==c.collisionObject2->getShapeFrame()->asShapeNode())contact["collision2"]=Json::UInt64(id);
    }
    if(!contact.isMember("collision1")||!contact.isMember("collision2"))complete=false;
    contact["depth_m"]=c.penetrationDepth;r["contacts"].append(contact);
  }
  r["complete"]=complete;
  if(!complete)r["failure_reason"]="non-bijective construction witness, missing mappings, unsupported world or incomplete inventory";
  Json::StreamWriterBuilder writer;writer["indentation"]="";writer["precision"]=17;
  std::ofstream stream(owner.output,owner.recordSteps.size()>1?std::ios::app:std::ios::out);
  stream<<Json::writeString(writer,r)<<"\n";
}
